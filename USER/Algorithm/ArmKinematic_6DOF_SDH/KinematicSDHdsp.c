//
// Created by 刘嘉俊 on 26-3-6.
//

#include "KinematicSDHdsp.h"
#include "usart_task.h"
#include "DMmotor_task.h"
/**  关节角θ  沿 z?轴的偏移量 d?  沿 x?轴的长度 a?  绕 x?轴的扭转角α  **/
// q[6] 要传弧度
// theta_offset 也必须是弧度
// alpha 也必须是弧度
//const SDH_Param_t arm_sdh_table[6] = {
//        {0.0f, 0.0f,   0.0f,  -M_PI_2},
//        {0.0f, 0.0f,   0.432f, 0.0f},
//        {0.0f, 0.149f, 0.02f,   -M_PI_2},
//        {0.0f, 0.433f, 0.0f,   M_PI_2},
//        {0.0f, 0.0f,   0.0f,   -M_PI_2},
//        {0.0f, 0.0f,   0.0f,   0.0f}
//};
////标准DH参数表
const SDH_Param_t arm_sdh_table[6] = {
        {0.0f,  0.0f,       0.0f,       -M_PI_2},
        {-2.617993878f,  0.0f,       0.295f,      0.0f},
        {1.0471975512f, -0.05765f,  -0.027098f,  -M_PI_2},
        {0.0f,  0.245f,     0.0f,        M_PI_2},
        {0.0f,  0.0f,       0.0f,       -M_PI_2},
        {0.0f,  0.0f,       0.0f,        0.0f}
};

const JointLimit_t joint_limit = {
        {
                MOTOR_1_MIN_LIMIT,
                MOTOR_2_MIN_LIMIT,
                MOTOR_3_MIN_LIMIT,
                MOTOR_4_MIN_LIMIT,
                MOTOR_5_MIN_LIMIT,
                MOTOR_6_MIN_LIMIT
        },
        {
                MOTOR_1_MAX_LIMIT,
                MOTOR_2_MAX_LIMIT,
                MOTOR_3_MAX_LIMIT,
                MOTOR_4_MAX_LIMIT,
                MOTOR_5_MAX_LIMIT,
                MOTOR_6_MAX_LIMIT
        }
};
/**
 * @brief 旋转矩阵转欧拉角（Z-Y-X顺序）（YAW PITCH ROLL）（YPR）
 * @param _rotationM 输入3x3旋转矩阵
 * @param _eulerAngles 输出欧拉角 [A, B, C] = [yaw, pitch, roll]（单位：弧度）
 * cc = cos(C) cb = cos(B) ca = cos(A) sc = sin(C) sb = sin(B) sa = sin(A)
 */
static void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles) {
    float A, B, C, cb;
    // 欧拉角奇异位置处理，旋转矩阵解出来可能导致欧拉角有无穷解或者无解，此时需要特殊处理
    if (fabsf(_rotationM[6]) >= 1.0f - 0.0001f) {
        if (_rotationM[6] < 0) {
            A = 0.0f;
            B = M_PI_2;
            C = atan2f(_rotationM[1], _rotationM[4]);
        } else {
            A = 0.0f;
            B = -M_PI_2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    } else {
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        cb = cosf(B);
        A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }

    _eulerAngles[0] = A; // yaw
    _eulerAngles[1] = B; // pitch
    _eulerAngles[2] = C; // roll
}

/**
 * @brief 欧拉角转旋转矩阵（Z-Y-X顺序）
 * @param _eulerAngles 输入欧拉角 [A, B, C] = [yaw, pitch, roll]（单位：弧度）
 * @param _rotationM 输出3x3旋转矩阵
 */
static void EulerAngleToRotMat(const float* _eulerAngles, float* _rotationM) {
    float ca, cb, cc, sa, sb, sc;

    cc = cosf(_eulerAngles[2]);
    cb = cosf(_eulerAngles[1]);
    ca = cosf(_eulerAngles[0]);
    sc = sinf(_eulerAngles[2]);
    sb = sinf(_eulerAngles[1]);
    sa = sinf(_eulerAngles[0]);

    _rotationM[0] = ca * cb;
    _rotationM[1] = ca * sb * sc - sa * cc;
    _rotationM[2] = ca * sb * cc + sa * sc;
    _rotationM[3] = sa * cb;
    _rotationM[4] = sa * sb * sc + ca * cc;
    _rotationM[5] = sa * sb * cc - ca * sc;
    _rotationM[6] = -sb;
    _rotationM[7] = cb * sc;
    _rotationM[8] = cb * cc;
}

/** 目标Pose6D转换函数，传入弧度制角度 **/
void Pose6D_SetFromXYZ_RollYawPitch(Pose6D_t *pose,
                                           float x, float y, float z,
                                           float roll, float yaw, float pitch)
{
    if (pose == NULL)
    {
        return;
    }

    memset(pose, 0, sizeof(Pose6D_t));

    pose->X = x;
    pose->Y = y;
    pose->Z = z;

    /* 注意：用户输入顺序是 ROLL / YAW / PITCH
     * 但结构体字段是 ROLL / PITCH / YAW
     */
    pose->ROLL  = roll;
    pose->YAW   = yaw;
    pose->PITCH = pitch;

    pose->roll_deg  = roll  * RAD_TO_DEG_FACTOR;
    pose->yaw_deg   = yaw   * RAD_TO_DEG_FACTOR;
    pose->pitch_deg = pitch * RAD_TO_DEG_FACTOR;

    /* 让IK内部自己根据欧拉角去生成旋转矩阵 */
    pose->hasR = false;
}

static float IK_Clamp(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float IK_WrapToPi(float a)
{
    while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static float IK_AngleDiff(float a, float b)
{
    return IK_WrapToPi(a - b);
}

static void IK_CopyQ6(float dst[6], const float src[6])
{
    for (int i = 0; i < 6; i++) {
        dst[i] = src[i];
    }
}

/**
 * @brief 计算两个旋转矩阵之间的最小旋转角误差（弧度）
 * @param R1 旋转矩阵1，3x3，按行存储
 * @param R2 旋转矩阵2，3x3，按行存储
 * @return 角误差，范围 [0, pi]
 */
static float IK_RotationAngleError(const float R1[9], const float R2[9])
{
    float R1T[9];
    float Rerr[9];
    float trace_val;
    float cos_theta;

    /* R1T = transpose(R1) */
    R1T[0] = R1[0];  R1T[1] = R1[3];  R1T[2] = R1[6];
    R1T[3] = R1[1];  R1T[4] = R1[4];  R1T[5] = R1[7];
    R1T[6] = R1[2];  R1T[7] = R1[5];  R1T[8] = R1[8];

    /* Rerr = R1T * R2 */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            Rerr[3 * r + c] =
                    R1T[3 * r + 0] * R2[3 * 0 + c] +
                    R1T[3 * r + 1] * R2[3 * 1 + c] +
                    R1T[3 * r + 2] * R2[3 * 2 + c];
        }
    }

    /* theta = acos((trace(Rerr)-1)/2) (旋转矩阵的夹角和矩阵的迹的公式关系)*/
    trace_val = Rerr[0] + Rerr[4] + Rerr[8];
    cos_theta = 0.5f * (trace_val - 1.0f);
    cos_theta = IK_Clamp(cos_theta, -1.0f, 1.0f);

    return acosf(cos_theta);
}

// 使用标准DH参数法构建标准齐次变换矩阵，具体公式参照机器人运动学中标准DH法齐次变换矩阵的构建
static void SDH_Build_T(const SDH_Param_t *sdh_table, float joint_rad, float T_buf[16], Matrix *T)
{
    float theta = joint_rad + sdh_table->theta_offset;
    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(sdh_table->alpha);
    float sa = sinf(sdh_table->alpha);

    // 按行存储 4x4 标准DH齐次变换矩阵
    // ct = cos_θ  st = sin_θ  ca = cos_α  sa = sin_α
    T_buf[0]  = ct;         T_buf[1]  = -st * ca;    T_buf[2]  =  st * sa;    T_buf[3]  = sdh_table->a * ct;
    T_buf[4]  = st;         T_buf[5]  =  ct * ca;    T_buf[6]  = -ct * sa;    T_buf[7]  = sdh_table->a * st;
    T_buf[8]  = 0.0f;       T_buf[9]  =  sa;         T_buf[10] =  ca;         T_buf[11] = sdh_table->d;
    T_buf[12] = 0.0f;       T_buf[13] = 0.0f;        T_buf[14] = 0.0f;        T_buf[15] = 1.0f;

    Matrix_Init(T, 4, 4, T_buf);
}

// q为指定输入的关节角度，构建六个齐次变换矩阵
static void SDH_Build_T_6(const SDH_Param_t sdh_table[6], const float q[6], float T_buf6[6][16], Matrix T[6])
{
    for (int i = 0; i < 6; i++) {
        SDH_Build_T(&sdh_table[i], q[i], T_buf6[i], &T[i]);
    }
}

// DSP矩阵运算库的arm_status枚举值	简单描述
// ARM_MATH_SUCCESS	执行成功
// ARM_MATH_ARGUMENT_ERROR	参数错误
// ARM_MATH_LENGTH_ERROR	数据缓冲区长度错误
// ARM_MATH_SIZE_MISMATCH	矩阵尺寸不兼容
// ARM_MATH_NANINF	计算出 NaN / 无穷大
// ARM_MATH_SINGULAR	矩阵奇异（无法求逆）
// ARM_MATH_TEST_FAILURE	库自测失败

// q[6] 输入：关节角； T06_buf[16] 输出：4×4 总齐次变换矩阵的数据； T06 输出：这个 4×4 矩阵的 DSP 句柄/封装对象
static bool SDH_ForwardKinematics(const SDH_Param_t table[6], const float q[6], float T06_buf[16], Matrix *T06)
{
    float T_buf[6][16];
    Matrix T[6];

    float T02_buf[16];
    float T03_buf[16];
    float T04_buf[16];
    float T05_buf[16];

    Matrix T02, T03, T04, T05;

    SDH_Build_T_6(table, q, T_buf, T);

    Matrix_Init(&T02, 4, 4, T02_buf);
    Matrix_Init(&T03, 4, 4, T03_buf);
    Matrix_Init(&T04, 4, 4, T04_buf);
    Matrix_Init(&T05, 4, 4, T05_buf);
    Matrix_Init(T06,  4, 4, T06_buf);

    if (Matrix_Multiply(&T[0], &T[1], &T02) != ARM_MATH_SUCCESS) return false;
    if (Matrix_Multiply(&T02,  &T[2], &T03) != ARM_MATH_SUCCESS) return false;
    if (Matrix_Multiply(&T03,  &T[3], &T04) != ARM_MATH_SUCCESS) return false;
    if (Matrix_Multiply(&T04,  &T[4], &T05) != ARM_MATH_SUCCESS) return false;
    if (Matrix_Multiply(&T05,  &T[5], T06)  != ARM_MATH_SUCCESS) return false;

    return true;
}

//TODO: 矩阵对角正交化，避免float浮点数运算引入的数值漂移
static float Vec3Dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static float Vec3Norm(const float v[3])
{
    return sqrtf(Vec3Dot(v, v));
}

static void Vec3Scale(float v[3], float s)
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
}

static void Vec3Sub(float out[3], const float a[3], const float b[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

static void Vec3Cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

static void Vec3NormalizeSafe(float v[3])
{
    float n = Vec3Norm(v);
    if (n > 1e-12f) {
        Vec3Scale(v, 1.0f / n);
    }
}

/*进行施密特正交化(旋转矩阵经过之前的浮点数计算，产生了小误差，现在以x为标准，重建标准正交矩阵，不要让后面运算误差继续扩大)*/
static void NormalizeRotationMatrix3x3(const float R_in[9], float R_out[9])
{
    /* 按列做 Gram-Schmidt，更符合旋转矩阵“列向量是坐标轴”的含义 */
    float x[3] = {R_in[0], R_in[3], R_in[6]};
    float y[3] = {R_in[1], R_in[4], R_in[7]};
    float z[3];

    float proj_xy;
    float y_ortho[3];

    /* 1) 先单位化第一列 */
    Vec3NormalizeSafe(x);

    /* 2) 第二列去掉在第一列上的投影，再单位化 */
    proj_xy = Vec3Dot(y, x);
    y_ortho[0] = y[0] - proj_xy * x[0];
    y_ortho[1] = y[1] - proj_xy * x[1];
    y_ortho[2] = y[2] - proj_xy * x[2];
    Vec3NormalizeSafe(y_ortho);

    /* 3) 第三列由叉乘重建，保证严格正交 */
    Vec3Cross(x, y_ortho, z);
    Vec3NormalizeSafe(z);

    /* 4) 为了进一步抑制误差，再用 z 和 x 重建 y */
    Vec3Cross(z, x, y_ortho);
    Vec3NormalizeSafe(y_ortho);

    /* 按列写回 */
    R_out[0] = x[0];       R_out[1] = y_ortho[0]; R_out[2] = z[0];
    R_out[3] = x[1];       R_out[4] = y_ortho[1]; R_out[5] = z[1];
    R_out[6] = x[2];       R_out[7] = y_ortho[2]; R_out[8] = z[2];
}

static void T06_ToPose6D(const float T06_buf[16], Pose6D_t *pose)
{
    float R_raw[9];
    float R_ortho[9];
    float euler[3];

    /* 位置 */
    pose->X = T06_buf[3];
    pose->Y = T06_buf[7];
    pose->Z = T06_buf[11];

    /* 从 T06 左上角提取原始 3x3 */
    R_raw[0] = T06_buf[0];
    R_raw[1] = T06_buf[1];
    R_raw[2] = T06_buf[2];

    R_raw[3] = T06_buf[4];
    R_raw[4] = T06_buf[5];
    R_raw[5] = T06_buf[6];

    R_raw[6] = T06_buf[8];
    R_raw[7] = T06_buf[9];
    R_raw[8] = T06_buf[10];

    /* 关键：先正交化，再作为 pose->R 输出 */
    NormalizeRotationMatrix3x3(R_raw, R_ortho);

    memcpy(pose->R, R_ortho, 9 * sizeof(float));
    pose->hasR = true;

    RotMatToEulerAngle(R_ortho, euler);

    pose->YAW   = euler[0];
    pose->PITCH = euler[1];
    pose->ROLL  = euler[2];

    pose->yaw_deg   = euler[0] * RAD_TO_DEG_FACTOR;
    pose->pitch_deg = euler[1] * RAD_TO_DEG_FACTOR;
    pose->roll_deg  = euler[2] * RAD_TO_DEG_FACTOR;
}

//
//static void T06_ToPose6D(const float T06_buf[16], Pose6D_t *pose)
//{
//    float R[9];
//    float euler[3];   // [C, B, A]，单位：弧度
//
//    // 取位置
//    pose->X = T06_buf[3];
//    pose->Y = T06_buf[7];
//    pose->Z = T06_buf[11];
//    // 取左上角3x3旋转矩阵
//    R[0] = T06_buf[0];
//    R[1] = T06_buf[1];
//    R[2] = T06_buf[2];
//
//    R[3] = T06_buf[4];
//    R[4] = T06_buf[5];
//    R[5] = T06_buf[6];
//
//    R[6] = T06_buf[8];
//    R[7] = T06_buf[9];
//    R[8] = T06_buf[10];
//
//    memcpy(pose->R, R, 9 * sizeof(float));
//    pose->hasR = true;
//
//    RotMatToEulerAngle(R, euler);
//
//    // RotMatToEulerAngle输出: euler[0]=YAW, euler[1]=PITCH, euler[2]=ROLL
//    pose->YAW = euler[0];
//    pose->PITCH = euler[1];
//    pose->ROLL = euler[2];
//    // 弧度转角度
//    pose->yaw_deg = euler[0] * RAD_TO_DEG;
//    pose->pitch_deg = euler[1] * RAD_TO_DEG;
//    pose->roll_deg = euler[2] * RAD_TO_DEG;
//}

bool SDH_FK_ToPose6D(const SDH_Param_t table[6], const float q[6], Pose6D_t *pose)
{
    float T06_buf[16];
    Matrix T06;

    if (table == NULL || q == NULL || pose == NULL) {
        return 0;
    }

    if (!SDH_ForwardKinematics(table, q, T06_buf, &T06)) {//得出6转到0的旋转矩阵
        return false;
    }

    T06_ToPose6D(T06_buf, pose);
    return true;
}


static bool IK_TargetPoseToR06Pw(const Pose6D_t *target,
                                 const float wrist_offset[3],
                                 float R06[9],
                                 float pw[3])
{
    float euler[3];
    float p06[3];
    float offset_base[3];

    if ((target == NULL) || (R06 == NULL) || (pw == NULL) || (wrist_offset == NULL)) {
        return false;
    }

    /* 1) 统一得到目标姿态旋转矩阵 R06 */
    if (target->hasR) {
        /* 直接使用用户提供的 3x3 旋转矩阵 */
        memcpy(R06, target->R, 9 * sizeof(float));
    } else {
        /* 由 Z-Y-X 欧拉角（YAW, PITCH, ROLL）转换 */
        euler[0] = target->YAW;
        euler[1] = target->PITCH;
        euler[2] = target->ROLL;
        EulerAngleToRotMat(euler, R06);
    }

    /* 2) 目标末端位置 p06 */
    p06[0] = target->X;
    p06[1] = target->Y;
    p06[2] = target->Z;

    /* 3) offset_base = R06 * wrist_offset */
    offset_base[0] = R06[0] * wrist_offset[0] + R06[1] * wrist_offset[1] + R06[2] * wrist_offset[2];
    offset_base[1] = R06[3] * wrist_offset[0] + R06[4] * wrist_offset[1] + R06[5] * wrist_offset[2];
    offset_base[2] = R06[6] * wrist_offset[0] + R06[7] * wrist_offset[1] + R06[8] * wrist_offset[2];

    /* 4) 腕心位置 pw = p06 - R06 * wrist_offset */
    pw[0] = p06[0] - offset_base[0];
    pw[1] = p06[1] - offset_base[1];
    pw[2] = p06[2] - offset_base[2];

    return true;
}


void Pose_AddOffsetInFrame6(const Pose6D_t *pose_6,
                            const float offset_6[3],
                            Pose6D_t *pose_out)
{
    float dx, dy, dz;

    *pose_out = *pose_6;

    dx = pose_6->R[0] * offset_6[0] + pose_6->R[1] * offset_6[1] + pose_6->R[2] * offset_6[2];
    dy = pose_6->R[3] * offset_6[0] + pose_6->R[4] * offset_6[1] + pose_6->R[5] * offset_6[2];
    dz = pose_6->R[6] * offset_6[0] + pose_6->R[7] * offset_6[1] + pose_6->R[8] * offset_6[2];

    pose_out->X += dx;
    pose_out->Y += dy;
    pose_out->Z += dz;
}

void IK_Solve_Q123_All(const SDH_Param_t *table,
                       const float pw[3],
                       const float q_last[6],
                       float q123_set[4][3],
                       int flag_q1[2],
                       int flag_q23[4],
                       int *count_q123)
{
    const float EPS = 1e-6f;

    int count = 0;
    int ind_arm;

    float q1_list[2];
    int q1_state[2] = {0, 0};

    if (table == NULL || pw == NULL || q123_set == NULL ||
        flag_q1 == NULL || flag_q23 == NULL || count_q123 == NULL) {
        if (count_q123) *count_q123 = 0;
        return;
    }

    /* =========================================================
     * 当前 PC 版字段语义：
     *   theta = 固定偏置 theta_offset
     *   d     = 连杆偏移 d
     *   a     = 连杆长度 a
     *   alpha = 扭转角 alpha
     *
     * 当前机器人前4轴结构：
     *   1: alpha1 = -pi/2
     *   2: alpha2 = 0
     *   3: alpha3 = -pi/2
     *   4: a4 = 0, 腕心由 d4 给出
     * ========================================================= */
    {
        float d1 = table[0].d;
        float a2 = table[1].a;
        float d2 = table[1].d;
        float a3 = table[2].a;
        float d3 = table[2].d;
        float d4 = table[3].d;

        /* 肩部偏置 */
        float ds = d2 + d3;

        /* 腕心目标 */
        float xw = pw[0];
        float yw = pw[1];
        float zw = pw[2];

        float rho2 = xw * xw + yw * yw;
        float rho  = sqrtf(rho2);

        /* =====================================================
         * 1) q1 两组解
         *
         * 由严格几何：
         *   x = c1*R - s1*ds
         *   y = s1*R + c1*ds
         *   rho^2 = R^2 + ds^2
         *
         * 得：
         *   R = ±sqrt(rho^2 - ds^2)
         * ===================================================== */
        if (rho < fabsf(ds) - EPS) //不是可到达空间
        {
            *count_q123 = 0;
            return;
        }

        if (fabsf(rho - fabsf(ds)) <= EPS)//太靠近基坐标系，形成肩部奇异有无穷个解
        {
            /* q1 奇异：两组肩型退化为一组 */
            float q1_keep = (q_last != NULL) ? q_last[0] : 0.0f;//进入奇异使用上一时刻的 J1 值（q_last[0]）作为当前解,如果无历史值,则默认取 0.0

            q1_list[0] = IK_WrapToPi(q1_keep);
            q1_list[1] = IK_WrapToPi(q1_keep);
            q1_state[0] = -1;
            q1_state[1] = -1;
        }
        else
        {
            float phi  = atan2f(yw, xw);
            float root = sqrtf(rho2 - ds * ds);//勾股定理

            /* 先求真实 theta1，再减去偏置 */
            {
                float theta1_a = phi - atan2f(ds,  root);
                float theta1_b = phi - atan2f(ds, -root);

                q1_list[0] = IK_WrapToPi(theta1_a - table[0].theta_offset);//要求的关节一的角度为基座标系和连杆1的夹角不是和末端在水平面投影到基座标系的夹角
                q1_list[1] = IK_WrapToPi(theta1_b - table[0].theta_offset);//没有奇异的话一般有两个解

                q1_state[0] = 1;
                q1_state[1] = 1;
            }
        }

        /* =====================================================
         * 2) 对每个 q1 分支，求 q2/q3 两组解
         *
         * 严格腕心方程：
         *   X = a2*cos(t2) + a3*cos(t2+t3) - d4*sin(t2+t3)
         *   Z = a2*sin(t2) + a3*sin(t2+t3) + d4*cos(t2+t3)
         *
         * 令：
         *   Lf  = sqrt(a3^2 + d4^2)
         *   psi = atan2(d4, a3)
         *
         * 则：
         *   a3*cos(u) - d4*sin(u) = Lf*cos(u + psi)
         *   a3*sin(u) + d4*cos(u) = Lf*sin(u + psi)
         *
         * 其中 u = t2 + t3
         *
         * 注意关键点：
         *   D = cos(t3 + psi)
         * 不是 cos(gamma)
         * ===================================================== */
        for (ind_arm = 0; ind_arm < 2; ++ind_arm) {//上面的公式解法与林佩群有些不同，将xz看成一个类似xy平面，通过矢量等式得出θ2、θ3
            float theta1;
            float c1, s1;
            float Rproj;
            float X, Z;
            float Lf, psi;
            float D;
            float delta;
            float theta2_a, theta2_b;
            float theta3_a, theta3_b;

            theta1 = q1_list[ind_arm] + table[0].theta_offset;
            c1 = cosf(theta1);
            s1 = sinf(theta1);

            /* 对应当前 q1 分支的平面投影长度 */
            Rproj = c1 * xw + s1 * yw;

            X = Rproj;
            Z = d1 - zw;

            Lf  = sqrtf(a3 * a3 + d4 * d4);//用到了a3cosu-d4sinu= sqart（a3?+d4?）cos(u+ψ)三角恒等变形
            psi = atan2f(d4, a3);//psi=ψ

            if (fabsf(a2) < EPS || Lf < EPS) {//结果会不稳定（溢出或产生 NaN）
                continue;
            }

            /* 关键：D = cos(t3 + psi) */
            D = (X * X + Z * Z - a2 * a2 - Lf * Lf) / (2.0f * a2 * Lf);

            if (D > 1.0f + EPS || D < -1.0f - EPS) {
                continue;
            }

            D = IK_Clamp(D, -1.0f, 1.0f);
            delta = acosf(D);

            /* 两组肘型 */
            theta3_a =  delta - psi;////得出两个θ3的解   ////接下来的解算有点看不懂,用到了比较抽象的解算方法（尾）
            theta3_b = -delta - psi;

            /* 对应两组 theta2 */
            {
                float sA = sinf(theta3_a + psi);
                float cA = cosf(theta3_a + psi);
                float sB = sinf(theta3_b + psi);
                float cB = cosf(theta3_b + psi);

                theta2_a = atan2f(Z, X) - atan2f(Lf * sA, a2 + Lf * cA);////得出两个θ2的解
                theta2_b = atan2f(Z, X) - atan2f(Lf * sB, a2 + Lf * cB);
            }

            if (fabsf(fabsf(D) - 1.0f) <= EPS)//伸直或者折叠
            {
                /* q2/q3 奇异：肘伸直/肘折叠 */
                q123_set[count][0] = IK_WrapToPi(q1_list[ind_arm]);
                q123_set[count][1] = IK_WrapToPi(theta2_a - table[1].theta_offset);
                q123_set[count][2] = IK_WrapToPi(theta3_a - table[2].theta_offset);
                flag_q23[count] = -1;
                count++;

                q123_set[count][0] = IK_WrapToPi(q1_list[ind_arm]);
                q123_set[count][1] = IK_WrapToPi(theta2_b - table[1].theta_offset);
                q123_set[count][2] = IK_WrapToPi(theta3_b - table[2].theta_offset);
                flag_q23[count] = -1;
                count++;
            } else {
                q123_set[count][0] = IK_WrapToPi(q1_list[ind_arm]);
                q123_set[count][1] = IK_WrapToPi(theta2_a - table[1].theta_offset);
                q123_set[count][2] = IK_WrapToPi(theta3_a - table[2].theta_offset);
                flag_q23[count] = 1;
                count++;

                q123_set[count][0] = IK_WrapToPi(q1_list[ind_arm]);
                q123_set[count][1] = IK_WrapToPi(theta2_b - table[1].theta_offset);
                q123_set[count][2] = IK_WrapToPi(theta3_b - table[2].theta_offset);
                flag_q23[count] = 1;
                count++;
            }
        }
    }

    flag_q1[0] = q1_state[0];
    flag_q1[1] = q1_state[1];
    *count_q123 = count;
}

static bool IK_Build_R03_From_Q123(const SDH_Param_t *table,
                                   float q1, float q2, float q3,
                                   float R03[9])
{
    float q_tmp[6] = {0.0f};
    float T_buf6[6][16];
    Matrix Tm[6];
    float T02_buf[16];
    float T03_buf[16];
    Matrix T02, T03;

    if (table == NULL || R03 == NULL) {
        return false;
    }

    /* 只给前三轴赋值，后三轴置零 */
    q_tmp[0] = q1;
    q_tmp[1] = q2;
    q_tmp[2] = q3;
    q_tmp[3] = 0.0f;
    q_tmp[4] = 0.0f;
    q_tmp[5] = 0.0f;

    /* 构造六个单节DH矩阵 A1..A6 */
    SDH_Build_T_6(table, q_tmp, T_buf6, Tm);

    /* 手动累计到 T03 = A1 * A2 * A3 */
    Matrix_Init(&T02, 4, 4, T02_buf);
    Matrix_Init(&T03, 4, 4, T03_buf);

    if (Matrix_Multiply(&Tm[0], &Tm[1], &T02) != ARM_MATH_SUCCESS) {
        return false;
    }
    if (Matrix_Multiply(&T02, &Tm[2], &T03) != ARM_MATH_SUCCESS) {
        return false;
    }

    /* 从 T03 中提取旋转矩阵 */
    R03[0] = T03_buf[0];   R03[1] = T03_buf[1];   R03[2] = T03_buf[2];
    R03[3] = T03_buf[4];   R03[4] = T03_buf[5];   R03[5] = T03_buf[6];
    R03[6] = T03_buf[8];   R03[7] = T03_buf[9];   R03[8] = T03_buf[10];

    return true;
}


static int IK_Solve_Q456_All(const float R03[9],
                             const float R06[9],
                             const float q_last[6],
                             float q456_set[2][3],
                             int flag_q456[2],
                             int *count_q456)
{
    const float EPS = 1e-6f;
    float R30[9];
    float R36[9];
    float s5_abs;

    if (R03 == NULL || R06 == NULL || q456_set == NULL ||
        flag_q456 == NULL || count_q456 == NULL) {
        return -1;
    }

    /* R30 = transpose(R03) */
    R30[0] = R03[0];  R30[1] = R03[3];  R30[2] = R03[6];
    R30[3] = R03[1];  R30[4] = R03[4];  R30[5] = R03[7];
    R30[6] = R03[2];  R30[7] = R03[5];  R30[8] = R03[8];

    /* R36 = R30 * R06 */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            R36[3 * r + c] =
                    R30[3 * r + 0] * R06[0 * 3 + c] +
                    R30[3 * r + 1] * R06[1 * 3 + c] +
                    R30[3 * r + 2] * R06[2 * 3 + c];
        }
    }

    /* 按你的腕部结构：
       R13 = R36[2] = -cos(q4)*sin(q5)
       R23 = R36[5] = -sin(q4)*sin(q5)
       R31 = R36[6] =  sin(q5)*cos(q6)
       R32 = R36[7] = -sin(q5)*sin(q6)
       R33 = R36[8] =  cos(q5)
    */

    s5_abs = sqrtf(R36[6] * R36[6] + R36[7] * R36[7]);//一般用sin来确定唯一角度

    /* ---------- 腕奇异：q5 = 0 or pi ---------- */
    if (s5_abs <= EPS) {
        float q4_keep = (q_last != NULL) ? q_last[3] : 0.0f;
        float q6_keep = (q_last != NULL) ? q_last[5] : 0.0f;

        /* 解1：固定 q4，反求 q6 */
        q456_set[0][0] = IK_WrapToPi(q4_keep);
        if (R36[8] > 0.0f) {
            /* q5 = 0 */
            q456_set[0][1] = 0.0f;
            q456_set[0][2] = atan2f(
                    sinf(q456_set[0][0]) * R36[0] - cosf(q456_set[0][0]) * R36[3],
                    cosf(q456_set[0][0]) * R36[0] + sinf(q456_set[0][0]) * R36[3]
            );
        } else {
            /* q5 = pi 或 -pi */
            q456_set[0][1] = (float)M_PI;
            q456_set[0][2] = atan2f(
                    sinf(q456_set[0][0]) * R36[0] - cosf(q456_set[0][0]) * R36[3],
                    -(cosf(q456_set[0][0]) * R36[0] + sinf(q456_set[0][0]) * R36[3])
            );
        }

        /* 解2：固定 q6，反求 q4 */
        q456_set[1][2] = IK_WrapToPi(q6_keep);
        if (R36[8] > 0.0f) {
            q456_set[1][1] = 0.0f;
            q456_set[1][0] = atan2f(
                    R36[3] * cosf(q456_set[1][2]) + R36[0] * sinf(q456_set[1][2]),
                    R36[0] * cosf(q456_set[1][2]) - R36[3] * sinf(q456_set[1][2])
            );
        } else {
            q456_set[1][1] = (float)M_PI;
            q456_set[1][0] = atan2f(
                    -(R36[3] * cosf(q456_set[1][2]) + R36[0] * sinf(q456_set[1][2])),
                    R36[3] * sinf(q456_set[1][2]) - R36[0] * cosf(q456_set[1][2])
            );
        }

        flag_q456[0] = -1;
        flag_q456[1] = -1;
    }
    else {
        /* ---------- 非奇异：两组 q5 ---------- */

        /* 解A：sin(q5) > 0 */
        q456_set[0][1] = atan2f(+s5_abs, R36[8]);
        q456_set[0][0] = atan2f(-R36[5], -R36[2]);
        q456_set[0][2] = atan2f(-R36[7],  R36[6]);

        /* 解B：sin(q5) < 0 */
        q456_set[1][1] = atan2f(-s5_abs, R36[8]);//atan2f处理的优越性表现在求解出的范围在±pi，类似arcsin 值域只有 [-π/2, π/2]
        q456_set[1][0] = atan2f( R36[5],  R36[2]);
        q456_set[1][2] = atan2f( R36[7], -R36[6]);

        flag_q456[0] = 1;
        flag_q456[1] = 1;
    }

    /* 统一归一化 */
    {
        int i;
        for (i = 0; i < 2; i++) {
            q456_set[i][0] = IK_WrapToPi(q456_set[i][0]);
            q456_set[i][1] = IK_WrapToPi(q456_set[i][1]);
            q456_set[i][2] = IK_WrapToPi(q456_set[i][2]);
        }
    }

    *count_q456 = 2;

    return 1;
}


static int Check_JointLimit_Internal(const float q[6], const JointLimit_t *limit)
{
    int i;
    if (q == NULL || limit == NULL) {
        return 0;
    }

    for (i = 0; i < 6; i++) {
        if (q[i] < limit->min[i] || q[i] > limit->max[i]) {
            USART7_DebugPrintf("[CheckJointLimit] Joint %d out of range: %.3f rad [%.3f, %.3f]\r\n",
                               i+1, q[i], limit->min[i], limit->max[i]);
            return 0;
        }
    }
    return 1;
}

int IK_Check_JointLimit(const float q[6], const JointLimit_t *limit)
{
    return Check_JointLimit_Internal(q, limit);
}


bool IK_Evaluate_Solution_Error(const SDH_Param_t *table,
                                const float q[6],
                                const float wrist_offset[3],
                                const Pose6D_t *target,
                                float *pos_err,
                                float *ori_err)
{
    Pose6D_t fk_pose_6;
    Pose6D_t fk_pose_tool;
    float target_R[9];
    float dx, dy, dz;

    if ((table == NULL) || (q == NULL) || (target == NULL) || (wrist_offset == NULL)) {
        return false;
    }

    /* 先求 T06 / 腕心位姿 */
    if (!SDH_FK_ToPose6D(table, q, &fk_pose_6)) {
        return false;
    }

    /* 再加上工具偏置，得到工具中心位姿 */
    Pose_AddOffsetInFrame6(&fk_pose_6, wrist_offset, &fk_pose_tool);

    if (target->hasR) {
        memcpy(target_R, target->R, 9 * sizeof(float));
    } else {
        float target_euler[3];
        target_euler[0] = target->YAW;
        target_euler[1] = target->PITCH;
        target_euler[2] = target->ROLL;
        EulerAngleToRotMat(target_euler, target_R);
    }

    dx = fk_pose_tool.X - target->X;
    dy = fk_pose_tool.Y - target->Y;
    dz = fk_pose_tool.Z - target->Z;

    if (pos_err) {
        *pos_err = sqrtf(dx * dx + dy * dy + dz * dz);
    }

    if (ori_err) {
        *ori_err = IK_RotationAngleError(fk_pose_tool.R, target_R);
    }

    return true;
}

/**
 * @brief 基于“位置误差 + 旋转矩阵角误差”验证逆解候选
 * @param table    SDH 参数表
 * @param q        候选关节角
 * @param target   目标位姿
 * @param pos_tol  位置容差（米）
 * @param ori_tol  姿态容差（弧度）
 * @return true 有效；false 无效
 */
static bool Validate_Solution_Internal(const SDH_Param_t *table,
                                       const float q[6],
                                       const float wrist_offset[3],
                                       const Pose6D_t *target,
                                       float pos_tol,
                                       float ori_tol)
{
    float pos_err, ori_err;

    if (!IK_Evaluate_Solution_Error(table, q, wrist_offset, target, &pos_err, &ori_err)) {
        return false;
    }

    return (pos_err < pos_tol) && (ori_err < ori_tol);
}


int IK_Solution_Validate(const SDH_Param_t *table,
                         const float q[6],
                         const Pose6D_t *target,
                         const float wrist_offset[3],
                         float pos_tol,
                         float ori_tol)
{
    return Validate_Solution_Internal(table, q, wrist_offset, target, pos_tol, ori_tol);
}

void  IK_Select_Best(IKCandidate_t cand[],
                    int cand_count,
                    const float q_last[6],
                    float q_best[6])
{
    const float w[6] = {3.0f, 3.0f, 2.0f, 1.0f, 1.0f, 1.0f};
    const float singular_penalty = 1000.0f;

    int best_idx = -1;
    float best_cost = 0.0f;
    int i, j, k;

    if (cand == NULL || q_best == NULL) {
        return;
    }

    for (i = 0; i < cand_count; i++)
    {
        if (!cand[i].valid) //经过FK回代后超过误差则valid=0
        {
            continue;
        }

        {
            float cost = 0.0f;

            for (j = 0; j < 6; j++) {
                float dq = (q_last != NULL) ? IK_AngleDiff(cand[i].q[j], q_last[j]) : cand[i].q[j];
                cost += w[j] * dq * dq;
            }
            //如果有目标点很接近奇异点则会加固定的代价值
            for (k = 0; k < 3; k++) {
                if (cand[i].flag[k] < 0) {
                    cost += singular_penalty;
                }
            }

            cand[i].cost = cost;

            if (best_idx < 0 || cost < best_cost) {
                best_idx = i;
                best_cost = cost;
            }
        }
    }

    if (best_idx >= 0) {
        IK_CopyQ6(q_best, cand[best_idx].q);
    }
}

void IK_Generate_Candidates(const float q123_set[4][3], int count_q123,
                            const float q456_set[2][3], int count_q456,
                            IKCandidate_t cand[IK_MAX_SOLUTIONS],
                            int *cand_count)
{
    int i, j;
    int idx = 0;

    if (q123_set == NULL || q456_set == NULL || cand == NULL || cand_count == NULL) {
        return;
    }

    for (i = 0; i < count_q123; i++) {
        for (j = 0; j < count_q456; j++) {
            if (idx >= IK_MAX_SOLUTIONS) {
                break;
            }

            cand[idx].q[0] = q123_set[i][0];
            cand[idx].q[1] = q123_set[i][1];
            cand[idx].q[2] = q123_set[i][2];
            cand[idx].q[3] = q456_set[j][0];
            cand[idx].q[4] = q456_set[j][1];
            cand[idx].q[5] = q456_set[j][2];

            cand[idx].valid   = 1;
            cand[idx].flag[0] = 1;
            cand[idx].flag[1] = 1;
            cand[idx].flag[2] = 1;
            cand[idx].cost    = 0.0f;
            idx++;
        }
    }

    *cand_count = idx;
}






/*
static bool IK_Solution_Validate(const SDH_Param_t table[6],
                                 const float q[6],
                                 const Pose6D_t *target,
                                 float pos_tol, float ori_tol)
{
    Pose6D_t fk_pose;

    if ((table == NULL) || (q == NULL) || (target == NULL)) {
        return false;
    }

    if (!SDH_FK_ToPose6D(table, q, &fk_pose)) {
        return false;
    }

    float target_euler[3];
    if (target->hasR) {
        RotMatToEulerAngle(target->R, target_euler);
    } else {
        target_euler[0] = target->YAW;
        target_euler[1] = target->PITCH;
        target_euler[2] = target->ROLL;
    }

    {
        float dx = fk_pose.X - target->X;
        float dy = fk_pose.Y - target->Y;
        float dz = fk_pose.Z - target->Z;
        float pos_err = sqrtf(dx * dx + dy * dy + dz * dz);

        float dyaw   = IK_AngleDiff(fk_pose.YAW,   target_euler[0]);
        float dpitch = IK_AngleDiff(fk_pose.PITCH, target_euler[1]);
        float droll  = IK_AngleDiff(fk_pose.ROLL,  target_euler[2]);
        float ori_err = sqrtf(dyaw * dyaw + dpitch * dpitch + droll * droll);

        return (pos_err < pos_tol && ori_err < ori_tol);
    }
}

*/


/* =========================================================
 * IK 主接口
 * ========================================================= */
int IK_Solve_All(const SDH_Param_t *table,
                 const float wrist_offset[3],
                 const Pose6D_t *target,
                 const float q_last[6],
                 const JointLimit_t *limit,
                 float pos_tol,
                 float ori_tol,
                 float q_best[6],
                 IKCandidate_t cand_out[IK_MAX_SOLUTIONS],
                 int *cand_count_out)
{
    float R06[9];
    float pw[3];

    float q123_set[4][3];
    int flag_q1_arm[2] = {0, 0};
    int flag_q23[4] = {0, 0, 0, 0};
    int count_q123 = 0;

    IKCandidate_t local_cand[IK_MAX_SOLUTIONS];
    int cand_count = 0;

    int i, j;

    if (table == NULL || wrist_offset == NULL || target == NULL || q_best == NULL) {
        if (cand_count_out) *cand_count_out = 0;
        return 0;
    }

    /* 1) 目标位姿 -> R06 + 腕心 pw */
    IK_TargetPoseToR06Pw(target, wrist_offset, R06, pw);//将偏置向量转到基座标系，然后用目标向量减去这个向量，然后得出腕心相对于基坐标系的xyz(求解的是腕心，目标是腕心前的工具中心)

    /* 2) 解前三轴全部分支 */
    IK_Solve_Q123_All(table, pw, q_last, q123_set, flag_q1_arm, flag_q23, &count_q123);

    if (count_q123 <= 0) {
        USART7_DebugPrintf("[IK_Solve_All] ERROR: No solution for q123 (count_q123=%d)\r\n", count_q123);
        if (cand_count_out) *cand_count_out = 0;
        return 0;
    }

    /* 3) 对每组 q123，求两组 q456，并组装成候选解 */
    for (i = 0; i < count_q123; i++) {
        float R03[9];
        float q456_set[2][3];
        int flag_q456[2] = {0, 0};
        int count_q456 = 0;

        IK_Build_R03_From_Q123(table,
                               q123_set[i][0],
                               q123_set[i][1],
                               q123_set[i][2],
                               R03);

        IK_Solve_Q456_All(R03, R06, q_last, q456_set, flag_q456, &count_q456);

        if (count_q456 <= 0) {
            continue;
        }

        for (j = 0; j < count_q456; j++) {
            if (cand_count >= IK_MAX_SOLUTIONS) {
                break;
            }

            local_cand[cand_count].q[0] = q123_set[i][0];
            local_cand[cand_count].q[1] = q123_set[i][1];
            local_cand[cand_count].q[2] = q123_set[i][2];
            local_cand[cand_count].q[3] = q456_set[j][0];
            local_cand[cand_count].q[4] = q456_set[j][1];
            local_cand[cand_count].q[5] = q456_set[j][2];

            /* q1 标志按前两组/后两组肩型对应 */
            local_cand[cand_count].flag[0] = (i < 2) ? flag_q1_arm[0] : flag_q1_arm[1];
            local_cand[cand_count].flag[1] = flag_q23[i];
            local_cand[cand_count].flag[2] = flag_q456[j];

            local_cand[cand_count].valid = 1;
            local_cand[cand_count].cost  = 0.0f;

            cand_count++;
        }
    }

    if (cand_count <= 0) {
        if (cand_count_out) *cand_count_out = 0;
        return 0;
    }

    /* 4) 关节限位过滤 */
    if (limit != NULL) {
        for (i = 0; i < cand_count; i++) {
            if (local_cand[i].valid &&
                !IK_Check_JointLimit(local_cand[i].q, limit)) {
                local_cand[i].valid = 0;
            }
        }
    }

    /* 5) FK 回代验证 */
    for (i = 0; i < cand_count; i++) {
        if (local_cand[i].valid &&
            !IK_Solution_Validate(table,
                                  local_cand[i].q,
                                  target,
                                  wrist_offset,
                                  pos_tol,
                                  ori_tol)) {
            local_cand[i].valid = 0;
        }
    }

    /* 6) 选最优解 */
    IK_Select_Best(local_cand, cand_count, q_last, q_best);

    /* 7) 输出候选解（提供分析调试） */
    if (cand_out != NULL)
    {
        for (i = 0; i < cand_count; i++)
        {
            cand_out[i] = local_cand[i];
        }
    }

    if (cand_count_out != NULL) {
        *cand_count_out = cand_count;
    }


    /* 8) 判断是否真的选到了有效解 */
    {
        int has_valid = 0;
        for (i = 0; i < cand_count; i++) {
            if (local_cand[i].valid) {
                has_valid = 1;
                break;
            }
        }
        return has_valid;
    }
}


static const float joint_sign[6] = { 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f };
static const float joint_zero[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

static void JointLimit_EncToModel(const JointLimit_t *limit_enc, //将电机编码器空间的关节限位转换到模型空间
                                  const float sign_map[6],
                                  const float q_zero[6],
                                  JointLimit_t *limit_model)
{
    int i;
    for (i = 0; i < 6; i++)
    {
        float a = sign_map[i] * (limit_enc->min[i] - q_zero[i]);
        float b = sign_map[i] * (limit_enc->max[i] - q_zero[i]);

        if (a <= b)
        {
            limit_model->min[i] = a;
            limit_model->max[i] = b;
        }
        else
        {
            limit_model->min[i] = b;
            limit_model->max[i] = a;
        }
    }
}

static void Joint_EncToModel(const float q_enc[6], float q_model[6])
{
    int i;
    for (i = 0; i < 6; i++) {
        q_model[i] = joint_sign[i] * (q_enc[i] - joint_zero[i]);//将电机反馈的角度转换成适合模型的角度，这里因为达妙电机方向居然不一样的原因，会有转换的步骤
    }
}

static void Joint_ModelToEnc(const float q_model[6], float q_enc[6])
{
    int i;
    for (i = 0; i < 6; i++) {
        q_enc[i] = joint_zero[i] + joint_sign[i] * q_model[i];
    }
}

static JointLimit_t joint_limit_model;
static bool joint_map_inited = false;

void Kinematic_MapInit(void)
{
    JointLimit_EncToModel(&joint_limit, joint_sign, joint_zero, &joint_limit_model);
    joint_map_inited = true;
}


bool SDH_FK_FromEnc(const float q_enc[6], Pose6D_t *pose)
{
    float q_model[6];

    Joint_EncToModel(q_enc, q_model);
    return SDH_FK_ToPose6D(arm_sdh_table, q_model, pose);
}


int IK_Solve_All_Enc(const float wrist_offset[3],
                     const Pose6D_t *target,
                     const float q_last_enc[6],
                     float pos_tol,
                     float ori_tol,
                     float q_best_enc[6],
                     IKCandidate_t cand_out[IK_MAX_SOLUTIONS],
                     int *cand_count_out)
{
    float q_last_model[6];
    float q_best_model[6];
    int ok;

    if (!joint_map_inited)
    {
        Kinematic_MapInit();
    }

    Joint_EncToModel(q_last_enc, q_last_model);//利用模型解算,所以这里的编码数据方向也要换成模型方向

    ok = IK_Solve_All(arm_sdh_table,
                      wrist_offset,
                      target,
                      q_last_model,
                      &joint_limit_model,
                      pos_tol,
                      ori_tol,
                      q_best_model,
                      cand_out,
                      cand_count_out);//得出一组关节组到达目标点的最优组

    if (!ok) {

        return 0;

    }

    Joint_ModelToEnc(q_best_model, q_best_enc);
    return 1;
}

