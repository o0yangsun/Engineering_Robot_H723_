//
// Created by 刘嘉俊 on 26-3-6.
//

#ifndef CTRBOARD_H7_ALL_KINEMATICSDHDSP_H
#define CTRBOARD_H7_ALL_KINEMATICSDHDSP_H

#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include <stdbool.h>

// 定义PI免去一次浮点计算
#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2      1.57079632679489661923f
#endif
// 弧度转角度常量
#define RAD_TO_DEG_FACTOR  57.295777754771045f

// DSP库加速运算
#define cosf(x) arm_cos_f32(x)
#define sinf(x) arm_sin_f32(x)

#define Matrix arm_matrix_instance_f32 // 矩阵描述结构体（存储了矩阵行列数和数据区指针）
#define Matrix_Init arm_mat_init_f32      // 矩阵初始化，一块float数组会被包装成矩阵
#define Matrix_Add arm_mat_add_f32        // 矩阵加法
#define Matrix_Subtract arm_mat_sub_f32   // 矩阵减法
#define Matrix_Multiply arm_mat_mult_f32  // 矩阵乘法
#define Matrix_Transpose arm_mat_trans_f32// 矩阵转置
#define Matrix_Inverse arm_mat_inverse_f32// 矩阵求逆




#define POSITION_TOLERANCE  (1e-2f)
#define ORIENTATION_TOLERANCE (1e-2f)

typedef struct {
    // 此处按理来说应该是theta，但由于机械编码器安装错位或者反相，可能导致多一个或者少一个PI/2的偏置补偿
    // 所以此处先写theta_offset，则公式为：theta = joint[i] + arm_sdh_table[i].theta_offset;
    // 在公式中theta是参与运动学解算的参数，而joint是我们最终要求解的关节角度，当偏置为0时，theta就是所求的关节角度
    float theta_offset;
    float d;       // DH参数中的d（连杆偏移）
    float a;       // DH参数中的a（连杆长度）
    float alpha;   // DH参数中的α（连杆扭转角）
} SDH_Param_t;


typedef struct {
    float X, Y, Z;   // 位置
    float ROLL, PITCH, YAW;   // 欧拉角，弧度
    float roll_deg, pitch_deg, yaw_deg; // 欧拉角，角度
    float R[9];      // 3x3旋转矩阵，按行存储
    bool hasR;  // 校验是否保存了旋转矩阵
} Pose6D_t;

typedef struct {
    float q[6];
    int valid;          // 1:可用 0:不可用
    int flag[3];        // [0] q1段, [1] q23段, [2] q456段; 1正常 0无解 -1奇异退化
    float cost;         // 最优解评分
} IKCandidate_t;

typedef struct {
    float min[6];
    float max[6];
} JointLimit_t;

void Pose6D_SetFromXYZ_RollYawPitch(Pose6D_t *pose,
                                    float x, float y, float z,
                                    float roll, float yaw, float pitch);

bool SDH_FK_ToPose6D(const SDH_Param_t table[6], const float q[6], Pose6D_t *pose);

#define IK_MAX_SOLUTIONS 8
extern const SDH_Param_t arm_sdh_table[6];
extern const JointLimit_t joint_limit;

void Pose_AddOffsetInFrame6(const Pose6D_t *pose_6,
                            const float offset_6[3],
                            Pose6D_t *pose_out);

void IK_Solve_Q123_All(const SDH_Param_t *table,
                       const float pw[3],
                       const float q_last[6],
                       float q123_set[4][3],
                       int flag_q1[2],
                       int flag_q23[4],
                       int *count_q123);

int IK_Check_JointLimit(const float q[6], const JointLimit_t *limit);

int IK_Solution_Validate(const SDH_Param_t *table,
                         const float q[6],
                         const Pose6D_t *target,
                         const float wrist_offset[3],
                         float pos_tol,
                         float ori_tol);

void IK_Select_Best(IKCandidate_t cand[],
                    int cand_count,
                    const float q_last[6],
                    float q_best[6]);

bool IK_Evaluate_Solution_Error(const SDH_Param_t *table,
                                const float q[6],
                                const float wrist_offset[3],
                                const Pose6D_t *target,
                                float *pos_err,
                                float *ori_err);

void IK_Generate_Candidates(const float q123_set[4][3], int count_q123,
                            const float q456_set[2][3], int count_q456,
                            IKCandidate_t cand[IK_MAX_SOLUTIONS],
                            int *cand_count);

int IK_Solve_All(const SDH_Param_t *table,
                 const float wrist_offset[3],
                 const Pose6D_t *target,
                 const float q_last[6],
                 const JointLimit_t *limit,
                 float pos_tol,
                 float ori_tol,
                 float q_best[6],
                 IKCandidate_t cand_out[IK_MAX_SOLUTIONS],
                 int *cand_count_out);


void Kinematic_MapInit(void);


bool SDH_FK_FromEnc(const float q_enc[6], Pose6D_t *pose);


int IK_Solve_All_Enc(const float wrist_offset[3],
                     const Pose6D_t *target,
                     const float q_last_enc[6],
                     float pos_tol,
                     float ori_tol,
                     float q_best_enc[6],
                     IKCandidate_t cand_out[IK_MAX_SOLUTIONS],
                     int *cand_count_out);
#endif //CTRBOARD_H7_ALL_KINEMATICSDHDSP_H
