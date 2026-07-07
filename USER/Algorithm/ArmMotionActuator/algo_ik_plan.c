#include "algo_ik_plan.h"

#include "usart_task.h"
#include "cmsis_os.h"

/* ---------------------------- 模块内部类型定义 ---------------------------- */
typedef struct
{
    /* 最新pose命令 */
    Pose6D_t target_pose;

    /* 初始TCP原点（基坐标系下的绝对位姿） */
    Pose6D_t origin_tcp_pose;
    uint8_t  origin_valid;

    /* pose命令状态 */
    uint8_t  cmd_pending;
    uint32_t cmd_seq;
    uint32_t pending_seq;
    uint32_t last_ik_seq;
    uint32_t last_start_seq;
    AlgoCmdState_e cmd_state;

    /* IK结果 */
    float q_ik_target[JOINT_NUM];
    IKCandidate_t ik_cands[IK_MAX_SOLUTIONS];
    int ik_cand_count;

    /* TCP偏置 */
    float wrist_offset[3];
} AlgoPosePlan_t;

typedef struct
{
    JointMoveJPlanner_t movej;
    JointLimit_t joint_limit;

    float q_target[JOINT_NUM];
    float vmax[JOINT_NUM];
    float amax[JOINT_NUM];

    uint8_t movej_cmd_pending;

    float time_scale;
    uint32_t track_print_div;
} AlgoPlannerRuntime_t;

typedef struct
{
    AlgoFeedback_t feedback;
    AlgoPosePlan_t pose_plan;
    AlgoPlannerRuntime_t planner;
} AlgorithmContext_t;

/* ---------------------------- 模块内部静态变量 ---------------------------- */
static AlgorithmContext_t g_algo;

#define PLAN_VMAX 3.0f
#define PLAN_AMAX 1.0f
static float wrist_toll_offset[3] = {0.0f, 0.0f, (0.016f + 0.114f)}; // 腕部偏置与夹爪工具偏置

//static const PoseTestPoint_t g_pose_test_list[] =
//        {
//                {0.267902f, -0.05765f, -0.245f, -3.1415926f, 0.0f, 0.0f, "P0_zero"},
//                {0.230000f, -0.10000f, -0.220f, -3.1415926f, 0.0f, 0.0f, "P1"},
//                {0.180000f, -0.12000f, -0.180f, -3.1415926f, 0.0f, 0.0f, "P2"},
//                {0.220000f,  0.02000f, -0.200f, -3.1415926f, 0.0f, 0.0f, "P3"},
//        };
//#define POSE_TEST_COUNT  ((uint32_t)(sizeof(g_pose_test_list) / sizeof(g_pose_test_list[0])))

/* ---------------------------- 模块内部函数声明 ---------------------------- */
static float update_time_scale(const JointMoveJPlanner_t *jp,
                               const float q_fb[JOINT_NUM],
                               const float v_fb[JOINT_NUM],
                               float scale_prev);

static bool MoveJ_StartFromCurrentFeedback(void);
static void Algorithm_HandlePoseCommand(void);
static void Algorithm_HandleMoveJStart(void);
static void Algorithm_HandlePlannerStep(float dt);

/* ---------------------------- 模块内部函数实现 ---------------------------- */
static float update_time_scale(const JointMoveJPlanner_t *jp,
                               const float q_fb[JOINT_NUM],
                               const float v_fb[JOINT_NUM],
                               float scale_prev)
{
    const float q_tol[JOINT_NUM] = {0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f};   // rad
    const float v_tol[JOINT_NUM] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};      // rad/s

    const float lambda = 0.3f;      // 速度误差权重
    const float k = 1.5f;           // 收缩强度
    const float scale_min = 0.15f;  // 最小时间缩放
    const float alpha = 0.30f;      // 一阶滤波系数

    float eq_n = 0.0f;
    float ev_n = 0.0f;
    float z, scale_raw, scale;

    for (int i = 0; i < JOINT_NUM; i++)
    {
        float eq = fabsf(jp->q_ref[i] - q_fb[i]) / q_tol[i];
        float ev = fabsf(jp->v_ref[i] - v_fb[i]) / v_tol[i];

        if (eq > eq_n) eq_n = eq;
        if (ev > ev_n) ev_n = ev;
    }

    z = eq_n + lambda * ev_n;
    scale_raw = scale_min + (1.0f - scale_min) / (1.0f + k * z);
    scale = scale_prev + alpha * (scale_raw - scale_prev);//低通滤波

    if (scale < scale_min) scale = scale_min;
    if (scale > 1.0f)      scale = 1.0f;

    return scale;
}

static bool MoveJ_StartFromCurrentFeedback(void)
{
    float q_target_local[JOINT_NUM];

    if (!g_algo.feedback.valid)
    {
        USART7_DebugPrintf("[MoveJ] feedback invalid\r\n");
        return false;
    }

    memcpy(q_target_local, g_algo.planner.q_target, sizeof(q_target_local));

    /* 测试阶段：4/5/6轴锁住；如果后面需要放开，删除下面三行即可 */
//    q_target_local[3] = g_algo.feedback.q_fb[3];
//    q_target_local[4] = g_algo.feedback.q_fb[4];
//    q_target_local[5] = g_algo.feedback.q_fb[5];

    if (!JointMoveJ_Start(&g_algo.planner.movej,
                          g_algo.feedback.q_fb,
                          q_target_local,
                          g_algo.planner.vmax,
                          g_algo.planner.amax,
                          &g_algo.planner.joint_limit))
    {
        USART7_DebugPrintf("[MoveJ] start failed\r\n");
        return false;
    }

    USART7_DebugPrintf("[MoveJ] start ok, total_time=%.3f\r\n", g_algo.planner.movej.prof.total_time);


    return true;
}

static void Algorithm_HandlePoseCommand(void)
{
    Pose6D_t pose_local;
    uint32_t pose_seq_local = 0;

    if (!g_algo.pose_plan.cmd_pending || !g_algo.feedback.valid)
    {
        return;
    }

    taskENTER_CRITICAL();
    if (g_algo.pose_plan.cmd_pending)
    {
        pose_local = g_algo.pose_plan.target_pose;
        pose_seq_local = g_algo.pose_plan.pending_seq;
        g_algo.pose_plan.cmd_pending = 0;
    }
    taskEXIT_CRITICAL();

    if (pose_seq_local == 0)
    {
        return;
    }

    USART7_DebugPrintf("[Pose %lu] target = xyz(%.4f, %.4f, %.4f) ryp(%.4f, %.4f, %.4f)\r\n",
                       (unsigned long)pose_seq_local,
                       pose_local.X, pose_local.Y, pose_local.Z,
                       pose_local.ROLL, pose_local.YAW, pose_local.PITCH);

    memset(g_algo.pose_plan.ik_cands, 0, sizeof(g_algo.pose_plan.ik_cands));
    g_algo.pose_plan.ik_cand_count = 0;

    if (IK_Solve_All_Enc(g_algo.pose_plan.wrist_offset,
                         &pose_local,
                         g_algo.feedback.q_fb,
                         POSITION_TOLERANCE,
                         ORIENTATION_TOLERANCE,
                         g_algo.pose_plan.q_ik_target,
                         g_algo.pose_plan.ik_cands,
                         &g_algo.pose_plan.ik_cand_count))
    {
        g_algo.pose_plan.last_ik_seq = pose_seq_local;
        g_algo.pose_plan.cmd_state = ALGO_CMD_IK_OK;

        USART7_DebugPrintf("[Pose %lu] IK ok, cand=%d\r\n",
                           (unsigned long)pose_seq_local,
                           g_algo.pose_plan.ik_cand_count);
        USART7_DebugPrintf("[Pose %lu] qik= %.3f %.3f %.3f %.3f %.3f %.3f\r\n",
                           (unsigned long)pose_seq_local,
                           g_algo.pose_plan.q_ik_target[0], g_algo.pose_plan.q_ik_target[1], g_algo.pose_plan.q_ik_target[2],
                           g_algo.pose_plan.q_ik_target[3], g_algo.pose_plan.q_ik_target[4], g_algo.pose_plan.q_ik_target[5]);

        memcpy(g_algo.planner.q_target,
               g_algo.pose_plan.q_ik_target,
               sizeof(g_algo.planner.q_target));

        g_algo.planner.movej_cmd_pending = 1;
    }
    else
    {
        g_algo.pose_plan.cmd_state = ALGO_CMD_IK_FAILED;

        USART7_DebugPrintf("[Pose %lu] qfb = %.4f %.4f %.4f %.4f %.4f %.4f\r\n",
                           (unsigned long)pose_seq_local,
                           g_algo.feedback.q_fb[0], g_algo.feedback.q_fb[1], g_algo.feedback.q_fb[2],
                           g_algo.feedback.q_fb[3], g_algo.feedback.q_fb[4], g_algo.feedback.q_fb[5]);
        USART7_DebugPrintf("[Pose %lu] IK failed\r\n",
                           (unsigned long)pose_seq_local);
    }
}

static void Algorithm_HandleMoveJStart(void)
{
    if (!g_algo.planner.movej_cmd_pending)
    {
        return;
    }

    if (MoveJ_StartFromCurrentFeedback())
    {
        g_algo.pose_plan.last_start_seq = g_algo.pose_plan.last_ik_seq;
        g_algo.pose_plan.cmd_state = ALGO_CMD_MOVEJ_STARTED;
        g_algo.planner.movej_cmd_pending = 0;
        g_algo.planner.track_print_div = 0;

//        USART7_DebugPrintf("[Pose %lu] MoveJ start\r\n",
//                           (unsigned long)g_algo.pose_plan.last_start_seq);

//        USART7_DebugPrintf("[MoveJ] qfb= %.3f %.3f %.3f, qtarget= %.3f %.3f %.3f\r\n",
//                           g_algo.feedback.q_fb[0], g_algo.feedback.q_fb[1], g_algo.feedback.q_fb[2],
//                           g_algo.planner.q_target[0], g_algo.planner.q_target[1], g_algo.planner.q_target[2]);

//        USART7_DebugPrintf("[MoveJ] total_time=%.3f, moving_joint_count=%d\r\n",
//                           g_algo.planner.movej.prof.total_time,
//                           g_algo.planner.movej.moving_joint_count);
    }
    else
    {
        g_algo.pose_plan.cmd_state = ALGO_CMD_MOVEJ_START_FAILED;
        g_algo.planner.movej_cmd_pending = 0;
    }
}

static void Algorithm_HandlePlannerStep(float dt)
{
    if (g_algo.planner.movej.state == JP_RUNNING)
    {
        float dt_eff;

        if (dt <= 0.0f)
        {
            dt = 0.001f;
        }

        g_algo.planner.time_scale = update_time_scale(&g_algo.planner.movej,
                                                      g_algo.feedback.q_fb,
                                                      g_algo.feedback.v_fb,
                                                      g_algo.planner.time_scale);//根据误差动态变化缩放因子到[0.15-1]


        dt_eff = dt * g_algo.planner.time_scale;

        if (!JointMoveJ_Update(&g_algo.planner.movej, dt_eff))
        {
            USART7_DebugPrintf("[MoveJ] Update failed\r\n");
            JointMoveJ_Stop(&g_algo.planner.movej);
        }
        else
        {
            g_algo.planner.track_print_div++;//分时间隔打印信息
            if (g_algo.planner.track_print_div >= 500)
            {
                g_algo.planner.track_print_div = 0;
                USART7_DebugPrintf("[Track] qfb= %.3f %.3f %.3f %.3f %.3f %.3f| qref= %.3f %.3f %.3f %.3f %.3f %.3f\r\n",
                                   g_algo.feedback.q_fb[0], g_algo.feedback.q_fb[1], g_algo.feedback.q_fb[2],g_algo.feedback.q_fb[3], g_algo.feedback.q_fb[4], g_algo.feedback.q_fb[5],
                                   g_algo.planner.movej.q_ref[0], g_algo.planner.movej.q_ref[1], g_algo.planner.movej.q_ref[2],g_algo.planner.movej.q_ref[3], g_algo.planner.movej.q_ref[4], g_algo.planner.movej.q_ref[5]);

                USART7_DebugPrintf("[MoveJ] time_scale: %f\r\n",g_algo.planner.time_scale);
            }
        }
    }
    else if (g_algo.planner.movej.state == JP_FAULT)
    {
        USART7_DebugPrintf("[MoveJ] planner fault\r\n");
    }

}

/* ---------------------------- 对外接口实现 ---------------------------- */
void Algo_InitContext(void)
{
    Kinematic_MapInit();//初始化模型，修正机械零点和电器零点的误差和方向

    memset(&g_algo, 0, sizeof(g_algo));

    /* planner */
    JointMoveJ_Init(&g_algo.planner.movej);

    for (uint16_t i = 0; i < JOINT_NUM; i++)
    {
        g_algo.planner.joint_limit.min[i] = joint_limit.min[i];
        g_algo.planner.joint_limit.max[i] = joint_limit.max[i];
        g_algo.planner.vmax[i] = PLAN_VMAX;
        g_algo.planner.amax[i] = PLAN_AMAX;
    }

    g_algo.planner.time_scale = 1.0f;//时间缩放因子 1为正常
    g_algo.planner.track_print_div = 0;
    g_algo.planner.movej_cmd_pending = 0;

    /* pose/IK */
    g_algo.pose_plan.wrist_offset[0] = wrist_toll_offset[0];
    g_algo.pose_plan.wrist_offset[1] =  wrist_toll_offset[1];
    g_algo.pose_plan.wrist_offset[2] =  wrist_toll_offset[2];  // 腕部偏置（关节5和关节6坐标系原点的距离直线） + Tool偏置（joint6原点到工具中心点的距离）

    g_algo.pose_plan.origin_valid = 0;
    g_algo.pose_plan.cmd_state = ALGO_CMD_IDLE;
}

void Algo_SetFeedback(const AlgoFeedback_t *fb)
{
    if (fb == NULL)
    {
        memset(&g_algo.feedback, 0, sizeof(g_algo.feedback));
        return;
    }

    g_algo.feedback = *fb;


    /* 第一次拿到有效反馈时，记录初始TCP原点 */
    if (g_algo.feedback.valid && !g_algo.pose_plan.origin_valid)
    {
        Pose6D_t pose6_now;
        Pose6D_t tcp_now;

        if (SDH_FK_FromEnc(g_algo.feedback.q_fb, &pose6_now))//将角度传入,通过正运动学得出joint6新坐标系相对于基座标系得xyz三维坐标和旋转的角度
        {
            Pose_AddOffsetInFrame6(&pose6_now,
                                   g_algo.pose_plan.wrist_offset,
                                   &tcp_now);//将偏置角度转到基座标系上后加到末端地方上

            g_algo.pose_plan.origin_tcp_pose = tcp_now;
            g_algo.pose_plan.origin_valid = 1;

            USART7_DebugPrintf("[Origin] set = (%.4f, %.4f, %.4f)\r\n",
                               tcp_now.X, tcp_now.Y, tcp_now.Z);
        }
        else
        {
            USART7_DebugPrintf("[Origin] FK failed\r\n");
        }
    }
}

bool Algo_PostPoseTarget(const Pose6D_t *pose_target)
{
    Pose6D_t pose_abs;

    if (pose_target == NULL)
    {
        return false;
    }

    /* 还没建立初始TCP原点时，不接受相对坐标命令 */
    if (!g_algo.pose_plan.origin_valid)
    {
        USART7_DebugPrintf("[Pose] origin not ready\r\n");
        return false;
    }

    /* 这里把外部传入的 XYZ 解释为“相对初始TCP原点”的坐标 */
    pose_abs = *pose_target;
    pose_abs.X = g_algo.pose_plan.origin_tcp_pose.X + pose_target->X;
    pose_abs.Y = g_algo.pose_plan.origin_tcp_pose.Y + pose_target->Y;
    pose_abs.Z = g_algo.pose_plan.origin_tcp_pose.Z + pose_target->Z;

    taskENTER_CRITICAL();
    g_algo.pose_plan.target_pose = pose_abs;
    g_algo.pose_plan.cmd_seq++;
    g_algo.pose_plan.pending_seq = g_algo.pose_plan.cmd_seq;
    g_algo.pose_plan.cmd_pending = 1;
    g_algo.pose_plan.cmd_state = ALGO_CMD_PENDING;
    taskEXIT_CRITICAL();

    return true;
}

bool Algo_PostPoseTargetXYZRYP_Rad(float x, float y, float z,
                                   float roll, float yaw, float pitch)
{
    Pose6D_t pose;
    Pose6D_SetFromXYZ_RollYawPitch(&pose, x, y, z, roll, yaw, pitch);
    return Algo_PostPoseTarget(&pose);
}

void Algo_Step(float dt)
{
    Algorithm_HandlePoseCommand();//通过目标点解得出最优的一组关节目标角度
    Algorithm_HandleMoveJStart();//得出让所有关节一起运动一起停止的时间和加减速段的时间和匀速段时间和每个关节运动的速度和加速度
    Algorithm_HandlePlannerStep(dt);//通过dt得出下一进程需要发布的目标值和角度
}

void Algo_GetOutput(AlgoOutput_t *out)
{
    int i;

    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(AlgoOutput_t));

    out->planner_state = (uint8_t)g_algo.planner.movej.state;
    out->cmd_seq = g_algo.pose_plan.cmd_seq;
    out->cmd_state = (uint8_t)g_algo.pose_plan.cmd_state;

    for (i = 0; i < JOINT_NUM; i++)
    {
        out->q_fb[i] = g_algo.feedback.q_fb[i];
        out->v_fb[i] = g_algo.feedback.v_fb[i];
    }

    if ((g_algo.planner.movej.state == JP_RUNNING) ||
        (g_algo.planner.movej.state == JP_DONE))
    {
        out->valid = 1;

        for (i = 0; i < JOINT_NUM; i++)
        {
            out->q_ref[i] = g_algo.planner.movej.q_ref[i];
            out->v_ref[i] = g_algo.planner.movej.v_ref[i];
        }
    }
    else
    {
        out->valid = 0;
    }
}