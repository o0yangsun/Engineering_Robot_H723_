/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */
#include <stdio.h>
#include "algorithm_task.h"
#include "cmsis_os.h"
#include "KalmanFilterOne.h"
#include "drv_dwt.h"
#include "msg_freertos.h"
#include "robot_task.h"
#include "usart_task.h"
#include "algo_ik_plan.h"

/* ================================ 使用说明 ===================================
 * 这版代码使用“离散点控制框架”：
 *
 * 1) 所有目标来源（测试用例 / Pose6D / XYZRYP）都先进入离散队列
 * 2) 统一由一个执行器按顺序执行：
 *      - 取一个点
 *      - 发给 Algo_PostPoseTarget()
 *      - 等 IK / MoveJ / Planner / 执行器到位
 *      - 再取下一个点
 *
 * 这样就把：
 *   测试用例
 *   Pose6D 模式
 *   XYZRYP 模式
 * 合并进了一套控制逻辑。
 *
 * --------------------------------------------------------------------------
 * 输入模式说明：
 *
 * 1) ALGO_INPUT_MODE_TEST_CASE
 *    自动循环把 g_task_test_list[] 里的测试点送入离散控制器。
 *
 * 2) ALGO_INPUT_MODE_MANUAL_API
 *    不自动生成点。外部通过：
 *      AlgorithmTask_PostPoseTarget(...)
 *      AlgorithmTask_PostPoseTargetXYZRYP_Rad(...)
 *    把点送进离散队列。
 *
 * 3) ALGO_INPUT_MODE_DEMO_XYZRYP
 *    线程启动后自动送入一个 XYZRYP 示例点（只送一次）
 *
 * 4) ALGO_INPUT_MODE_DEMO_POSE6D
 *    线程启动后自动送入一个 Pose6D 示例点（只送一次）
 *
 * --------------------------------------------------------------------------
 * 注意：
 * - 这是“离散点”框架，不适合遥控器那种连续实时跟随。
 * - 如果外部连续塞很多点，这里会按队列顺序一个一个执行。
 * - 队列满时，新点会入队失败并返回 false。
 * ========================================================================== */

/* -------------------------------- 输入模式选择 -------------------------------- */
#define ALGO_INPUT_MODE_TEST_CASE      1
#define ALGO_INPUT_MODE_MANUAL_API     2
#define ALGO_INPUT_MODE_DEMO_XYZRYP    3
#define ALGO_INPUT_MODE_DEMO_POSE6D    4
#define ALGO_INPUT_MODE_MANUAL_TRIGGER   5

/* 手动触发控制变量（调试工具可直接修改） */
volatile float g_man_x = 0.1f;
volatile float g_man_y = 0.1f;
volatile float g_man_z = 0.2f;
volatile float g_man_roll = 0.0f;
volatile float g_man_yaw = -3.1415f;
volatile float g_man_pitch = -1.57f;
volatile uint8_t g_man_trigger = 0;



/* 在这里切换输入模式 */
#define ALGO_INPUT_MODE                ALGO_INPUT_MODE_TEST_CASE

/* -------------------------------- 离散控制参数 -------------------------------- */
#define ALGO_DISCRETE_QUEUE_CAPACITY   16u
#define ALGO_DISCRETE_TIMEOUT_MS       20000u
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static dm_arm_feedback_msg_t algorithm_subscribe_arm_feedback_data;

static subscriber_t *subscribe_arm_feedback_topic;
static publisher_t *publish_movej_ref_topic;

static movej_ref_msg_t algorithm_publish_movej_ref_data;
static uint32_t movej_pub_seq = 0;

static void algorithm_topic_pub_init(void);
static void algorithm_topic_sub_init(void);
static void algorithm_topic_pub_push(void);
static void algorithm_topic_sub_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t algorithm_task_dwt = 0;   // 毫秒监测
static float algorithm_task_dt = 0;       // 线程实际运行时间dt
static float algorithm_task_delta = 0;    // 监测线程运行时间
static float algorithm_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static mat_type_t filtered_data[NUM_JOINTS]= {0};
static float angles[NUM_JOINTS] = {0}; // 从队列中读取的角度值（度数）


extern QueueHandle_t xKalmanOneQueue;
extern QueueHandle_t xControlQueue; // 队列句柄
/* -------------------------------- 算法输入输出缓存 -------------------------------- */
static AlgoOutput_t g_algo_out;
static AlgoFeedback_t g_algo_fb;

/* -------------------------------- 反馈解析相关 -------------------------------- */
#define ACTIVE_MASK        0x3Fu    /* 当前只要求 joint[012345] 反馈有效 */

static bool AlgorithmTask_BuildFeedback(AlgoFeedback_t *fb);
/* -------------------------------- 测试点相关 -------------------------------- */
typedef struct
{
    float x;
    float y;
    float z;
    float roll;
    float yaw;
    float pitch;
    const char *name;
} AlgorithmTask_TestPoint_t;

static const AlgorithmTask_TestPoint_t g_task_test_list[] =
        {
                {-0.02f, -0.2f, -0.05f, 0.0f, -3.1415926f, -1.57f, "P0_zero"},
                {0.07f, -0.2f, -0.05f, 0.0f, -3.1415926f, -1.57f, "P1"},
                {0.07f, -0.2f, 0.05f, 0.0f, -3.1415926f, -1.57f, "P2"},
                {0.0f, 0.0f, 0.0f, 0.0f, -3.1415926f, -1.57f, "P3"},
        };

#define TASK_TEST_COUNT ((uint32_t)(sizeof(g_task_test_list) / sizeof(g_task_test_list[0])))
/* -------------------------------- 离散点来源类型 -------------------------------- */
typedef enum
{
    ALGO_SRC_EXTERNAL_POSE6D = 0,
    ALGO_SRC_EXTERNAL_XYZRYP,
    ALGO_SRC_TEST_CASE,
    ALGO_SRC_DEMO_POSE6D,
    ALGO_SRC_DEMO_XYZRYP
} AlgorithmTask_CmdSource_e;
/* -------------------------------- 离散点命令定义 -------------------------------- */
typedef struct
{
    Pose6D_t pose;
    uint8_t source;
    uint32_t source_index;
    const char *name;
} AlgorithmTask_DiscreteCmd_t;

/* -------------------------------- 离散点队列 -------------------------------- */
typedef struct
{
    AlgorithmTask_DiscreteCmd_t buf[ALGO_DISCRETE_QUEUE_CAPACITY];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} AlgorithmTask_DiscreteQueue_t;

static AlgorithmTask_DiscreteQueue_t g_discrete_queue;
/* -------------------------------- 离散执行器运行时 -------------------------------- */
typedef struct
{
    uint8_t waiting_finish;
    uint32_t start_tick_ms;
    AlgorithmTask_DiscreteCmd_t active_cmd;
} AlgorithmTask_DiscreteRuntime_t;

static AlgorithmTask_DiscreteRuntime_t g_discrete_rt;

/* -------------------------------- Demo状态 -------------------------------- */
static uint8_t g_demo_sent = 0;
static uint32_t g_test_next_index = 0;

/* -------------------------------- 内部函数声明 -------------------------------- */
static bool AlgorithmTask_IsSettled3FromOutput(const AlgoOutput_t *out);

static bool AlgorithmTask_DiscreteQueuePop(AlgorithmTask_DiscreteCmd_t *cmd);
static uint16_t AlgorithmTask_DiscreteQueueCount(void);

static bool AlgorithmTask_EnqueuePose(const Pose6D_t *pose,
                                      uint8_t source,
                                      uint32_t source_index,
                                      const char *name);

static void AlgorithmTask_SourceFillStep(const AlgoFeedback_t *fb, const AlgoOutput_t *out);
static void AlgorithmTask_DiscreteExecutorStep(const AlgoFeedback_t *fb, const AlgoOutput_t *out);
static void AlgorithmTask_OnActiveCommandFinished(void);
/* -------------------------------- 对外接口 -------------------------------- */
/* 现在这两个接口不再直接调用 Algo_PostPoseTarget，而是“入离散队列” */

bool AlgorithmTask_PostPoseTarget(const Pose6D_t *pose_target)
{
    return AlgorithmTask_EnqueuePose(pose_target,
                                     ALGO_SRC_EXTERNAL_POSE6D,
                                     0,
                                     "external_pose6d");
}

bool AlgorithmTask_PostPoseTargetXYZRYP_Rad(float x, float y, float z,
                                            float roll, float yaw, float pitch)
{
    Pose6D_t pose;
    Pose6D_SetFromXYZ_RollYawPitch(&pose, x, y, z, roll, yaw, pitch);

    return AlgorithmTask_EnqueuePose(&pose,
                                     ALGO_SRC_EXTERNAL_XYZRYP,
                                     0,
                                     "external_xyzryp");
}
/* -------------------------------- 内部函数实现 -------------------------------- */
static bool AlgorithmTask_BuildFeedback(AlgoFeedback_t *fb)
{
    int i;
    uint32_t mask_got;

    if (fb == NULL)
    {
        return false;
    }

    memset(fb, 0, sizeof(AlgoFeedback_t));

    mask_got = algorithm_subscribe_arm_feedback_data.update_mask & ACTIVE_MASK;
    if (mask_got != ACTIVE_MASK)
    {
        fb->valid = 0;
        return false;
    }

    for (i = 0; i < JOINT_NUM; i++)
    {
        fb->q_fb[i] = algorithm_subscribe_arm_feedback_data.joint[i].pos_rad;
        fb->v_fb[i] = algorithm_subscribe_arm_feedback_data.joint[i].vel_rad_s;
    }

    fb->valid = 1;
    return true;
}

static bool AlgorithmTask_IsSettled3FromOutput(const AlgoOutput_t *out)
{
    const float q_tol = 0.02f;   // 约 1.15 度
    const float v_tol = 0.08f;   // rad/s

    if (out == NULL)
    {
        return false;
    }

    if (out->planner_state != JP_DONE)
    {
        return false;
    }

    if (!out->valid)
    {
        return false;
    }

    for (int i = 0; i < JOINT_NUM; i++)
    {
        float q_err = fabsf(out->q_fb[i] - out->q_ref[i]);
        float v_abs = fabsf(out->v_fb[i]);

        if (q_err > q_tol)
        {
            return false;
        }

        if (v_abs > v_tol)
        {
            return false;
        }
    }

    return true;
}

/* ---------------------------- 离散队列 ---------------------------- */

static void AlgorithmTask_DiscreteQueueInit(void)
{
    memset(&g_discrete_queue, 0, sizeof(g_discrete_queue));
}

static uint16_t AlgorithmTask_DiscreteQueueCount(void)
{
    return g_discrete_queue.count;
}

static bool AlgorithmTask_DiscreteQueuePush(const AlgorithmTask_DiscreteCmd_t *cmd)
{
    if (cmd == NULL)
    {
        return false;
    }

    if (g_discrete_queue.count >= ALGO_DISCRETE_QUEUE_CAPACITY)
    {
        return false;
    }

    g_discrete_queue.buf[g_discrete_queue.tail] = *cmd;
    g_discrete_queue.tail = (uint16_t)((g_discrete_queue.tail + 1u) % ALGO_DISCRETE_QUEUE_CAPACITY);
    g_discrete_queue.count++;
    return true;
}

static bool AlgorithmTask_DiscreteQueuePop(AlgorithmTask_DiscreteCmd_t *cmd)
{
    if (cmd == NULL)
    {
        return false;
    }

    if (g_discrete_queue.count == 0u)
    {
        return false;
    }

    *cmd = g_discrete_queue.buf[g_discrete_queue.head];
    g_discrete_queue.head = (uint16_t)((g_discrete_queue.head + 1u) % ALGO_DISCRETE_QUEUE_CAPACITY);
    g_discrete_queue.count--;
    return true;
}

static bool AlgorithmTask_EnqueuePose(const Pose6D_t *pose,
                                      uint8_t source,
                                      uint32_t source_index,
                                      const char *name)
{
    AlgorithmTask_DiscreteCmd_t cmd;
    bool ok;

    if (pose == NULL)
    {
        return false;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.pose = *pose;
    cmd.source = source;
    cmd.source_index = source_index;
    cmd.name = name;

    taskENTER_CRITICAL();
    ok = AlgorithmTask_DiscreteQueuePush(&cmd);
    taskEXIT_CRITICAL();

    if (!ok)
    {
        // USART7_DebugPrintf("[Discrete] queue full, drop cmd\r\n");
    }

    return ok;
}

float xxx = 0.07f;
float yyy = -0.20f;
float zzz = -0.05f;
/* ---------------------------- 目标来源填充 ---------------------------- */
/* 根据当前输入模式，把“目标来源”统一送进离散队列 */

static void AlgorithmTask_SourceFillStep(const AlgoFeedback_t *fb, const AlgoOutput_t *out)
{
    (void)out;

    if ((fb == NULL) || (!fb->valid))
    {
        return;
    }

#if (ALGO_INPUT_MODE == ALGO_INPUT_MODE_TEST_CASE)
    if ((!g_discrete_rt.waiting_finish) && (AlgorithmTask_DiscreteQueueCount() == 0u))
    {
        Pose6D_t pose;
        const AlgorithmTask_TestPoint_t *pt = &g_task_test_list[g_test_next_index];

        Pose6D_SetFromXYZ_RollYawPitch(&pose,
                                       pt->x, pt->y, pt->z,
                                       pt->roll, pt->yaw, pt->pitch);

        if (AlgorithmTask_EnqueuePose(&pose,
                                      ALGO_SRC_TEST_CASE,
                                      g_test_next_index,
                                      pt->name))
        {
            USART7_DebugPrintf("[TEST] enqueue %s\r\n", pt->name);
        }
    }
#endif

#if (ALGO_INPUT_MODE == ALGO_INPUT_MODE_DEMO_XYZRYP)
    if ((!g_demo_sent) &&
        (!g_discrete_rt.waiting_finish) &&
        (AlgorithmTask_DiscreteQueueCount() == 0u))
    {
        Pose6D_t pose;
        Pose6D_SetFromXYZ_RollYawPitch(&pose,
                                       xxx, yyy, zzz,
                                       0.0f, -3.1415f, -1.57f);//将基坐标系按照EularAngle旋转（yaw-pitch-roll）,朝向以Z轴正方向为准

        if (AlgorithmTask_EnqueuePose(&pose,
                                      ALGO_SRC_DEMO_XYZRYP,
                                      0,
                                      "demo_xyzryp"))
        {
            USART7_DebugPrintf("[DEMO_XYZRYP] enqueue demo pose\r\n");
            g_demo_sent = 1;
        }
    }
#endif

#if (ALGO_INPUT_MODE == ALGO_INPUT_MODE_DEMO_POSE6D)
    if ((!g_demo_sent) &&
        (!g_discrete_rt.waiting_finish) &&
        (AlgorithmTask_DiscreteQueueCount() == 0u))
    {
        Pose6D_t pose;
        Pose6D_SetFromXYZ_RollYawPitch(&pose,
                                       0.220000f, 0.020000f, -0.200000f,
                                       -3.1415926f, 0.0f, 0.0f);

        if (AlgorithmTask_EnqueuePose(&pose,
                                      ALGO_SRC_DEMO_POSE6D,
                                      0,
                                      "demo_pose6d"))
        {
            USART7_DebugPrintf("[DEMO_POSE6D] enqueue demo pose\r\n");
            g_demo_sent = 1;
        }
    }
#endif

#if (ALGO_INPUT_MODE == ALGO_INPUT_MODE_MANUAL_API)
    /* 手动API模式不自动生成目标；
       外部通过 AlgorithmTask_PostPoseTarget / XYZRYP 接口入队 */
//    AlgorithmTask_PostPoseTargetXYZRYP_Rad(xxx, yyy, zzz,
//                                           0.0f, 3.141592503f, -1.570796371f);
#endif

#if (ALGO_INPUT_MODE == ALGO_INPUT_MODE_MANUAL_TRIGGER)
    /* 手动触发一次性发送 */
    if (g_man_trigger && !g_discrete_rt.waiting_finish && (AlgorithmTask_DiscreteQueueCount() == 0u))
    {
        Pose6D_t pose;
        Pose6D_SetFromXYZ_RollYawPitch(&pose, g_man_x, g_man_y, g_man_z,
                                       g_man_roll, g_man_yaw, g_man_pitch);
        if (AlgorithmTask_EnqueuePose(&pose, ALGO_SRC_EXTERNAL_XYZRYP, 0, "manual_trigger"))
        {
            USART7_DebugPrintf("[MANUAL] trigger sent (%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\r\n",
                               g_man_x, g_man_y, g_man_z,
                               g_man_roll, g_man_yaw, g_man_pitch);
            g_man_trigger = 0;
        }
    }
#endif


}

/* ---------------------------- 活动命令完成后的收尾 ---------------------------- */

static void AlgorithmTask_OnActiveCommandFinished(void)
{

    /*---------------------只针对ALGO_SRC_TEST_CASE这一种控制方式-----------------------------*/
    if (g_discrete_rt.active_cmd.source == ALGO_SRC_TEST_CASE)
    {
        g_test_next_index++;
        if (g_test_next_index >= TASK_TEST_COUNT)
        {
            g_test_next_index = 0;
        }
    }
    /*---------------------只针对ALGO_SRC_TEST_CASE这一种控制方式-----------------------------*/

    memset(&g_discrete_rt.active_cmd, 0, sizeof(g_discrete_rt.active_cmd));
    g_discrete_rt.waiting_finish = 0;
    g_discrete_rt.start_tick_ms = 0;
}

/* ---------------------------- 离散执行器 ---------------------------- */
/* 统一处理：发点 -> 等IK/MoveJ/Planner/执行器到位 -> 再发下一个 */

static void AlgorithmTask_DiscreteExecutorStep(const AlgoFeedback_t *fb, const AlgoOutput_t *out)
{
    uint32_t now_ms;
    AlgorithmTask_DiscreteCmd_t cmd;

    if ((fb == NULL) || (out == NULL))
    {
        return;
    }

    if (!fb->valid)
    {
        return;
    }

    now_ms = algorithm_subscribe_arm_feedback_data.tick_ms;

    /* 还没有正在执行的点,则尝试从队列中取下一个点 */
    if (!g_discrete_rt.waiting_finish)
    {
        if (((out->planner_state == JP_IDLE) || (out->planner_state == JP_DONE)) &&
            (AlgorithmTask_DiscreteQueueCount() > 0u))
        {
            taskENTER_CRITICAL();
            if (AlgorithmTask_DiscreteQueuePop(&cmd))
            {
                taskEXIT_CRITICAL();

                if (Algo_PostPoseTarget(&cmd.pose))
                {
                    g_discrete_rt.active_cmd = cmd;
                    g_discrete_rt.waiting_finish = 1;
                    g_discrete_rt.start_tick_ms = now_ms;

                    if (cmd.name != NULL)
                    {
                        // USART7_DebugPrintf("[Discrete] send %s\r\n", cmd.name);
                    }
                    else
                    {
                        USART7_DebugPrintf("[Discrete] send cmd src=%u idx=%lu\r\n",
                                           (unsigned int)cmd.source,
                                           (unsigned long)cmd.source_index);
                    }
                }
                else
                {
                    USART7_DebugPrintf("[Discrete] Algo_PostPoseTarget failed\r\n");
                }
            }
            else
            {
                taskEXIT_CRITICAL();
            }
        }

        return;
    }

    /* 已经有一个点在执行：检查各种结束条件 */

    if (out->cmd_state == ALGO_CMD_IK_FAILED)
    {
        USART7_DebugPrintf("[Discrete] IK failed\r\n");
        AlgorithmTask_OnActiveCommandFinished();
    }
    else if (out->cmd_state == ALGO_CMD_MOVEJ_START_FAILED)
    {
        USART7_DebugPrintf("[Discrete] MoveJ start failed\r\n");
        AlgorithmTask_OnActiveCommandFinished();
    }
    else if (out->planner_state == JP_FAULT)
    {
        USART7_DebugPrintf("[Discrete] planner fault\r\n");
        AlgorithmTask_OnActiveCommandFinished();
    }
    else if ((g_discrete_rt.start_tick_ms != 0u) &&
             ((now_ms - g_discrete_rt.start_tick_ms) > ALGO_DISCRETE_TIMEOUT_MS))
    {
        USART7_DebugPrintf("[Discrete] timeout > %lu ms\r\n",
                           (unsigned long)ALGO_DISCRETE_TIMEOUT_MS);
        AlgorithmTask_OnActiveCommandFinished();
    }
    else if (AlgorithmTask_IsSettled3FromOutput(out))
    {
        if (g_discrete_rt.active_cmd.name != NULL)
        {
            // USART7_DebugPrintf("[Discrete] done %s\r\n", g_discrete_rt.active_cmd.name);
        }
        else
        {
            USART7_DebugPrintf("[Discrete] done cmd src=%u idx=%lu\r\n",
                               (unsigned int)g_discrete_rt.active_cmd.source,
                               (unsigned long)g_discrete_rt.active_cmd.source_index);
        }

        AlgorithmTask_OnActiveCommandFinished();
    }
}

/* -------------------------------- 线程入口 ------------------------------- */
void AlgorithmTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    Init_KalmanFiltersOne(KALMAN_F, KALMAN_H, KALMAN_Q, KALMAN_R);
    /* MoveJ 初始化：只做一次 */
    /* Algo 初始化：只做一次 */
    Algo_InitContext();
    memset(&g_algo_out, 0, sizeof(g_algo_out));
    memset(&g_algo_fb, 0, sizeof(g_algo_fb));
    memset(&g_discrete_rt, 0, sizeof(g_discrete_rt));

    AlgorithmTask_DiscreteQueueInit();
    g_demo_sent = 0;
    g_test_next_index = 0;
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
    algorithm_topic_sub_init();
    algorithm_topic_pub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */

/* -------------------------------- 调试监测线程调度 --------------------------------- */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */

    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();
        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */

/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
        algorithm_topic_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */

        if (xQueueReceive(xKalmanOneQueue, angles, 0) == pdTRUE)//处理自定义控制器发来的控制命令，与自动解算无关
        {
            // 对接收的数据进行滤波处理
            KalmanFilterOne_Data(angles, filtered_data);
            xQueueSend(xControlQueue, filtered_data, 0);
        }

        /* 1) 从Topic构造反馈 */
        AlgorithmTask_BuildFeedback(&g_algo_fb);

        /* 2) 喂给算法层,根据FK得出工具在基座标系上的坐标和朝向*/
        Algo_SetFeedback(&g_algo_fb);

        /* 3) 根据当前输入模式填充离散目标队列 */
        AlgorithmTask_SourceFillStep(&g_algo_fb, &g_algo_out);

        /* 4) 离散执行器：统一发点/等完成/再发下一个 */
        AlgorithmTask_DiscreteExecutorStep(&g_algo_fb, &g_algo_out);

        /* 5) 跑一步算法 */
        Algo_Step(algorithm_task_dt);

        /* 6) 获取当前输出 */
        Algo_GetOutput(&g_algo_out);


/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        algorithm_topic_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */

        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
static void algorithm_topic_pub_init(void)
{
    publish_movej_ref_topic = pub_register("movej_ref_pub", sizeof(movej_ref_msg_t));
}

static void algorithm_topic_sub_init(void)
{
    // 订阅关节反馈数据
    subscribe_arm_feedback_topic = sub_register("dm_arm_feedback_pub", sizeof(dm_arm_feedback_msg_t));
}

static void algorithm_topic_pub_push(void)
{
    int i;

    memset(&algorithm_publish_movej_ref_data, 0, sizeof(algorithm_publish_movej_ref_data));

    algorithm_publish_movej_ref_data.seq = ++movej_pub_seq;
    algorithm_publish_movej_ref_data.tick_ms = algorithm_subscribe_arm_feedback_data.tick_ms;
    algorithm_publish_movej_ref_data.planner_state = g_algo_out.planner_state;
    algorithm_publish_movej_ref_data.valid = g_algo_out.valid;

    if (g_algo_out.valid)
    {
        for (i = 0; i < JOINT_NUM; i++)
        {
            algorithm_publish_movej_ref_data.q_ref_rad[i]   = g_algo_out.q_ref[i];
            algorithm_publish_movej_ref_data.v_ref_rad_s[i] = g_algo_out.v_ref[i];

            algorithm_publish_movej_ref_data.q_fb_rad[i]    = g_algo_out.q_fb[i];
            algorithm_publish_movej_ref_data.v_fb_rad_s[i]  = g_algo_out.v_fb[i];
        }
    }

    pub_push_msg(publish_movej_ref_topic, &algorithm_publish_movej_ref_data);
}

static void algorithm_topic_sub_pull(void)
{
    sub_get_msg(subscribe_arm_feedback_topic, &algorithm_subscribe_arm_feedback_data);
}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */