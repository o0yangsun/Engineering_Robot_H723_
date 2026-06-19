#include "joint_movej_planner.h"
#include "usart_task.h"
#include <string.h>
#include <math.h>

static bool JointMoveJ_CheckLimit(const float q[JOINT_NUM], const JointLimit_t *limit)
{
    int i;

    if ((q == 0) || (limit == 0))
    {
        return false;
    }

    for (i = 0; i < JOINT_NUM; i++)
    {
        if ((q[i] < limit->min[i]) || (q[i] > limit->max[i]))
        {
            return false;
        }
    }

    return true;
}

/* 本地最小值函数
 * 用来在多关节中找出最严格的全局约束 */
static float minf_local(float a, float b)
{
    return (a < b) ? a : b;
}

/* 初始化关节空间 MoveJ 规划器
 * 作用：
 *   1. 把整个结构体清零
 *   2. 把状态设为 JP_IDLE（空闲）
 */
void JointMoveJ_Init(JointMoveJPlanner_t *jp)
{
    if (jp == 0)
    {
        return;
    }

    memset(jp, 0, sizeof(JointMoveJPlanner_t));
    jp->state = JP_IDLE;
}

/* 停止当前 MoveJ 规划
 * 作用：
 *   1. 停掉内部标量轨迹 prof
 *   2. 把状态设为 JP_ABORT（中止）
 *   3. 把每个关节的参考速度清零
 */
void JointMoveJ_Stop(JointMoveJPlanner_t *jp)
{
    int i;

    if (jp == 0)
    {
        return;
    }

    /* 关闭内部标量轨迹 */
    jp->prof.running = 0;

    /* 状态改为中止 */
    jp->state = JP_ABORT;

    /* 参考速度清零 */
    for (i = 0; i < JOINT_NUM; i++)
    {
        jp->v_ref[i] = 0.0f;
    }
}

/* 启动一次关节空间 MoveJ 规划
 *
 * 输入：
 *   jp            : 规划器对象
 *   q0[6]         : 起点关节角
 *   q1[6]         : 终点关节角
 *   vmax[6]       : 每个关节的最大速度约束
 *   amax[6]       : 每个关节的最大加速度约束
 *
 * 输出：
 *   成功返回 true，失败返回 false
 *
 * 核心思想：
 *   不是给 6 个关节分别做 6 条独立轨迹，
 *   而是用一个统一的标量进度 s(t) 驱动：
 *
 *     q_ref[i] = q0[i] + dq[i] * s
 *     v_ref[i] = dq[i] * sdot
 *
 *   为了保证所有关节都不超限，
 *   要先根据 6 个关节的位移和约束，求出统一的：
 *     vs_max
 *     as_max
 */
bool JointMoveJ_Start(JointMoveJPlanner_t *jp,
                      const float q0[JOINT_NUM],
                      const float q1[JOINT_NUM],
                      const float vmax[JOINT_NUM],
                      const float amax[JOINT_NUM],
                      const JointLimit_t *limit)
{
    int i;
    float adq;
    bool found_moving_joint = false;

    /* 参数检查 */
    if ((jp == 0) || (q0 == 0) || (q1 == 0) || (vmax == 0) || (amax == 0))
    {
        return false;
    }

    /* 清空规划器内部状态，准备重新启动一条新轨迹 */
    memset(jp, 0, sizeof(JointMoveJPlanner_t));

    /* 先做起点/终点关节限位检查 */
    if (!JointMoveJ_CheckLimit(q0, limit))
    {
        jp->state = JP_FAULT;
        USART7_DebugPrintf("Start Point Out Of Joint Limit");
        return false;
    }

    if (!JointMoveJ_CheckLimit(q1, limit))
    {
        jp->state = JP_FAULT;
        USART7_DebugPrintf("End Point Out Of Joint Limit");
        return false;
    }

    /* 先把全局约束初始化成极大值
     * 后面会逐关节取最小值 */
    jp->vs_max = 1e30f;
    jp->as_max = 1e30f;
    jp->moving_joint_count = 0;

    /* 遍历 6 个关节 */
    for (i = 0; i < JOINT_NUM; i++)
    {
        /* 保存起点、终点 */
        jp->q0[i] = q0[i];
        jp->q1[i] = q1[i];

        /* 计算本关节位移 dq = q1 - q0 */
        jp->dq[i] = q1[i] - q0[i];

        /* 保存本关节速度/加速度约束 */
        jp->vmax[i] = vmax[i];
        jp->amax[i] = amax[i];

        /* 启动时，参考位置先放在起点，参考速度先置 0 */
        jp->q_ref[i] = q0[i];
        jp->v_ref[i] = 0.0f;

        /* 取位移绝对值 */
        adq = fabsf(jp->dq[i]);

        /* 静止关节不参与全局约束
         * 即如果本关节几乎不用动，就跳过 */
        if (adq < JOINT_EPS_MOVE)
        {
            continue;
        }

        /* 对于真正有运动的关节，必须保证 vmax / amax 合法 */
        if ((vmax[i] <= 0.0f) || (amax[i] <= 0.0f))
        {
            jp->state = JP_FAULT;
            return false;
        }

        /* 计算统一标量进度 s 的最大速度约束
         *
         * 因为：
         *   v_ref[i] = dq[i] * sdot
         *
         * 所以要满足：
         *   |sdot| <= vmax[i] / |dq[i]|
         *
         * 6 个关节共同同步运动时，必须取所有关节里最严格的那个 */
        jp->vs_max = minf_local(jp->vs_max, vmax[i] / adq);

        /* 同理，统一标量进度 s 的最大加速度约束
         *
         * 因为：
         *   a_ref[i] = dq[i] * sddot
         *
         * 所以：
         *   |sddot| <= amax[i] / |dq[i]| */
        jp->as_max = minf_local(jp->as_max, amax[i] / adq);

        /* 统计真正有运动的关节数量 */
        jp->moving_joint_count++;
        found_moving_joint = true;
    }

    /* 如果 6 个关节都不用动，则直接完成 */
    if (!found_moving_joint)
    {
        for (i = 0; i < JOINT_NUM; i++)
        {
            jp->q_ref[i] = jp->q1[i];
            jp->v_ref[i] = 0.0f;
        }

        jp->state = JP_DONE;
        return true;
    }

    /* 再做一次全局约束有效性检查 */
    if ((jp->vs_max <= 0.0f) || (jp->as_max <= 0.0f))
    {
        jp->state = JP_FAULT;
        return false;
    }

    /* 启动内部标量梯形/三角轨迹
     * 注意：这里规划的不是某个具体关节，而是统一进度 s(t) */
    if (!TrajTimeSync_Start(&jp->prof, jp->vs_max, jp->as_max))//得出让进度条0-1,一起开始一起结束的加速时间和匀速时间
    {
        jp->state = JP_FAULT;
        return false;
    }

    /* 进入运行状态 */
    jp->state = JP_RUNNING;
    return true;
}

/* 周期更新 MoveJ 规划器
 *
 * 输入：
 *   jp  : 规划器对象
 *   dt  : 控制周期
 *
 * 输出：
 *   成功返回 true，失败返回 false
 *
 * 功能：
 *   1. 从内部标量轨迹中取出当前 s 和 sdot
 *   2. 映射成 6 个关节的 q_ref 和 v_ref
 *   3. 如果轨迹结束，则把参考值钉在终点并置 DONE
 */
bool JointMoveJ_Update(JointMoveJPlanner_t *jp, float dt)
{
    int i;
    float s, sdot;

    /* 参数检查 */
    if (jp == 0)
    {
        return false;
    }

    /* 如果已经完成，则持续输出终点参考值 */
    if (jp->state == JP_DONE)
    {
        for (i = 0; i < JOINT_NUM; i++)
        {
            jp->q_ref[i] = jp->q1[i];
            jp->v_ref[i] = 0.0f;
        }
        return true;
    }

    /* 只有 RUNNING 状态才允许更新 */
    if (jp->state != JP_RUNNING)
    {
        return false;
    }

    /* 更新内部标量轨迹，得到当前的 s 和 sdot */
    if (!TrajTimeSync_Update(&jp->prof, dt, &s, &sdot))
    {
        jp->state = JP_FAULT;
        return false;
    }

    /* 用统一的 s / sdot 计算每个关节当前周期的参考值 */
    for (i = 0; i < JOINT_NUM; i++)
    {
        /* q_ref = q0 + dq * s
         * 表示当前总进度 s 下，本关节应该走到哪里 */
        jp->q_ref[i] = jp->q0[i] + jp->dq[i] * s;

        /* v_ref = dq * sdot
         * 表示当前总进度速度 sdot 下，本关节应该用多快 */
        jp->v_ref[i] = jp->dq[i] * sdot;   //sdot = 1/t = Vref/dθ    s为进度速度
    }

    /* 如果内部标量轨迹已经结束，则把末值强制钉在终点 */
    if (jp->prof.running == 0)
    {
        for (i = 0; i < JOINT_NUM; i++)
        {
            jp->q_ref[i] = jp->q1[i];
            jp->v_ref[i] = 0.0f;
        }
        jp->state = JP_DONE;
    }

    return true;
}