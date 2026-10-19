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
#include <math.h>
#include "DMmotor_task.h"
#include "drv_dwt.h"
#include "PID.h"
#include "cmd_task.h"
#include "msg_freertos.h"
#include "transmission_task.h"
#include "robot_task.h"
#include "gripper_force.h"

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

static struct pc_cmd_arm_msg dm_receive_pc_cmd_arm_msg_data = {0};
static subscriber_t *subscribe_cmd_pc_arm_topic;
static int8_t pc_ctrl_process_last_state;
static dm_arm_feedback_msg_t dm_arm_feedback_pub_msg = {0};
static publisher_t *publish_dm_arm_feedback_topic = NULL;
static subscriber_t *publish_dm_arm_ctrl_mode_topic = NULL;
static subscriber_t *publish_gripper_ctrl_mode_topic = NULL;
static subscriber_t *publish_mcu_gripper_ctrl_mode_topic = NULL;

extern sbus_data_t sbus_data_fdb;

static void DMmotor_topic_pub_init(void);
static void DMmotor_topic_sub_init(void);
static void DMmotor_topic_pub_push(void);
static void DMmotor_topic_sub_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t DMmotor_task_dwt = 0;   // 毫秒监测
static float DMmotor_task_dt = 0;       // 线程实际运行时间dt
static float DMmotor_task_delta = 0;    // 监测线程运行时间
static float DMmotor_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */


static pid_obj_t *execute_track_movej_planner_pid;
static pid_config_t execute_track_movej_config = INIT_PID_CONFIG(0.45, 0.0, 0.012, 0.0, 4.3, PID_Trapezoid_Intergral);

static float current_angle[6] = {0.0f};        // 实际的关节输出角度，也是需要滤波的值
static float dm_angles[6] = {0.0f};   // 队列读取值
static float dm_pc_motor_angles[6] = {0.0f};   // 期望角度值
static float dm_user_motor_angles[6] = {0.0f};   // 期望角度值

/* ★ 说明：`gripper_state` 会被 `DMmotor_topic_sub_pull()` 用话题里的
 * `gripper_ctrl_mode` 覆盖（即"命令回显"），所以它的初值其实无关紧要；
 * `mcu_gripper_state` 则被 `MCU_gripper_ctrl` 每周期覆盖（algorithm_task.c:739）。
 * 两个都给 Gripper_FREE 做初值：万一话题还没来，也不会凭空产生一条"张开"命令。 */
Gripper_mode_e gripper_state = Gripper_FREE;
Gripper_mode_e mcu_gripper_state = Gripper_FREE;
extern Gripper_mode_e gripper_ctrl_mode;

//User_defined_Controller, //自定义模式控制器
//PC_based_Controller,    //上位机控制
static Arm_mode_e arm_control_state;
static Arm_mode_e arm_control_last_state;
Auto_ctrl_mode auto_ctrl_mode = AUTO_WAIT ;
//AUTO_WAIT,         // 0: 等待
//AUTO_RIGHT_PLACE,   // 1: 左边放置
//AUTO_RIGHT_GRAB,    // 2: 左边抓取
//AUTO_LEFT_PLACE,  // 3: 右边放置
//AUTO_LEFT_GRAB    // 4: 右边抓取
extern QueueHandle_t xControlQueue;


DMmotorControl motor_controls[6] = {
        { MOTOR_1_MIN_LIMIT, MOTOR_1_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 0 (FDCAN3)
        { MOTOR_2_MIN_LIMIT, MOTOR_2_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 1 (FDCAN2)
        { MOTOR_3_MIN_LIMIT, MOTOR_3_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 2 (FDCAN2)
        { MOTOR_4_MIN_LIMIT, MOTOR_4_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 3 (FDCAN2)
        { MOTOR_5_MIN_LIMIT, MOTOR_5_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 4 (FDCAN2)
        { MOTOR_6_MIN_LIMIT, MOTOR_6_MAX_LIMIT, 0.0f, 0.0f, 0 }  // Motor 5 (FDCAN2)
};

struct arm_cmd_msg arm_cmd = {
        .ctrl_mode = ARM_DISABLE,
        .last_mode = ARM_DISABLE
};

/* ------------------------- 夹爪(第 7 路)归一化力矩指令 -------------------------
 * 语义（2026-10-09 定，2026-10-10 改为 PSI 实现）：
 *   0x21 里夹爪只占【1 个字节 int8】= 【方向 + 力矩大小】，**协议长度不变（LEN 仍 27）**。
 *       cmd == 0          → 零状态：不控开闭、也不出力
 *       cmd ∈ (+1..+100]  → 张开，力矩上限 = |cmd|/100 × GF_TAU_OPEN_NM
 *       cmd ∈ [-100..-1)  → 闭合，力矩上限 = |cmd|/100 × GF_TAU_CLOSE_NM
 *
 * ★ 2026-10-11：先在 10-10 搬到 PSI(PVT)，台架实测后**改回 MIT + 软件外环**。
 *   台架结论（详见 `gripper_force.h` 文件头）：
 *     ① 这台 DM4310 的 PVT/PSI 环路**发散** —— 目标 = 当前位置（理论零误差）
 *        也会持续加速到 +3.85 rad/s；同一台电机 MIT 目标 −12.105 稳定收敛到
 *        −12.073、τ 仅 −0.10 N·m；
 *     ② 模式由电机 EEPROM 的 **RID10** 决定，**不是**由使能帧的 CAN ID 偏移决定
 *        （RID10=1 时发 0x307 也照样纹丝不动）；
 *     ③ MIT 是出厂默认（RID10=1）⇒ 不需要用达妙助手改 EEPROM。
 *   MIT 缺的"定速 / 力上限"两个入口，改由软件外环补上（都在 `gripper_force.c`）：
 *     · 定速开合 → 软件斜坡，把 pos_set 按 `speed × dt` 逐步逼近目标；
 *     · 夹持力上限 → 读 `para.tor`，|τ| 超过上限就**冻结 pos_set 的推进**；
 *     · 零状态 → kp=0、tor=0，只留一点 kd 阻尼（可自由掰动）。
 *   旧的两个量仍已下线：
 *     · `DM7_TAU_MAX`  → 拆成 GF_TAU_OPEN_NM(1.0) / GF_TAU_CLOSE_NM(1.5)
 *     · `DM7_TAU_RATE` 斜率限制 → 仍不需要（力矩上限由位置环 + 外环天然实现）
 * --------------------------------------------------------------------------- */

/* Gripper_mode_e(0/1 开关) → 归一化力矩指令。
 * 拨杆 / 键盘 / MCU 档沿用旧符号约定：OPEN → +100、CLOSE → −100。
 * ★ 2026-10-10 新增第三态：Gripper_FREE（= "没有命令"）→ **0**（车端 PSI 零状态）。
 *   它与 Gripper_OPEN(=0) 的区别见 cmd_task.h 的枚举注释 —— 核心是"零初始化的默认值
 *   不该被当成一条真实命令"，否则上电就会自己跑。 */
static inline int8_t gripper_mode_to_cmd(Gripper_mode_e m)
{
    if (m == Gripper_CLOSE) return (int8_t)-100;
    if (m == Gripper_OPEN)  return (int8_t) 100;
    return (int8_t)0;                       /* Gripper_FREE */
}

void arm_mode_change_init_process(const float motor_angle[6])
{

    dm_motor_enable(&hfdcan3, &motor[Motor1]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan3, motor[Motor1].id, -motor_angle[0]/57.3f, 0.5f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定

    dm_motor_enable(&hfdcan3, &motor[Motor2]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan3, motor[Motor2].id, motor_angle[1]/57.3f, 0.5f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定


    dm_motor_enable(&hfdcan2, &motor[Motor3]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan2, motor[Motor3].id, motor_angle[2]/57.3f, 0.5f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定

    dm_motor_enable(&hfdcan2, &motor[Motor4]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan2, motor[Motor4].id, -motor_angle[3]/57.3f, 0.5f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定

    dm_motor_enable(&hfdcan2, &motor[Motor5]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan2, motor[Motor5].id, motor_angle[4]/57.3f, 0.5f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定

    dm_motor_enable(&hfdcan2, &motor[Motor6]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan2, motor[Motor6].id, -motor_angle[5]/57.3f, 0.5f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定

    /* ★★ 上电时序（2026-10-11 修正）—— 先换回真实位置，再使能。
     *   `para.pos` 在收到第一帧 CAN 反馈之前恒为 0。若直接 enable 再让位置环
     *   拿着 pos_set=0 跑，夹爪会朝"绝对 0 rad"冲过去（那个位置可能在行程之外）。
     *   这里在【电机仍失能】时先发 3 帧**零刚度零力矩**的 MIT 帧（probe_send）：
     *   帧内 kp/kd/tor/vel 全 0 ⇒ 物理上不可能产生运动，只为让电机进入 MIT 模式
     *   并开始周期上报反馈，好把 `para.pos` 填成真值。安全。 */
    for (int i = 0; i < 3; i++) {
        gripper_force_probe_send(&hfdcan2);
        vTaskDelay(5);
    }
    gripper_force_init(&hfdcan2);   /* 切 MIT + 清零控制量（必须在 enable 之前） */
    dm_motor_enable(&hfdcan2, &motor[Motor7]);//不用校准//开始发送夹爪初始化控制指令
    vTaskDelay(200); // 延时，等待电机稳定

    /* ★ 2026-10-11 新增：切档后把夹爪**回零到张开位**（用户选定：三处触发全上）。
     *   理由：切档 = 重新接管。过去这里只 init（目标对齐到当前位置）⇒ 夹爪停在
     *   原地、位姿不确定；而下游（遥操作映射 / PC 采集 / VLA 推理）都假定"从张开位开始"。
     *   回零由 gripper_force 内部状态机完成（复用斜坡限速 + 力外环限力），
     *   是**非阻塞**的（本函数不等它走完），到位后自动把控制权交还拨杆/PC。 */
    gripper_force_start_home(&hfdcan2);

    arm_cmd.ctrl_mode = ARM_ENABLE; // 使能机械臂
    arm_cmd.last_mode = ARM_ENABLE;
    vTaskDelay(1000); // 延时，等待电机稳定
}



void arm_cmd_enable(void) {
    if (arm_cmd.last_mode == ARM_DISABLE && arm_cmd.ctrl_mode == ARM_ENABLE) {
        dm_motor_enable(&hfdcan3, &motor[Motor1]);
        vTaskDelay(1);
        dm_motor_enable(&hfdcan3, &motor[Motor2]);
        vTaskDelay(1);
        for(int i=2;i<6;i++)
        {
            dm_motor_enable(&hfdcan2, &motor[i]);
            vTaskDelay(1);
        }
        /* ★ 2026-10-10 P0-1 修复：夹爪(第7路)必须一并纳入使能/失能，
         *   否则按一次"失能"后，夹爪会永久停在失能态（再按使能也不会恢复）。
         *   单独处理的原因：它是 MIT 模式(位置 + 软件外环)，不能混进上面那个
         *   只为 4 个 pos_mode 关节设计的循环。
         *   这里调 gripper_force_init() 而不是裸 enable，是为了【先把 kp/tor 清零】——
         *   否则"失能→使能"会把上一次的夹持力原样重新加上去，操作员的手若在夹爪里
         *   就会被突然夹住。清零后需要 PC 重新下发命令才出力，更安全。 */
        /* ★ 同 arm_cmd_init()：此刻电机刚被失能 ⇒ 先发几帧零力 MIT 帧换回真实位置，
         *   再 init（清零控制量）+ enable。apply() 的首帧还会再把斜坡目标对齐一次，
         *   双保险，杜绝"使能瞬间朝绝对 0 冲"。 */
        for (int i = 0; i < 3; i++) {
            gripper_force_probe_send(&hfdcan2);
            vTaskDelay(5);
        }
        gripper_force_init(&hfdcan2);
        dm_motor_enable(&hfdcan2, &motor[Motor7]);
        vTaskDelay(1);
        /* ★ 2026-10-11 新增：遥控器"使能"时夹爪回零（张开位）。
         *   与上电/切档用**同一个** gripper_force_start_home()，避免"一条路径有、另一条没有"
         *   的老毛病（评审 P1-2 就是这类问题）。非阻塞，1 kHz 的主循环会持续驱动它。 */
        gripper_force_start_home(&hfdcan2);
        arm_cmd.last_mode = ARM_ENABLE;
    }
}
void arm_cmd_disable(void) {
    if (arm_cmd.last_mode == ARM_ENABLE && arm_cmd.ctrl_mode == ARM_DISABLE) {
        dm_motor_disable(&hfdcan3, &motor[Motor1]);
        vTaskDelay(1);
        dm_motor_disable(&hfdcan3, &motor[Motor2]);
        vTaskDelay(1);
        for(int i=2;i<6;i++)
        {
            dm_motor_disable(&hfdcan2, &motor[i]);
            vTaskDelay(1);
        }
        /* ★ 2026-10-10 P0-1 修复：夹爪(第7路)必须先卸力再失能。
         *   原实现只循环到 Motor6 ⇒ 按"机械臂失能"后，夹爪电机仍带电、并按最后的
         *   cmd 持续顶住物体。gripper_force_disable() 会先**真正下发**一帧
         *   "零刚度零力矩 + 目标=当前位置"的 MIT 指令，再发失能帧，避免带大电流被切断。 */
        gripper_force_disable(&hfdcan2);
        vTaskDelay(1);
        arm_cmd.last_mode = ARM_DISABLE;
    }
}

void arm_cmd_init(void) {
    if (arm_cmd.last_mode == ARM_ENABLE && arm_cmd.ctrl_mode == ARM_INIT) {
        pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 1.2f); // 发送控制命令
        vTaskDelay(200); // 延时，等待电机稳定
        pos_ctrl(&hfdcan3, motor[Motor2].id, 0, 1.2f); // 发送控制命令
        vTaskDelay(200); // 延时，等待电机稳定

        for(int i=2;i<6;i++)
        {
            dm_motor_enable(&hfdcan2, &motor[i]);
            pos_ctrl(&hfdcan2, motor[i].id, 0, 1.2f); // 发送控制命令
            vTaskDelay(200); // 延时，等待电机稳定
        }
        /* 注意：这里【故意不含 Motor7】。回零位只针对 6 个关节，夹爪不该被 pos_ctrl
         * 拉到 0 rad（那会是一次不可预期的开/合动作）；且 Motor7 现在是 MIT 模式，
         * pos_ctrl 发的 POS_MODE 帧(0x1xx)在电机侧会被直接忽略。 */
        arm_cmd.last_mode = ARM_ENABLE;  //TODO:BUG一个
    }
}

void arm_cmd_state_machine(void) {

    switch (arm_cmd.ctrl_mode) {
        case ARM_ENABLE:
            arm_cmd_enable();
            break;
        case ARM_DISABLE:
            arm_cmd_disable();
            break;
//        case ARM_INIT:
//            arm_cmd_init();
//            break;
        default:
//            arm_cmd_disable();
            break;
    }
}


static subscriber_t *subscribe_movej_ref_topic;
static movej_ref_msg_t dmmotor_subscribe_movej_ref_data;
static uint32_t dmmotor_last_movej_seq = 0;

float Kp_track = 4.0f;      // 先从小值开始调
float Kv_track = 0.95f;
float pos_cmd[6];
float vel_cmd[6];
static void DMmotor_apply_movej_ref(const movej_ref_msg_t *ref)
{
    float pos_fdb[6];
    float vel_fdb[6];

    float pos_err[6];
    float vel_err[6];



    const float v_min_follow = 0.5f; // 有误差时最小追赶速度
    const float v_max_exec   = 6.0f;  // 执行层最大速度
    const float pos_tol      = 0.01f; // 约 0.57 度
    /* 无效轨迹，不发送 */
    if (ref == 0 || ref->valid == 0)
    {
        return;
    }

    // 此为上一个ms周期的误差，等会需要优先把反馈误差更新
    for (uint8_t i = 0; i < 6; i++)
    {
        pos_fdb[i] = dm_arm_feedback_pub_msg.joint[i].pos_rad;
        vel_fdb[i] = dm_arm_feedback_pub_msg.joint[i].vel_rad_s;
        pos_err[i] = ref->q_ref_rad[i] - pos_fdb[i];
        vel_err[i] = ref->v_ref_rad_s[i] - vel_fdb[i];
        vel_cmd[i] = fabsf(ref->v_ref_rad_s[i]) + Kp_track * fabsf(pos_err[i]) + Kv_track * fabsf(vel_err[i]);
        if (fabsf(pos_err[i]) > pos_tol && vel_cmd[i] < v_min_follow) vel_cmd[i] = v_min_follow;
        if (vel_cmd[i] > v_max_exec) vel_cmd[i] = v_max_exec;
        pos_cmd[i] = ref->q_ref_rad[i];
    }

    /* 按你原来的方向定义修正，1轴通常要核对是否需要负号 */
    pos_ctrl(&hfdcan3, motor[Motor1].id, pos_cmd[0], vel_cmd[0]);
    pos_ctrl(&hfdcan3, motor[Motor2].id,  pos_cmd[1], vel_cmd[1]);
    pos_ctrl(&hfdcan2, motor[Motor3].id,  pos_cmd[2], vel_cmd[2]);
    pos_ctrl(&hfdcan2, motor[Motor4].id, pos_cmd[3], vel_cmd[3]);
    pos_ctrl(&hfdcan2, motor[Motor5].id, pos_cmd[4], vel_cmd[4]);
    pos_ctrl(&hfdcan2, motor[Motor6].id, pos_cmd[5], vel_cmd[5]);
}

/* 每个关节一个角度中值滤波器 */
static median_filter5_t read_joint_pos_filter[6] = {0};
// 死区处理函数
static inline float joint_deadband_apply(float x, float threshold)
{
    return (fabsf(x) <= threshold) ? 0.0f : x;
}

/* 对 5 个数做中值滤波：排序后取中间值 */
static float median5_calc(const float in[MEDIAN_WIN_SIZE])
{
    float tmp[MEDIAN_WIN_SIZE];
    uint8_t i, j;
    float key;
    /* 拷贝一份，避免改原数据 */
    for (i = 0; i < MEDIAN_WIN_SIZE; i++) {
        tmp[i] = in[i];
    }
    /* 插入排序 */
    for (i = 1; i < MEDIAN_WIN_SIZE; i++) {
        key = tmp[i];
        j = i;
        while ((j > 0u) && (tmp[j - 1u] > key)) {
            tmp[j] = tmp[j - 1u];
            j--;
        }
        tmp[j] = key;
    }
    /* 5个数的中值下标就是 2 */
    return tmp[MEDIAN_WIN_SIZE / 2u];
}

static float median_filter5_update(median_filter5_t *filter, float input)
{
    uint8_t i;
    /* 第一次进入时，用首个值填满窗口，避免启动阶段窗口未满 */
    if (filter->inited == 0u) {
        for (i = 0; i < MEDIAN_WIN_SIZE; i++) {
            filter->buf[i] = input;
        }
        filter->index = 0u;
        filter->inited = 1u;
        return input;
    }
    /* 环形覆盖 */
    filter->buf[filter->index] = input;
    filter->index++;
    if (filter->index >= MEDIAN_WIN_SIZE) {
        filter->index = 0u;
    }
    return median5_calc(filter->buf);
}

static inline void copy_one_joint_feedback_filtered(dm_joint_feedback_t *dst,
                                                    const motor_t *src,
                                                    uint8_t joint_idx)
{
    float pos_raw;
    float vel_raw;
    float pos_filtered;

    pos_raw = src->para.pos;
    vel_raw = src->para.vel;
    /* 1. 角度先做 5窗口中值滤波 */
    pos_filtered = median_filter5_update(&read_joint_pos_filter[joint_idx], pos_raw);
    /* 2. 再做死区归零 */
    pos_filtered = joint_deadband_apply(pos_filtered, POS_DEADBAND_RAD);
    /* 3. 速度直接做死区归零 */
    vel_raw = joint_deadband_apply(vel_raw, VEL_DEADBAND_RAD_S);

    dst->id        = src->para.id;
    dst->state     = src->para.state;
    dst->pos_rad   = pos_filtered;
    dst->vel_rad_s = vel_raw;
    dst->tor_nm    = src->para.tor;
    dst->mos_temp  = src->para.Tmos;
    dst->coil_temp = src->para.Tcoil;
}

static const uint8_t joint_motor_name[6] = {
        Motor1, Motor2, Motor3, Motor4, Motor5, Motor6
};

static void dm_feedback_cache_update(void)
{
    taskENTER_CRITICAL();

    dm_arm_feedback_pub_msg.tick_ms = (uint32_t)dwt_get_time_ms();
    dm_arm_feedback_pub_msg.update_mask = 0;

    for (uint8_t i = 0; i < 6; i++) {
        copy_one_joint_feedback_filtered(&dm_arm_feedback_pub_msg.joint[i],
                                         &motor[joint_motor_name[i]], i);
        dm_arm_feedback_pub_msg.update_mask |= (1u << i);  // 表示每个周期六个电机都更新
    }
    /* ★ 2026-10-10：上报前把 Gripper_FREE(=2) 归一化成 OPEN(0)。
     *   0x20 里这一字节的对外契约仍然是【0 = 张开 / 1 = 闭合】，PC 侧
     *   `decode_follower_uplink` 也是按这个解的。FREE 只是车端内部"还没有命令"
     *   的表示，不该作为一种新取值泄漏到总线上。 */
    dm_arm_feedback_pub_msg.gripper_state =
        (gripper_state == Gripper_CLOSE) ? Gripper_CLOSE : Gripper_OPEN;
    /* 2026-10-09 新增：夹爪实测位置 / 实测力矩（评审 P2-5，供 0x20 上行回传）。
     *   刻意用【原始值】，不走 1~6 关节那套中值滤波 + 死区：
     *     ① 夹爪是力控器件，滤波会把"刚接触物体"那一瞬的力矩台阶抹平；
     *     ② motor[Motor7] 的 TMAX = 10.0f 与达妙助手读到的 DM4310 真值一致
     *        ⇒ uint_to_float 换算准确，力矩读数可信（对比 J1~J3 的 50）。
     *   注意：DM 电机在 MIT 模式下会【保持最后一帧命令】，所以夹爪失联时
     *   这两个值会冻结而不是归零；下游判活要用 arm_control_state / 帧龄。 */
    dm_arm_feedback_pub_msg.gripper_pos_rad = motor[Motor7].para.pos;
    dm_arm_feedback_pub_msg.gripper_tor_nm  = motor[Motor7].para.tor;
    arm_control_last_state = dm_arm_feedback_pub_msg.arm_control_state;
    dm_arm_feedback_pub_msg.arm_control_state = arm_control_state;
    dm_arm_feedback_pub_msg.auto_ctrl_mode = auto_ctrl_mode;


    taskEXIT_CRITICAL();
}

float joint_pos[6] = {0};
float joint_vel[6] = {0};

/* -------------------------------- 线程入口 ------------------------------- */
void DMmotorTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    /** 此任务线程为机械臂运动学逆解算的执行层 **/
    /** 含义是：algorithm线程完成FK IK解算并且实时完成时间同步规划器运算，以及关节轨迹规划器运算，发送目标运动角度和运动速度到此dmmotor线程开始目标执行 **/
    /** 为了避免执行器和规划器的调度误差，需要在执行器层给速度和角度添加误差跟踪PID **/
     execute_track_movej_planner_pid = pid_register(&execute_track_movej_config);
    /* ------------------------ 规划器与执行器分割线 --------------------------------- */

/* -------------------------------- 外设初始化段落 ------------------------------- */
    for (int i = 0; i < 6; i++) {
        motor_controls[i].current_angle_rad = 0.0f;
        motor_controls[i].last_angle_rad = 0.0f;
        motor_controls[i].calibrated = 0;
    }

    dm_motor_enable(&hfdcan3, &motor[Motor1]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 1.0f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定

    dm_motor_enable(&hfdcan3, &motor[Motor2]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan3, motor[Motor2].id, 0, 1.0f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定

    for(int i=2;i<6;i++)
    {
        dm_motor_enable(&hfdcan2, &motor[i]);
        vTaskDelay(200); // 延时，等待电机稳定
        pos_ctrl(&hfdcan2, motor[i].id, 0, 1.0f); // 发送控制命令
        vTaskDelay(200); // 延时，等待电机稳定
    }
    vTaskDelay(200); // 延时，等待电机稳定
    /* ★ 2026-10-11：与 arm_cmd_enable() 用同一套时序 —— 先发几帧零力 MIT 帧换回
     *   真实位置，再 init + enable。原来这里是**裸** dm_motor_enable()，
     *   而 gripper_force_init() 从未被调用过（评审 P1-2）。虽然 apply() 的首帧
     *   对齐已经能兜住（不会朝 0 冲），但显式 probe 能让 para.pos 更早变成真值、
     *   缩短上电盲区，同时也让"使能路径"只有一种写法。 */
    for (int i = 0; i < 3; i++) {
        gripper_force_probe_send(&hfdcan2);
        vTaskDelay(5);
    }
    gripper_force_init(&hfdcan2);
    dm_motor_enable(&hfdcan2, &motor[Motor7]);//不用校准//开始发送夹爪初始化控制指令
    vTaskDelay(200); // 延时，等待电机稳定

    /* ★ 2026-10-11 新增：上电自启动后夹爪回零（张开位）—— 用户明确要求
     *   "上电之后要让夹爪自动回到零点"。非阻塞：启动标志即可，实际推进由
     *   1 kHz 的主循环（DMcontrol_motor_7 → gripper_force_apply）完成。 */
    gripper_force_start_home(&hfdcan2);

    arm_cmd.ctrl_mode = ARM_ENABLE; // 使能机械臂
    arm_cmd.last_mode = ARM_ENABLE;
    vTaskDelay(1000); // 延时，等待电机稳定

    pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 1.0f); // 发送控制命令
    vTaskDelay(100); // 延时，等待电机稳定

    pos_ctrl(&hfdcan3, motor[Motor2].id, 0, 1.0f); // 发送控制命令
    vTaskDelay(100); // 延时，等待电机稳定

    for(int i=2;i<6;i++)
    {
        pos_ctrl(&hfdcan2, motor[i].id, 0, 1.0f); // 发送控制命令
        vTaskDelay(100); // 延时，等待电机稳定
    }


/* -------------------------------- 线程间Topics初始化 ------------------------------- */
    DMmotor_topic_sub_init();
    DMmotor_topic_pub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    DMmotor_task_dt = dwt_get_delta(&DMmotor_task_dwt);
    DMmotor_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        DMmotor_task_delta = dwt_get_time_ms() - DMmotor_task_start_dt;
        DMmotor_task_start_dt = dwt_get_time_ms();
        DMmotor_task_dt = dwt_get_delta(&DMmotor_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
        /** 进入此执行线程优先更新关节角信息并且做角度速度滤波处理，用于接下来的关节信息发布 **/
        dm_feedback_cache_update();
        DMmotor_topic_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        /**接收自定义控制器发来的关节命令**/
        if (xQueueReceive(xControlQueue, dm_angles, 0) == pdPASS)
        {
            for(uint8_t i=0;i<6;i++)
            {
                dm_user_motor_angles[i] = dm_angles[i];
            }
        }
        /**控制状态发生改变，将电机转到对应数组的位置**/
        if (arm_control_last_state != dm_arm_feedback_pub_msg.arm_control_state)
        {
            float *target_angles = NULL;
            switch (dm_arm_feedback_pub_msg.arm_control_state) {
                case User_defined_Controller:
                    target_angles = dm_user_motor_angles;
                    break;
                case PC_based_Controller:
                    target_angles = dm_pc_motor_angles;
                    break;
                case MCU_based_Controller:
                    target_angles = dmmotor_subscribe_movej_ref_data.q_ref_rad;
                    break;
                default:
                    break;
            }
            if (target_angles != NULL) {
                arm_mode_change_init_process(target_angles);
            }
        }
        /**根据不同的控制模式进行控制**/
        if(dm_arm_feedback_pub_msg.arm_control_state == User_defined_Controller)//自定义控制模式
        {

            DMcontrol_motor_1(&hfdcan3, &motor_controls[Motor1], dm_user_motor_angles[Motor1]);
            DMcontrol_motor_2(&hfdcan3, &motor_controls[Motor2], dm_user_motor_angles[Motor2]);
            DMcontrol_motor_3(&hfdcan2, &motor_controls[Motor3], dm_user_motor_angles[Motor3]);
            DMcontrol_motor_4(&hfdcan2, &motor_controls[Motor4], dm_user_motor_angles[Motor4]);
            DMcontrol_motor_5(&hfdcan2, &motor_controls[Motor5], dm_user_motor_angles[Motor5]);
            DMcontrol_motor_6(&hfdcan2, &motor_controls[Motor6], dm_user_motor_angles[Motor6]);
            DMcontrol_motor_7(&hfdcan2, gripper_mode_to_cmd(gripper_ctrl_mode));//夹爪控制（拨杆/键盘 → ±100）
        }
        else if(dm_arm_feedback_pub_msg.arm_control_state == PC_based_Controller)//PC控制模式
        {
            for (int i = 0; i < 6; i++)
            {
                dm_pc_motor_angles[i] = dm_receive_pc_cmd_arm_msg_data.joint_pos[i] * 57.3f;
            }
            DMcontrol_motor_1(&hfdcan3, &motor_controls[Motor1], dm_pc_motor_angles[Motor1]);
            DMcontrol_motor_2(&hfdcan3, &motor_controls[Motor2], dm_pc_motor_angles[Motor2]);
            DMcontrol_motor_3(&hfdcan2, &motor_controls[Motor3], dm_pc_motor_angles[Motor3]);
            DMcontrol_motor_4(&hfdcan2, &motor_controls[Motor4], dm_pc_motor_angles[Motor4]);
            DMcontrol_motor_5(&hfdcan2, &motor_controls[Motor5], dm_pc_motor_angles[Motor5]);
            DMcontrol_motor_6(&hfdcan2, &motor_controls[Motor6], dm_pc_motor_angles[Motor6]);
            /* ★ PC 档：直接用 0x21 来的归一化力矩指令（原为 0/1 开关）。
             *   注意：话题值被 memset 成 0 时 ⇒ τ = 0（空闲），不会像旧实现那样
             *   "上电就恒定 ±1.5 N·m 顶住"。 */
            DMcontrol_motor_7(&hfdcan2, dm_receive_pc_cmd_arm_msg_data.gripper_torque_cmd);//夹爪控制
            if(dm_receive_pc_cmd_arm_msg_data.pc_ctrl_process_state == 6 && pc_ctrl_process_last_state != 6)//完成第一个步骤
            {
                auto_ctrl_mode = AUTO_WAIT;
            }
            pc_ctrl_process_last_state = dm_receive_pc_cmd_arm_msg_data.pc_ctrl_process_state;
        }
        else if(dm_arm_feedback_pub_msg.arm_control_state == MCU_based_Controller)
        {
            if (dmmotor_subscribe_movej_ref_data.seq != dmmotor_last_movej_seq)
            {
                dmmotor_last_movej_seq = dmmotor_subscribe_movej_ref_data.seq;

                if (dmmotor_subscribe_movej_ref_data.valid)
                {
                    DMmotor_apply_movej_ref(&dmmotor_subscribe_movej_ref_data);
                }
            }
            DMcontrol_motor_7(&hfdcan2, gripper_mode_to_cmd(mcu_gripper_state));//夹爪控制//一键夹取功能
        }

/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        DMmotor_topic_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/**
 * @brief chassis 线程中所有发布者初始化
 */
static void DMmotor_topic_pub_init(void)
{
    publish_dm_arm_feedback_topic = pub_register("dm_arm_feedback_pub", sizeof(dm_arm_feedback_msg_t));

}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void DMmotor_topic_sub_init(void)
{
    subscribe_cmd_pc_arm_topic = sub_register("pc_cmd_arm_pub",sizeof(struct pc_cmd_arm_msg));
    subscribe_movej_ref_topic = sub_register("movej_ref_pub", sizeof(movej_ref_msg_t));
    publish_dm_arm_ctrl_mode_topic = sub_register("dm_arm_ctrl_mode", sizeof(Arm_mode_e));
    publish_gripper_ctrl_mode_topic = sub_register("gripper_ctrl_mode", sizeof(Gripper_mode_e ));
    publish_mcu_gripper_ctrl_mode_topic = sub_register("MCU_gripper_ctrl_mode", sizeof(Gripper_mode_e ));
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void DMmotor_topic_pub_push(void)
{
    pub_push_msg(publish_dm_arm_feedback_topic, &dm_arm_feedback_pub_msg);

}
/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void DMmotor_topic_sub_pull(void)
{
    sub_get_msg(subscribe_cmd_pc_arm_topic, &dm_receive_pc_cmd_arm_msg_data);
    sub_get_msg(subscribe_movej_ref_topic, &dmmotor_subscribe_movej_ref_data);
    sub_get_msg(publish_dm_arm_ctrl_mode_topic, &arm_control_state);
    sub_get_msg(publish_gripper_ctrl_mode_topic, &gripper_state);
    sub_get_msg(publish_mcu_gripper_ctrl_mode_topic, &mcu_gripper_state);
}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

float clamp_radians(float radians, float min_limit, float max_limit) {
    if (radians > max_limit) return max_limit;
    if (radians < min_limit) return min_limit;
    return radians;
}

void smooth_motion_1(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[0] = target_angle;  // 直接使用目标角度，无需插值
    pos_ctrl(hcan, motor->id, -current_angle[0], 5.0f);  // 符号处理保留
}

void smooth_motion_2(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[1] = target_angle;  // 保留齿轮比转换
    pos_ctrl(hcan, motor->id, current_angle[1], 5.0f);
}

void smooth_motion_3(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[2] = target_angle;
    pos_ctrl(hcan, motor->id, current_angle[2], 5.0f);//有点奇怪
}

void smooth_motion_4(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[3] = target_angle;
    pos_ctrl(hcan, motor->id, -current_angle[3], 5.0f);
}

void smooth_motion_5(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[4] = target_angle;
    pos_ctrl(hcan, motor->id, current_angle[4], 5.0f);
}

void smooth_motion_6(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[5] = target_angle;
    pos_ctrl(hcan, motor->id, -current_angle[5], 5.0f);
}

void smooth_motion_7(hcan_t* hcan, motor_t* motor, float target_rad,float target_torque,float target_vel,float kp,float kd) {
    mit_ctrl(hcan,motor,motor->id,target_rad, target_vel, kp, kd, target_torque);
}


void DMcontrol_motor_1(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated)
    {
        if(dm_user_motor_angles[Motor1] != 0 )
        {
            motor_control->calibrated = 1;
        }
        else if(dm_pc_motor_angles[Motor1] != 0 )
        {
            motor_control->calibrated = 1;
        }
    }
    else if(motor_control->calibrated == 1)
    {
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad,motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_1(hcan, &motor[Motor1], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}
void DMcontrol_motor_2(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_user_motor_angles[Motor2] != 0) {
            motor_control->calibrated = 1;
        } else if (dm_pc_motor_angles[Motor2] != 0) {
            motor_control->calibrated = 1;
        }
    } else if (motor_control->calibrated == 1) {
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);
        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_2(hcan, &motor[Motor2], angle);
        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_3(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_user_motor_angles[Motor3] != 0) {
            motor_control->calibrated = 1;
        } else if (dm_pc_motor_angles[Motor3] != 0) {
            motor_control->calibrated = 1;
        }
    } else if (motor_control->calibrated == 1) {
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);
        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_3(hcan, &motor[Motor3], angle);
        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_4(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_user_motor_angles[Motor4] != 0) {
            motor_control->calibrated = 1;
        } else if (dm_pc_motor_angles[Motor4] != 0) {
            motor_control->calibrated = 1;
        }
    } else if (motor_control->calibrated == 1) {
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);
        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_4(hcan, &motor[Motor4], angle);
        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_5(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_user_motor_angles[Motor5] != 0) {
            motor_control->calibrated = 1;
        } else if (dm_pc_motor_angles[Motor5] != 0) {
            motor_control->calibrated = 1;
        }
    } else if (motor_control->calibrated == 1) {
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);
        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_5(hcan, &motor[Motor5], angle);
        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_6(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_user_motor_angles[Motor6] != 0) {
            motor_control->calibrated = 1;
        } else if (dm_pc_motor_angles[Motor6] != 0) {
            motor_control->calibrated = 1;
        }
    } else if (motor_control->calibrated == 1) {
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);
        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);
        smooth_motion_6(hcan, &motor[Motor6], angle);
        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_7(hcan_t* hcan, int8_t cmd)
{
    /* 本函数现在只做两件事：
     *   ① 维护对 0x20 上报的【方向状态】（Gripper_mode_e 仍是两值语义）；
     *   ② 把 cmd 转交给 gripper_force 模块（MIT 模式 + 软件外环）。
     *
     * 语义不变（一个字节同时表达方向和力矩大小）：
     *   cmd == 0          → 零状态：不控开闭、也不出力
     *   cmd ∈ (+1..+100]  → 张开，力矩上限 = |cmd|/100 × GF_TAU_OPEN_NM
     *   cmd ∈ [-100..-1)  → 闭合，力矩上限 = |cmd|/100 × GF_TAU_CLOSE_NM
     * 具体换算、软件斜坡与力外环见 gripper_force.c
     * （含目标角限幅 / 速率钳位 / τ 上限迟滞冻结 / kp×lead 兜底）。 */
    if (cmd > 0)      gripper_state = Gripper_OPEN;
    else if (cmd < 0) gripper_state = Gripper_CLOSE;
    /* cmd == 0（零状态）不改变 gripper_state —— "不输出"不改变夹爪的开合归属 */

    gripper_force_apply(hcan, cmd);
}

