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
#include "Auto_store.h"
#include "msg_freertos.h"
#include "transmission_task.h"
#include "robot_task.h"


/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

static struct pc_cmd_arm_msg dm_receive_pc_cmd_arm_msg_data = {0};
static subscriber_t *subscribe_cmd_pc_arm_topic;
static dm_arm_feedback_msg_t dm_arm_feedback_pub_msg = {0};
static publisher_t *publish_dm_arm_feedback_topic = NULL;
static subscriber_t *publish_dm_arm_ctrl_mode_topic = NULL;
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
static float dm_motor_angles[6] = {0.0f};   // 期望角度值
Gripper_mode_e gripper_state = Gripper_OPEN;
//User_defined_Controller, //自定义模式控制器
//PC_based_Controller,    //上位机控制
static Arm_mode_e arm_control_state = User_defined_Controller;
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
        arm_cmd.last_mode = ARM_DISABLE;  // 修复：初始化后等待用户使能
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

 float Kp_track = 2.7f;      // 先从小值开始调
 float Kv_track = 0.95f;

static void DMmotor_apply_movej_ref(const movej_ref_msg_t *ref)
{
    float pos_fdb[6];
    float vel_fdb[6];

    float pos_err[6];
    float vel_err[6];

    float pos_cmd[6];
    float vel_cmd[6];

    const float v_min_follow = 0.4f; // 有误差时最小追赶速度
    const float v_max_exec   = 6.0f;  // 执行层最大速度
    const float pos_tol      = 0.001f; // 约 0.57 度
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
    dm_arm_feedback_pub_msg.gripper_state = gripper_state;
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
    // execute_track_movej_planner_pid = pid_register(&execute_track_movej_config);
    /* ------------------------ 规划器与执行器分割线 --------------------------------- */

    /* -------------------------------- 外设初始化段落 ------------------------------- */
    for (int i = 0; i < 6; i++) {
        motor_controls[i].current_angle_rad = 0.0f;
        motor_controls[i].last_angle_rad = 0.0f;
        motor_controls[i].initial_offset_rad = 0.0f;
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
    dm_motor_enable(&hfdcan2, &motor[Motor7]);//不用校准//开始发送夹爪初始化控制指令
    vTaskDelay(200); // 延时，等待电机稳定

    arm_cmd.ctrl_mode = ARM_ENABLE; // 使能机械臂
    arm_cmd.last_mode = ARM_ENABLE;
    vTaskDelay(2000); // 延时，等待电机稳定
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
    if(dm_arm_feedback_pub_msg.arm_control_state == User_defined_Controller)//自定义控制模式
    {
        if (xQueueReceive(xControlQueue, dm_angles, 0) == pdPASS)
        {
            for(uint8_t i=0;i<6;i++){
                dm_motor_angles[i] = dm_angles[i];
            }
            DMcontrol_motor_1(&hfdcan3, &motor_controls[Motor1], dm_motor_angles[Motor1]);
            DMcontrol_motor_2(&hfdcan3, &motor_controls[Motor2], dm_motor_angles[Motor2]);
            DMcontrol_motor_3(&hfdcan2, &motor_controls[Motor3], dm_motor_angles[Motor3]);
            DMcontrol_motor_4(&hfdcan2, &motor_controls[Motor4], dm_motor_angles[Motor4]);
            DMcontrol_motor_5(&hfdcan2, &motor_controls[Motor5], dm_motor_angles[Motor5]);
            DMcontrol_motor_6(&hfdcan2, &motor_controls[Motor6], dm_motor_angles[Motor6]);

        }
        DMcontrol_motor_7(&hfdcan2,dm_arm_feedback_pub_msg.gripper_state);//PC控制模式
    }
    else if(dm_arm_feedback_pub_msg.arm_control_state == PC_based_Controller)
    {
        /* 检测键盘触发信号 */
        if (auto_store_kb_trigger)
        {
            auto_store_trigger(auto_store_kb_target);
            auto_store_kb_trigger = 0;
        }

        /* 存储罐就位状态机 */
        uint8_t arm_execute = auto_store_update();

        /* 机械臂动作执行 */
        if (arm_execute) {
            dm_motor_angles[0] = dm_receive_pc_cmd_arm_msg_data.joint_pos[0] * 57.3f;
            dm_motor_angles[1] = dm_receive_pc_cmd_arm_msg_data.joint_pos[1] * 57.3f;
            dm_motor_angles[2] = dm_receive_pc_cmd_arm_msg_data.joint_pos[2] * 57.3f;
            dm_motor_angles[3] = dm_receive_pc_cmd_arm_msg_data.joint_pos[3] * 57.3f;
            dm_motor_angles[4] = dm_receive_pc_cmd_arm_msg_data.joint_pos[4] * 57.3f;
            dm_motor_angles[5] = dm_receive_pc_cmd_arm_msg_data.joint_pos[5] * 57.3f;
            DMcontrol_motor_1(&hfdcan3, &motor_controls[Motor1], dm_motor_angles[Motor1]);
            DMcontrol_motor_2(&hfdcan3, &motor_controls[Motor2], dm_motor_angles[Motor2]);
            DMcontrol_motor_3(&hfdcan2, &motor_controls[Motor3], dm_motor_angles[Motor3]);
            DMcontrol_motor_4(&hfdcan2, &motor_controls[Motor4], dm_motor_angles[Motor4]);
            DMcontrol_motor_5(&hfdcan2, &motor_controls[Motor5], dm_motor_angles[Motor5]);
            DMcontrol_motor_6(&hfdcan2, &motor_controls[Motor6], dm_motor_angles[Motor6]);
            auto_store_complete();
        }

        DMcontrol_motor_7(&hfdcan2, dm_receive_pc_cmd_arm_msg_data.gripper_ctrl);
    }



//        if (dmmotor_subscribe_movej_ref_data.seq != dmmotor_last_movej_seq)
//        {
//            dmmotor_last_movej_seq = dmmotor_subscribe_movej_ref_data.seq;
//
//            if (dmmotor_subscribe_movej_ref_data.valid)
//            {
//                DMmotor_apply_movej_ref(&dmmotor_subscribe_movej_ref_data);
//            }
//        }
//        DMcontrol_motor_7(&hfdcan2,gripper_state);//夹爪控制//一个抓取动作
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
}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

float clamp_radians(float radians, float min_limit, float max_limit) {
    if (radians > max_limit) return max_limit;
    if (radians < min_limit) return min_limit;
    return radians;
}

void smooth_motion_1(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[0] = target_angle;  // 直接使用目标角度，无需插值
    pos_ctrl(hcan, motor->id, -current_angle[0], 10.0f);  // 符号处理保留
}

void smooth_motion_2(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[1] = target_angle;  // 保留齿轮比转换
    pos_ctrl(hcan, motor->id, current_angle[1], 10.0f);
}

void smooth_motion_3(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[2] = target_angle;
    pos_ctrl(hcan, motor->id, current_angle[2], 10.0f);//有点奇怪
}

void smooth_motion_4(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[3] = target_angle;
    pos_ctrl(hcan, motor->id, -current_angle[3], 10.0f);
}

void smooth_motion_5(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[4] = target_angle;
    pos_ctrl(hcan, motor->id, current_angle[4], 10.0f);
}

void smooth_motion_6(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[5] = target_angle;
    pos_ctrl(hcan, motor->id, -current_angle[5], 10.0f);
}

void smooth_motion_7(hcan_t* hcan, motor_t* motor, float target_rad,float target_torque,float target_vel,float kp,float kd) {
    mit_ctrl(hcan,motor,motor->id,target_rad, target_vel, kp, kd, target_torque);
}

void DMcontrol_motor_1(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor1] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor1]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1) {
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad,motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_1(hcan, &motor[Motor1], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_2(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor2] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor2]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_2(hcan, &motor[Motor2], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_3(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor3] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor3]);
            motor_control->calibrated = 1;
        }

    }else if(motor_control->calibrated == 1){

        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_3(hcan, &motor[Motor3], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_4(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor4] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor4]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_4(hcan, &motor[Motor4], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_5(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor5] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor5]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_5(hcan, &motor[Motor5], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_6(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor6] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor6]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_6(hcan, &motor[Motor6], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;

    }
}

void DMcontrol_motor_7(hcan_t* hcan,Gripper_mode_e Gripper_ctrl)
{
    float target_rad,target_torque,target_vel,target_kp,target_kd;
    if(Gripper_ctrl == Gripper_OPEN)//一键抓取模式,在这里调参
    {
        target_rad = 0.0f;
        target_torque = 1.5f;
        target_vel = 0.0f;
        target_kp = 0.0f;
        target_kd = 0.5f;
        gripper_state = Gripper_OPEN;//改变夹爪状态报给上位机
    }
    else//关闭
    {
        target_rad = 0.0f;
        target_torque = -1.5f;
        target_vel = 0.0f;
        target_kp = 0.0f;
        target_kd = 0.5f;
        gripper_state = Gripper_CLOSE;
    }

    smooth_motion_7(hcan, &motor[Motor7], target_rad, target_torque ,target_vel,target_kp,target_kd);
}

