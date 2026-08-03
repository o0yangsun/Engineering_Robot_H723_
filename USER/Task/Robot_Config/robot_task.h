//
// Created by 刘嘉俊 on 25-6-21.
//

#ifndef CTRBOARD_H7_ALL_ROBOT_TASK_H
#define CTRBOARD_H7_ALL_ROBOT_TASK_H

#include <cmsis_os.h>
#include "chassis_task.h"
#include "cmd_task.h"

#ifdef BSP_USING_EXAMPLE_TASK
#include "example_task.h"
#endif /* BSP_USING_EXAMPLE_TASK */
#ifdef BSP_USING_INS_TASK
#include "ins_task.h"
#endif /* BSP_USING_INS_TASK */
#ifdef BSP_USING_MOTOR_TASK
#include "motor_task.h"
#endif /* BSP_USING_MOTOR_TASK */
#ifdef BSP_USING_CMD_TASK
#include "cmd_task.h"
#endif /* BSP_USING_CMD_TASK */
#ifdef BSP_USING_CHASSIS_TASK
#include "chassis_task.h"
#endif /* BSP_USING_CHASSIS_TASK */
#ifdef BSP_USING_GIMBAL_TASK
#include "gimbal_task.h"
#endif /* BSP_USING_GIMBAL_TASK */
#ifdef BSP_USING_TRANSMISSION_TASK
#include "transmission_task.h"
#endif /* BSP_USING_TRANSMISSION_TASK */
#ifdef BSP_USING_SHOOT_TASK
#include "shoot_task.h"
#endif /* BSP_USING_SHOOT_TASK */
#ifdef BSP_USING_REFEREE_TASK
#include "referee_task.h"
#include "Referee_system.h"
#endif /* BSP_USING_REFEREE_TASK */


/** -------------------------------- Algorithm_Task Topics_Msg ------------------------------- **/
typedef struct
{
    uint32_t seq;                 // 发布序号，每发一次+1
    uint32_t tick_ms;             // 时间戳
    uint8_t  valid;               // 1=本消息有效
    uint8_t  planner_state;       // JP_IDLE / JP_RUNNING / JP_DONE / JP_FAULT

    float q_ref_rad[6];           // 目标关节角(rad)
    float v_ref_rad_s[6];         // 目标关节速度(rad/s)

    float q_fb_rad[6];            // 可选：当前反馈角(rad)
    float v_fb_rad_s[6];          // 可选：当前反馈速度(rad/s)
} dm_arm_movej_target_msg_t;

typedef struct
{
    uint32_t seq;                 // 发布序号，每发一次+1
    uint32_t tick_ms;             // 时间戳
    uint8_t  valid;               // 1=本消息有效
    uint8_t  planner_state;       // JP_IDLE / JP_RUNNING / JP_DONE / JP_FAULT

    float q_ref_rad[6];           // 目标关节角(rad)
    float v_ref_rad_s[6];         // 目标关节速度(rad/s)

    float q_fb_rad[6];            // 可选：当前反馈角(rad)
    float v_fb_rad_s[6];          // 可选：当前反馈速度(rad/s)
} movej_ref_msg_t;
/** -------------------------------- Algorithm_Task Topics_Msg ------------------------------- **/

/** -------------------------------- DMmotor_Task Topics_Msg ------------------------------- **/
typedef struct
{
    uint8_t id;
    uint8_t state;
    float pos_rad;      // 位置弧度rad
    float vel_rad_s;    // vel 目前是 rad/s
    float tor_nm;       // tor
    float mos_temp;
    float coil_temp;
} dm_joint_feedback_t;

typedef struct
{
    uint32_t update_mask;      // 哪些电机本周期更新过
    uint32_t tick_ms;          // 时间戳，可选
    dm_joint_feedback_t joint[6];   // 0~5 对应关节1~6
    Gripper_mode_e gripper_state;
    Arm_mode_e arm_control_state;
    Auto_ctrl_mode auto_ctrl_mode;
} dm_arm_feedback_msg_t;
/** -------------------------------- DMmotor_Task Topics_Msg ------------------------------- **/

/** -------------------------------- Ins_Task Topics_Msg ------------------------------- **/
struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    float motion_accel_b[3]; // 机体坐标加速度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;
};
/** -------------------------------- Ins_Task Topics_Msg ------------------------------- **/

/** -------------------------------- pose_Task Topics_Msg ------------------------------- **/
struct pose_msg
{
    // 位姿
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

};
/** -------------------------------- pose_Task Topics_Msg ------------------------------- **/

/** -------------------------------- Cmd_Task Topics_Msg ------------------------------- **/
struct cmd_chassis_msg
{
    float vx;                  // 前进方向速度
    float vy;                  // 横移方向速度
    float vw;                  // 旋转速度
    chassis_mode_e ctrl_mode;  // 当前底盘控制模式
    chassis_mode_e last_mode;  // 上一次底盘控制模式
};

struct pc_cmd_voice_control_msg
{
    float vx;                  // 前进方向速度
    float vy;                  // 横移方向速度
    float vw;                  // 旋转速度
};

/** -------------------------------- Cmd_Task Topics_Msg ------------------------------- **/

/** --------------------------- Transmission_Task Topics_Msg --------------------------- **/
struct transmission_msg
{
    float yaw;
    float pitch;
    uint8_t heartbeat;
};
/** --------------------------- Transmission_Task Topics_Msg --------------------------- **/

/** ------------------------- Chassis_Task Feedback Topics_Msg ------------------------- **/
struct chassis_feedback_msg
{
    float x_pos_gim;
    float y_pos_gim;
};
/** ------------------------- Chassis_Task Feedback Topics_Msg ------------------------ **/


#endif //CTRBOARD_H7_ALL_ROBOT_TASK_H
