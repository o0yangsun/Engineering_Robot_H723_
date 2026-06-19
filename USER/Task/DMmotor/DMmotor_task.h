//
// Created by 刘嘉俊 on 25-1-14.
//

#ifndef CTRBOARD_H7_ALL_DMMOTOR_TASK_H
#define CTRBOARD_H7_ALL_DMMOTOR_TASK_H
#include "cmsis_os.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"
#include "cmd_task.h"
#include "rc_sbus.h"
// 转换宏定义
#define DEG_TO_RAD(x) ((x) * (M_PI / 180.0f)) // 度数转弧度
#define RAD_TO_DEG(x) ((x) * (180.0f / M_PI)) // 弧度转度数
#define NUM_INITIAL_READINGS 10 // 设置读取次数

// 六号电机的角度限制
#define MOTOR_6_MIN_LIMIT (-1.57f)
#define MOTOR_6_MAX_LIMIT (1.57f)


// 五号电机的角度限制
#define MOTOR_5_MIN_LIMIT (-1.57f)  //  5号电机转动 1圈，末端齿轮转动 1.5556圈,限幅九十度
#define MOTOR_5_MAX_LIMIT (1.57f)

// 四号电机的角度限制
#define MOTOR_4_MIN_LIMIT (-3.14f)  // 最多3.14
#define MOTOR_4_MAX_LIMIT 3.14f

// 三号电机的角度限制
#define MOTOR_3_MIN_LIMIT -0.05f
#define MOTOR_3_MAX_LIMIT 2.65f    // 点位说明，2为即将越过点位，2.6朝天，3.1越出点位，4.2反向垂直，4.9垂直吸盘，5.2极限

// 计算二号电机的角度限制
#define MOTOR_2_MIN_LIMIT (-2.1f)
#define MOTOR_2_MAX_LIMIT (0.05f)//一定要留正数，如果填-0.01，因为反馈角度不精准，所以可能会反馈0.0005，导致一直规划失败

// 计算一号电机的角度限制
#define MOTOR_1_MIN_LIMIT (-2.4f)
#define MOTOR_1_MAX_LIMIT 2.4f

void arm_cmd_state_machine(void);
void arm_cmd_enable(void);
void arm_cmd_disable(void);

typedef struct {
    float motor_min_limit;       // 电机最小角度限制
    float motor_max_limit;       // 电机最大角度限制
    float initial_offset_rad;        // 电机的初始偏差（弧度）

    float current_angle_rad;            // 电机的齿轮比
    float last_angle_rad;            // 电机上一次的角度（弧度）
    int calibrated;              // 校准状态
} DMmotorControl;

typedef enum
{
    ARM_DISABLE,
    ARM_ENABLE,
    ARM_INIT,
} arm_mode_e;

struct arm_cmd_msg
{
    arm_mode_e ctrl_mode;
    arm_mode_e last_mode;
};

/** 关节电机角度速度处理相关函数 **/
#define POS_DEADBAND_RAD        0.0002f
#define VEL_DEADBAND_RAD_S      0.009f
#define MEDIAN_WIN_SIZE         5u  // 5窗口中值滤波

typedef struct
{
    float buf[MEDIAN_WIN_SIZE];
    uint8_t index;
    uint8_t inited;
} median_filter5_t;

/** 关节电机角度速度处理相关函数 **/


// 限幅函数
float clamp_radians(float radians, float min_limit, float max_limit);

void DMcontrol_motor_1(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_2(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_3(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_4(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_5(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_6(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_7(hcan_t* hcan,Gripper_mode_e Gripper_ctrl);



#endif //CTRBOARD_H7_ALL_DMMOTOR_TASK_H