//
// Created by 刘嘉俊 on 25-1-6.
//

#ifndef CTRBOARD_H7_ALL_CMD_TASK_H
#define CTRBOARD_H7_ALL_CMD_TASK_H
#include "cmsis_os.h"

void remote_to_cmd_sbus(void);
void Gripper_ctrl(void);



typedef enum
{
    Gripper_OPEN,
    Gripper_CLOSE,
} Gripper_mode_e;

typedef enum
{
    User_defined_Controller, //自定义模式控制器
    PC_based_Controller,    //上位机控制
} Arm_mode_e;



typedef enum
{
    AUTO_WAIT,         // 0: 等待
    AUTO_RIGHT_PLACE,   // 1: 左边放置
    AUTO_RIGHT_GRAB,    // 2: 左边抓取
    AUTO_LEFT_PLACE,  // 3: 右边放置
    AUTO_LEFT_GRAB    // 4: 右边抓取
} Auto_ctrl_mode;

#endif //CTRBOARD_H7_ALL_CMD_TASK_H
