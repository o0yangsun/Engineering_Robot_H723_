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
    Gripper_OPEN,       /* 0 —— 真实命令：张开 */
    Gripper_CLOSE,      /* 1 —— 真实命令：闭合 */
    /* ★ 2026-10-10 新增：Gripper_FREE = **"没有命令"**，不是"命令张开"。
     *
     *   为什么必须与 Gripper_OPEN 分开：
     *     `Gripper_OPEN` 的数值是 0，而 C 里全局/静态变量**零初始化天然就是 0**
     *     ⇒ 上电时所有命令源（`gripper_ctrl_mode` / `mcu_gripper_state` / `MCU_gripper_ctrl`）
     *       看起来都已经在"命令张开"。旧实现下这会让夹爪**一上电就自己跑到全开位**
     *       （改 PSI 之前是"一上电就恒定 +1.5 N·m 顶住"，见评审 P0-2）。
     *     加一个非 0 的 FREE 之后，零初始化 = "还没人下过命令" ⇒ 上电零状态。
     *
     *   行为：`gripper_mode_to_cmd(FREE) = 0` ⇒ 车端 PSI 零状态（不出力、不控开闭、可自由掰）。
     *   上电后**拨杆/键盘/一键夹取第一次动作**，才会变成 OPEN 或 CLOSE。
     *
     *   ⚠️ 这是**车端内部状态**，不上总线：0x20 上报时会被归一化成 0/1（见
     *      `DMmotor_task.c` 里 `dm_arm_feedback_pub_msg.gripper_state = ...`），
     *      PC 侧的 `GRIPPER_OPEN(0)/GRIPPER_CLOSE(1)` 语义保持不变。 */
    Gripper_FREE,
} Gripper_mode_e;

/* ★ 全局唯一的「夹爪控制量」。
 *   定义在 keyboard.c:55（键盘 X/Z 与遥控器拨杆都写它）。
 *   消费点：DMmotor_task.c:472（【只在 User_defined_Controller 档】读它）。
 *
 *   ⚠️ 历史坑（2026-10-08 修）：cmd_task.c 里曾另有一份同名的 file-static 变量，
 *      而 DMmotor_task.c:62 的 extern 指向的是本声明（= keyboard.c 那份）。
 *      ⇒ 拨杆代码写那份 static 等于【没写】，且编译/运行都不会有任何告警。
 *      症状是"拨杆取消注释后夹爪纹丝不动"，极易误判成硬件坏。
 *      现在统一到这一个符号上，任何新代码要改夹爪都必须写它。 */
extern Gripper_mode_e gripper_ctrl_mode;

typedef enum
{
    User_defined_Controller, //自定义模式控制器
    PC_based_Controller,    //上位机控制
    MCU_based_Controller    //下位机自动模式
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

