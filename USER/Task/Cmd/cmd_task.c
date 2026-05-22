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
#include <string.h>
#include "cmd_task.h"
#include "robot.h"
#include "robot_task.h"
#include "stm32h7xx_hal.h"
#include "rc_sbus.h"
#include "ramp.h"
#include "drv_dwt.h"
#include "usart.h"
#include "keyboard.h"
#include "DMmotor_task.h"
#include "pump.h"
#include "vt13_vt03.h"
#include "ins_task.h"
#include "msg_freertos.h"
#include "chassis_task.h"
#include "store.h"
#include "tim.h"
#include "store.h"
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

static struct cmd_chassis_msg pc_cmd_data;
static uint16_t receive_nuc_keyboard_data;
static struct pc_cmd_voice_control_msg receive_pc_cmd_voice_control_data;
static publisher_t *chassis_cmd_pub;
static subscriber_t *pc_cmd_sub;
static publisher_t *dm_arm_ctrl_mode_pub;
static subscriber_t *pc_cmd_voice_control_subscriber;
static subscriber_t *nuc_keyboard_subscriber;
static void cmd_pub_init(void);
static void cmd_sub_init(void);
static void cmd_pub_push(void);
static void cmd_sub_pull(void);

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t cmd_task_dwt = 0;   // 毫秒监测
static float cmd_task_dt = 0;       // 线程实际运行时间dt
static float cmd_task_delta = 0;    // 监测线程运行时间
static float cmd_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

extern sbus_data_t sbus_data_fdb;
extern keyboard_control_t keyboard;
extern keyboard_control_t nuc_keyboard;
extern vt13_remote_parsed_data_t vt13_remote_parsed_data_fdb;
static pc_control_t pc_data;
static pc_control_t nuc_data;
static Arm_mode_e dm_arm_ctrl_mode;

extern struct referee_fdb_msg referee_fdb;
struct cmd_chassis_msg cmd_chassis;
extern Gripper_mode_e gripper_state ;

/* 外部变量声明 */
/*键盘加速度的斜坡*/
ramp_obj_t *km_vx_ramp = NULL;;//x轴控制斜坡
ramp_obj_t *km_vy_ramp = NULL;//y周控制斜坡
ramp_obj_t *km_vw_ramp = NULL;//y周控制斜坡

ramp_obj_t *nuc_km_vx_ramp = NULL;;//x轴控制斜坡
ramp_obj_t *nuc_km_vy_ramp = NULL;//y周控制斜坡
ramp_obj_t *nuc_km_vw_ramp = NULL;//y周控制斜坡

/* 气泵控制状态 */

/* -------------------------------- 线程入口 ------------------------------- */
void CmdTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    sbus_data_init();
    sbus_data_fdb.sw1 = RC_UP;
    sbus_data_fdb.sw2 = RC_UP;
    sbus_data_fdb.sw3 = RC_UP;
    sbus_data_fdb.sw4 = RC_UP;

    vt13_remote_data_init();

    km_vx_ramp = ramp_register(0, 200); //2500000
    km_vy_ramp = ramp_register(0, 200);  // 0 -2的累加次数
    km_vw_ramp = ramp_register(0, 200);

    nuc_km_vx_ramp = ramp_register(0, 200); //2500000
    nuc_km_vy_ramp = ramp_register(0, 200);  // 0 -2的累加次数
    nuc_km_vw_ramp = ramp_register(0, 200);

    /* 获取原始键盘数据 */
    memset(&pc_data, 0, sizeof(pc_control_t));
    memset(&keyboard, 0, sizeof(keyboard_control_t));
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
    cmd_pub_init();
    cmd_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    cmd_task_dt = dwt_get_delta(&cmd_task_dwt);
    cmd_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        cmd_task_delta = dwt_get_time_ms() - cmd_task_start_dt;
        cmd_task_start_dt = dwt_get_time_ms();
        cmd_task_dt = dwt_get_delta(&cmd_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
        cmd_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        pc_data = convert_remote_to_pc(&vt13_remote_parsed_data_fdb);
        nuc_data.keyboard.key_code = receive_nuc_keyboard_data;//只解析16位键盘数据
        PC_keyboard_mouse(&pc_data);
        NUC_keyboard_mouse(&nuc_data);
        remote_to_cmd_sbus();
        arm_cmd_state_machine(); // 机械臂状态机
        chassis_cmd_state_machine();
        store_ctrl();//存储罐控制
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        cmd_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */


/**
 * @brief cmd_task 线程中所有发布者初始化
 */
static void cmd_pub_init(void)
{
    chassis_cmd_pub = pub_register("chassis_cmd_pub", sizeof(struct cmd_chassis_msg));
    dm_arm_ctrl_mode_pub = pub_register("dm_arm_ctrl_mode", sizeof(Arm_mode_e));
}


/**
 * @brief cmd_task  线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    pc_cmd_sub = sub_register("pc_cmd_chassis", sizeof(struct cmd_chassis_msg));
    pc_cmd_voice_control_subscriber = sub_register("voice_control_pub",sizeof(struct pc_cmd_voice_control_msg));
    nuc_keyboard_subscriber = sub_register("nuc_keyboard_data",sizeof(uint16_t));
}

/**
 * @brief cmd_task  线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    sub_get_msg(pc_cmd_sub, &pc_cmd_data);
    sub_get_msg(pc_cmd_voice_control_subscriber, &receive_pc_cmd_voice_control_data);
    sub_get_msg(nuc_keyboard_subscriber, &receive_nuc_keyboard_data);
}

/**
 * @brief cmd_task  线程中所有订阅者推送话题
 */
static void cmd_pub_push(void)
{
    pub_push_msg(chassis_cmd_pub,&cmd_chassis);
    pub_push_msg(dm_arm_ctrl_mode_pub, &dm_arm_ctrl_mode);
}

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

static uint8_t fn_1_last_state = 0;  // 保存上次的状态,初始为未按下
static uint8_t fn_2_last_state = 0;  // 保存上次的状态,初始为未按下
extern struct arm_cmd_msg arm_cmd;
/* ------------------------------ 将遥控器数据转换为控制指令 ----------------------------- */
void remote_to_cmd_sbus(void) {

    cmd_chassis.last_mode = cmd_chassis.ctrl_mode;
    //导航控制、福斯控制和VT13控制,以加法手段结合
    if (vt13_remote_parsed_data_fdb.online) {
        // 新遥控器（vt13）通道映射
        cmd_chassis.vx = (vt13_remote_parsed_data_fdb.ch[1] * CHASSIS_VT13_RC_MOVE_RATIO_X / VT13_RC_MAX_VALUE
                          + keyboard.vx * CHASSIS_PC_MOVE_RATIO_Y + pc_cmd_data.vx+receive_pc_cmd_voice_control_data.vx + nuc_keyboard.vx * CHASSIS_PC_MOVE_RATIO_X);
        cmd_chassis.vy = (vt13_remote_parsed_data_fdb.ch[3] * CHASSIS_VT13_RC_MOVE_RATIO_Y / VT13_RC_MAX_VALUE
                          + keyboard.vy * CHASSIS_PC_MOVE_RATIO_X + pc_cmd_data.vy+receive_pc_cmd_voice_control_data.vy + nuc_keyboard.vy * CHASSIS_PC_MOVE_RATIO_Y);
        cmd_chassis.vw = (vt13_remote_parsed_data_fdb.ch[0] * CHASSIS_VT13_RC_MOVE_RATIO_W / VT13_RC_MAX_VALUE
                          + keyboard.vw * CHASSIS_PC_MOVE_RATIO_W + pc_cmd_data.vw+receive_pc_cmd_voice_control_data.vw + nuc_keyboard.vw * CHASSIS_PC_MOVE_RATIO_W);

        if (vt13_remote_parsed_data_fdb.mode_sw == 0)//夹爪控制模式
        {
            gripper_state =Gripper_OPEN ;
        }
        else if(vt13_remote_parsed_data_fdb.mode_sw == 2)
        {
            gripper_state =Gripper_CLOSE;
        }
        //底盘失使能
        if(vt13_remote_parsed_data_fdb.fn_1 && !fn_1_last_state)
        {
            dm_arm_ctrl_mode = !dm_arm_ctrl_mode;
        }
        fn_1_last_state = vt13_remote_parsed_data_fdb.fn_1;
        //机械臂失使能
        if(vt13_remote_parsed_data_fdb.fn_2 && !fn_2_last_state )
        {
            arm_cmd.ctrl_mode = !arm_cmd.ctrl_mode;
        }
        fn_2_last_state = vt13_remote_parsed_data_fdb.fn_2;
    } else {
        // 原SBUS遥控器数据（保持原有逻辑）
            cmd_chassis.vx = (sbus_data_fdb.ch2 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE
                              + keyboard.vx * CHASSIS_PC_MOVE_RATIO_X + pc_cmd_data.vx+receive_pc_cmd_voice_control_data.vx + nuc_keyboard.vx * CHASSIS_PC_MOVE_RATIO_X);
            cmd_chassis.vy = (sbus_data_fdb.ch4 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE
                              + keyboard.vy * CHASSIS_PC_MOVE_RATIO_Y + pc_cmd_data.vy+receive_pc_cmd_voice_control_data.vy + nuc_keyboard.vy * CHASSIS_PC_MOVE_RATIO_Y);
            cmd_chassis.vw = (sbus_data_fdb.ch1 * CHASSIS_RC_MOVE_RATIO_W / RC_MAX_VALUE
                              + keyboard.vw * CHASSIS_PC_MOVE_RATIO_W + pc_cmd_data.vw+receive_pc_cmd_voice_control_data.vw + nuc_keyboard.vw * CHASSIS_PC_MOVE_RATIO_W);
        // 原SBUS遥控器泵模式控制（保持原有逻辑）
        if (sbus_data_fdb.sw3 == RC_MI) {
            gripper_state = Gripper_OPEN;
        } else if (sbus_data_fdb.sw3 == RC_DN) {
            gripper_state = Gripper_CLOSE;
        }

        if (sbus_data_fdb.sw2 == RC_UP) {
            cmd_chassis.ctrl_mode = CHASSIS_ENABLE;
        } else if (sbus_data_fdb.sw2 == RC_DN) {
            cmd_chassis.ctrl_mode = CHASSIS_RELAX;
        }

        if (sbus_data_fdb.sw1 == RC_UP) {
            arm_cmd.ctrl_mode = ARM_ENABLE;
        } else if (sbus_data_fdb.sw1 == RC_DN) {
            arm_cmd.ctrl_mode = ARM_DISABLE;
        }
    }
}

