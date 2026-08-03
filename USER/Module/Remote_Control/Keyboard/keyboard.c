//
// Created by 刘嘉俊 on 25-3-8.
//

#include "keyboard.h"
#include "FreeRTOS.h"
#include "robot.h"
#include "referee_system.h"
#include "user_lib.h"
#include "ramp.h"
#include "chassis_task.h"
#include "robot_task.h"
#include "pump.h"
#include "DMmotor_task.h"
#include "cmd_task.h"
#include "tim.h"
#include "algorithm_task.h"
#include "usart_task.h"

/* key acceleration time */
#define KEY_ACC_TIME     2200  //ms

extern struct referee_fdb_msg referee_fdb;
extern struct cmd_chassis_msg cmd_chassis;
extern ramp_obj_t *km_vx_ramp;//x轴控制斜坡
extern ramp_obj_t *km_vy_ramp;//y周控制斜坡
extern ramp_obj_t *km_vw_ramp; // 旋转控制斜坡，需在外部定义

extern ramp_obj_t *nuc_km_vx_ramp;//x轴控制斜坡
extern ramp_obj_t *nuc_km_vy_ramp;//y周控制斜坡
extern ramp_obj_t *nuc_km_vw_ramp; // 旋转控制斜坡，需在外部定义

//-------------储存罐参数--------------
extern Auto_ctrl_mode auto_ctrl_mode;
uint8_t left_full[2] = {0};   // 左侧两个罐子是否有物体
uint8_t right_full[2] = {0};   // 右侧两个罐子
//-------------储存罐参数--------------

static float base_delta = 3.0f  / KEY_ACC_TIME;
static float base_delta_w = MAX_CHASSIS_VW_SPEED  / KEY_ACC_TIME;

/* 时间参数宏定义 */
#define LONG_PRESS_DEFAULT_MS   800   // 默认长按时间
#define SHIFT_LONG_PRESS_MS     500   // SHIFT长按时间
#define FUNCTION_KEY_PRESS_MS   300   // 功能键长按时间
/* 控制参数定义 ------------------------------------------------------------*/
#define LONG_PRESS_TIME       600     // 长按判定时间(ms)
#define DEBOUNCE_TIME         10      // 消抖时间(ms)
#define MICRO_SENSITIVITY     0.4f    // CTRL微调灵敏度系数
#define BOOST_FACTOR          1.2f    // SHIFT加速倍率
#define NORMAL_DECAY          0.85f   // 常规衰减系数
#define MICRO_DECAY           0.95f   // 微调模式衰减系数
#define DEAD_ZONE             5.0f    // 速度死区(mm/s)

Gripper_mode_e gripper_ctrl_mode;
// 全局键盘控制对象定义
keyboard_control_t keyboard = {
        .vx = 0, .vy = 0, .vw = 0,
        .max_spd = 3000,
        .move_mode = NORMAL_MODE,
        .shift = {KEY_RELEASE, 0, 500, 0},   // SHIFT长按800ms
        .ctrl  = {KEY_RELEASE, 0, 500, 0},   // CTRL长按800ms
        .v     = {KEY_RELEASE, 0, 800, 0},   // V键短按800ms
        .b     = {KEY_RELEASE, 0, 800, 0},   // V键短按800ms
        .g     = {KEY_RELEASE, 0, 800, 0},   // G键快速响应
        .f     = {KEY_RELEASE, 0, 800, 0},    // F键快速响应
        .x     = {KEY_RELEASE, 0, 800, 0},    // F键快速响应
        .z     = {KEY_RELEASE, 0, 800, 0},
        .r     = {KEY_RELEASE, 0, 800, 0}    // F键快速响应
};

keyboard_control_t nuc_keyboard = {
        .vx = 0, .vy = 0, .vw = 0,
        .max_spd = 3000,
        .move_mode = NORMAL_MODE,
        .shift = {KEY_RELEASE, 0, 500, 0},   // SHIFT长按800ms
        .ctrl  = {KEY_RELEASE, 0, 500, 0},   // CTRL长按800ms
        .v     = {KEY_RELEASE, 0, 800, 0},   // V键短按800ms
        .b     = {KEY_RELEASE, 0, 800, 0},   // V键短按800ms
        .g     = {KEY_RELEASE, 0, 800, 0},   // G键快速响应
        .f     = {KEY_RELEASE, 0, 800, 0},    // F键快速响应
        .x     = {KEY_RELEASE, 0, 800, 0},    // F键快速响应
        .z     = {KEY_RELEASE, 0, 800, 0},
        .r     = {KEY_RELEASE, 0, 800, 0}    // F键快速响应
};
mouse_control_t mouse = {0} ;

void key_state_machine(key_status_t *key, uint8_t key_input)
{
    switch (key->state)
    {
        case KEY_RELEASE:
        {
            if (key_input)
                key->state = KEY_WAIT_EFFECTIVE;
            else
                key->state = KEY_RELEASE;
        } break;

        case KEY_WAIT_EFFECTIVE:
        {
            if (key_input)
                key->state = KEY_PRESS_DOWN;
            else
                key->state = KEY_RELEASE;
        } break;

        case KEY_PRESS_DOWN:
        {
            if (key_input)
            {
                key->state = KEY_PRESS_ONCE;
                // 根据具体情况选择左键还是右键计数重置
                if (key->state == mouse.lk_state)
                    mouse.lk_cnt = 0;
                else
                    mouse.rk_cnt = 0;
            }
            else
                key->state = KEY_RELEASE;
        } break;

        case KEY_PRESS_ONCE:
        {
            if (key_input)
            {
                if (key->state == mouse.lk_state)
                {
                    if (mouse.lk_cnt++ > LONG_PRESS_TIME )
                        key->state = KEY_PRESS_LONG;
                }
                else
                {
                    if (mouse.rk_cnt++ > LONG_PRESS_TIME )
                        key->state = KEY_PRESS_LONG;
                }
            }
            else
                key->state = KEY_RELEASE;
        } break;

        case KEY_PRESS_LONG:
        {
            if (!key_input)
            {
                key->state = KEY_RELEASE;
            }
        } break;

        default:
            break;
    }
}

/*
* @brief 将裁判系统解析后的键盘鼠标数据转换成方便使用的遥控器数据结构体
* @param remote 指向原始裁判系统数据的指针
* @return 转换后的遥控器数据结构体
*/
pc_control_t convert_remote_to_pc(const vt13_remote_parsed_data_t *remote)
{
    pc_control_t pc;

    if(remote == NULL)
    {
        // 如果需要，可以在此处理错误情况
        pc.mouse.x = 0;
        pc.mouse.y = 0;
        pc.mouse.z = 0;
        pc.mouse.l = 0;
        pc.mouse.r = 0;
        pc.keyboard.key_code = 0;
        return pc;
    }

    pc.mouse.x = remote->mouse_x;
    pc.mouse.y = remote->mouse_y;
    pc.mouse.z = remote->mouse_z;
    pc.mouse.l = (uint8_t)remote->mouse_left;
    pc.mouse.r = (uint8_t)remote->mouse_right;
    pc.keyboard.key_code = remote->key;

    return pc;
}


volatile uint8_t gripper_cmd_updated = 0;   // 1表示有新命令待执行
extern struct arm_cmd_msg arm_cmd;

void PC_keyboard_mouse(const pc_control_t *pc_control)
{

    key_state_machine(&keyboard.x, pc_control->keyboard.bit.X);
    key_state_machine(&keyboard.z, pc_control->keyboard.bit.Z);
    // X按键用于打开夹爪
    if(keyboard.x.state == KEY_PRESS_ONCE) {
        gripper_ctrl_mode = Gripper_OPEN;
        gripper_cmd_updated =1;
    }
    // Z按键用于关闭夹爪
    if(keyboard.z.state == KEY_PRESS_ONCE) {
        gripper_ctrl_mode = Gripper_CLOSE;
        gripper_cmd_updated =1;
    }

//    key_state_machine(&keyboard.g,pc_control->keyboard.bit.G);
//    if (keyboard.g.state == KEY_PRESS_ONCE)
//    {
//        cmd_chassis.last_mode= cmd_chassis.ctrl_mode;
//        cmd_chassis.ctrl_mode =CHASSIS_RELAX;
//    }
//
//    key_state_machine(&keyboard.f,pc_control->keyboard.bit.F);
//    if (keyboard.f.state == KEY_PRESS_ONCE)
//    {
//        cmd_chassis.last_mode= cmd_chassis.ctrl_mode;
//        cmd_chassis.ctrl_mode=CHASSIS_ENABLE;
//    }

    key_state_machine(&keyboard.b,pc_control->keyboard.bit.B);
    if (keyboard.b.state == KEY_PRESS_LONG)
    {
        arm_cmd.last_mode = arm_cmd.ctrl_mode;
        arm_cmd.ctrl_mode = ARM_DISABLE;
    }

    key_state_machine(&keyboard.v,pc_control->keyboard.bit.V);
    if (keyboard.v.state == KEY_PRESS_LONG)
    {
        arm_cmd.last_mode = arm_cmd.ctrl_mode;
        arm_cmd.ctrl_mode = ARM_ENABLE;
    }

    /* 模式优先级处理 */
    keyboard.move_mode = NORMAL_MODE;
    keyboard.max_spd = 0.8f;
    // 先处理CTRL
    key_state_machine(&keyboard.ctrl, pc_control->keyboard.bit.CTRL);
    if(keyboard.ctrl.state == KEY_PRESS_LONG) {
        keyboard.move_mode = SLOW_MODE;
        keyboard.max_spd = 0.4f;
    }
    // 后处理SHIFT（更高优先级）
    key_state_machine(&keyboard.shift, pc_control->keyboard.bit.SHIFT);
    if(keyboard.shift.state == KEY_PRESS_LONG) {
        keyboard.move_mode = FAST_MODE;
        keyboard.max_spd = 1.2f;
    }

    /* 计算动态参数 */
    float delta = (keyboard.move_mode == FAST_MODE) ?
                  (base_delta * BOOST_FACTOR) :
                  (keyboard.move_mode == SLOW_MODE) ?
                  (base_delta * MICRO_SENSITIVITY) :
                  base_delta;

    float decay = (keyboard.move_mode == SLOW_MODE) ?
                  MICRO_DECAY : NORMAL_DECAY;


    // 前后方向（W/S -> vy）
    if(pc_control->keyboard.bit.W) {
        keyboard.vx += delta;
    } else if(pc_control->keyboard.bit.S) {
        keyboard.vx -= delta;
    } else {
        keyboard.vx *= (1 - km_vy_ramp->calc(km_vy_ramp) * decay);
    }

    // 左右方向（A/D -> vx）
    if(pc_control->keyboard.bit.A) {
        keyboard.vy -= delta;
    } else if(pc_control->keyboard.bit.D) {
        keyboard.vy += delta;
    } else {     //TODO: 加速斜坡函数反转，变成减速斜坡函数，加速阶段不使用斜坡函数
        keyboard.vy *= (1 - km_vy_ramp->calc(km_vy_ramp) * decay);
    }

    // 旋转控制（Q/E）
    if(pc_control->keyboard.bit.Q) {
        keyboard.vw -= base_delta_w * 1.25f;  // 旋转灵敏度系数
    } else if(pc_control->keyboard.bit.E) {
        keyboard.vw += base_delta_w * 1.25f;
    } else {
        keyboard.vw *= (1 - km_vw_ramp->calc(km_vw_ramp) * decay);
    }

//    // 死区处理
//    if(fabs(keyboard.vx) < DEAD_ZONE) keyboard.vx = 0;
//    if(fabs(keyboard.vy) < DEAD_ZONE) keyboard.vy = 0;
//    if(fabs(keyboard.vw) < DEAD_ZONE) keyboard.vw = 0;


    VAL_LIMIT(keyboard.vx, -keyboard.max_spd, keyboard.max_spd);
    VAL_LIMIT(keyboard.vy, -keyboard.max_spd, keyboard.max_spd);

    VAL_LIMIT(keyboard.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
    VAL_LIMIT(keyboard.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
    VAL_LIMIT(keyboard.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);



//    key_state_machine(&keyboard.r,pc_control->keyboard.bit.R);
//    if (keyboard.r.state == KEY_PRESS_ONCE)
//    {
//        arm_cmd.last_mode = arm_cmd.ctrl_mode;
//        arm_cmd.ctrl_mode = ARM_INIT;
//    }
}
uint32_t store_1_pwm_set = 1833;
uint32_t store_2_pwm_set = 1833;
void NUC_keyboard_mouse(const pc_control_t *pc_control)
{

    key_state_machine(&nuc_keyboard.x, pc_control->keyboard.bit.X);
    key_state_machine(&nuc_keyboard.z, pc_control->keyboard.bit.Z);
    // X按键用于打开夹爪
    if(nuc_keyboard.x.state == KEY_PRESS_ONCE) {
        gripper_ctrl_mode = Gripper_OPEN;
        gripper_cmd_updated =1;
    }
    // Z按键用于关闭夹爪
    if(nuc_keyboard.z.state == KEY_PRESS_ONCE) {
        gripper_ctrl_mode = Gripper_CLOSE;
        gripper_cmd_updated =1;
    }

//    key_state_machine(&keyboard.g,pc_control->keyboard.bit.G);
//    if (keyboard.g.state == KEY_PRESS_ONCE)
//    {
//        cmd_chassis.last_mode= cmd_chassis.ctrl_mode;
//        cmd_chassis.ctrl_mode =CHASSIS_RELAX;
//    }
//
//    key_state_machine(&keyboard.f,pc_control->keyboard.bit.F);
//    if (keyboard.f.state == KEY_PRESS_ONCE)
//    {
//        cmd_chassis.last_mode= cmd_chassis.ctrl_mode;
//        cmd_chassis.ctrl_mode=CHASSIS_ENABLE;
//    }

    //------------------------------------------储存罐控制-----------------------------------------


    key_state_machine(&nuc_keyboard.g,pc_control->keyboard.bit.G);
    key_state_machine(&nuc_keyboard.r,pc_control->keyboard.bit.R);
    key_state_machine(&nuc_keyboard.f,pc_control->keyboard.bit.F);
    key_state_machine(&nuc_keyboard.b,pc_control->keyboard.bit.B);

// 左侧放置（原G键，右侧请自行调整）
    if (nuc_keyboard.g.state == KEY_PRESS_LONG && auto_ctrl_mode == AUTO_WAIT)
    {
        USART7_DebugPrintf("get nuc keyboard cmd\r\n");
        for (int i = 0; i < 2; i++)
        {
            if (!left_full[i])
            {
                left_full[i] = 1;
                AlgorithmTask_RunSequence(SEQ_LEFT_PLACE);
                store_2_pwm_set = i ? 1833 : 500;
                break;
            }
        }
    }

// 左侧抓取（原R键）
    if (nuc_keyboard.r.state == KEY_PRESS_LONG && auto_ctrl_mode == AUTO_WAIT) {

        for (int i = 0; i < 2; i++) {
            if (left_full[i]) {
                AlgorithmTask_RunSequence(SEQ_LEFT_GRAB);
                left_full[i] = 0;
                store_2_pwm_set = i ? 1833 : 500;
                break;
            }
        }
    }

    // 右侧放置（原F键）
    if (nuc_keyboard.f.state == KEY_PRESS_LONG && auto_ctrl_mode == AUTO_WAIT) {

        for (int i = 0; i < 2; i++) {
            if (!right_full[i]) {
                AlgorithmTask_RunSequence(SEQ_RETURN_TO_CENTER);
                right_full[i] = 1;
                store_1_pwm_set = i ? 1833 : 500;

                break;
            }
        }
    }

// 右侧抓取（原B键）
    if (nuc_keyboard.b.state == KEY_PRESS_LONG && auto_ctrl_mode == AUTO_WAIT) {

        for (int i = 0; i < 2; i++) {
            if (right_full[i]) {
                AlgorithmTask_RunSequence(SEQ_LEFT_GRAB_RIGHT_PLACE);
                right_full[i] = 0;
                store_1_pwm_set = i ? 1833 : 500;

                break;
            }
        }
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 ,store_1_pwm_set);//500-2500
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, store_2_pwm_set);//500-2500

    //------------------------------------------储存罐控制-----------------------------------------


//    key_state_machine(&nuc_keyboard.b,pc_control->keyboard.bit.B);
//    if (nuc_keyboard.b.state == KEY_PRESS_LONG)
//    {
//        arm_cmd.last_mode = arm_cmd.ctrl_mode;
//        arm_cmd.ctrl_mode = ARM_DISABLE;
//    }
//
//    key_state_machine(&nuc_keyboard.v,pc_control->keyboard.bit.V);
//    if (nuc_keyboard.v.state == KEY_PRESS_LONG)
//    {
//        arm_cmd.last_mode = arm_cmd.ctrl_mode;
//        arm_cmd.ctrl_mode = ARM_ENABLE;
//    }

    /* 模式优先级处理 */
    nuc_keyboard.move_mode = NORMAL_MODE;
    nuc_keyboard.max_spd = 0.8f;
    // 先处理CTRL
    key_state_machine(&nuc_keyboard.ctrl, pc_control->keyboard.bit.CTRL);
    if(nuc_keyboard.ctrl.state == KEY_PRESS_LONG) {
        nuc_keyboard.move_mode = SLOW_MODE;
        nuc_keyboard.max_spd = 0.4f;
    }
    // 后处理SHIFT（更高优先级）
    key_state_machine(&nuc_keyboard.shift, pc_control->keyboard.bit.SHIFT);
    if(nuc_keyboard.shift.state == KEY_PRESS_LONG) {
        nuc_keyboard.move_mode = FAST_MODE;
        nuc_keyboard.max_spd = 1.2f;
    }

    /* 计算动态参数 */
    float delta = (nuc_keyboard.move_mode == FAST_MODE) ?
                  (base_delta * BOOST_FACTOR) :
                  (nuc_keyboard.move_mode == SLOW_MODE) ?
                  (base_delta * MICRO_SENSITIVITY) :
                  base_delta;

    float decay = (nuc_keyboard.move_mode == SLOW_MODE) ?
                  MICRO_DECAY : NORMAL_DECAY;


    // 前后方向（W/S -> vy）
    if(pc_control->keyboard.bit.W) {
        nuc_keyboard.vx += delta;
    } else if(pc_control->keyboard.bit.S) {
        nuc_keyboard.vx -= delta;
    } else {
        nuc_keyboard.vx *= (1 - nuc_km_vx_ramp->calc(nuc_km_vx_ramp) * decay);
    }

    // 左右方向（A/D -> vx）
    if(pc_control->keyboard.bit.A) {
        nuc_keyboard.vy -= delta;
    } else if(pc_control->keyboard.bit.D) {
        nuc_keyboard.vy += delta;
    } else {
        nuc_keyboard.vy *= (1 - nuc_km_vy_ramp->calc( nuc_km_vy_ramp) * decay);
    }

    // 旋转控制（Q/E）
    if(pc_control->keyboard.bit.Q) {
        nuc_keyboard.vw -= base_delta_w * 1.25f;  // 旋转灵敏度系数
    } else if(pc_control->keyboard.bit.E) {
        nuc_keyboard.vw += base_delta_w * 1.25f;
    } else {
        nuc_keyboard.vw *= (1 -nuc_km_vw_ramp->calc(nuc_km_vw_ramp) * decay);
    }

//    // 死区处理
//    if(fabs(keyboard.vx) < DEAD_ZONE) keyboard.vx = 0;
//    if(fabs(keyboard.vy) < DEAD_ZONE) keyboard.vy = 0;
//    if(fabs(keyboard.vw) < DEAD_ZONE) keyboard.vw = 0;


    VAL_LIMIT(nuc_keyboard.vx, -nuc_keyboard.max_spd, nuc_keyboard.max_spd);
    VAL_LIMIT(nuc_keyboard.vy, -nuc_keyboard.max_spd, nuc_keyboard.max_spd);

    VAL_LIMIT(nuc_keyboard.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
    VAL_LIMIT(nuc_keyboard.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
    VAL_LIMIT(nuc_keyboard.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);



//    key_state_machine(&keyboard.r,pc_control->keyboard.bit.R);
//    if (keyboard.r.state == KEY_PRESS_ONCE)
//    {
//        arm_cmd.last_mode = arm_cmd.ctrl_mode;
//        arm_cmd.ctrl_mode = ARM_INIT;
//    }
}



