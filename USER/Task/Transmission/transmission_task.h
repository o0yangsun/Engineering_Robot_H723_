//
// Created by 刘嘉俊 on 25-3-31.
//

#ifndef CTRBOARD_H7_ALL_TRANSMISSION_TASK_H
#define CTRBOARD_H7_ALL_TRANSMISSION_TASK_H


#include <stdint.h>
#include "cmd_task.h"

struct pc_cmd_arm_msg
{
    float joint_pos[6];
    uint8_t control_state;
    /* 2026-10-09: 由 Gripper_mode_e(0/1 开关) 改成【归一化力矩指令 -100..+100】。
     *   车端换算(2026-10-10 起): 已改为 PSI 模式(位置 + 速度 + 电流上限)。
 *     cmd 的符号 = 开/合方向;
 *     |cmd|/100 * {GF_TAU_OPEN_NM=1.0, GF_TAU_CLOSE_NM=1.5} = 力矩上限(N*m);
 *     电流 = 力矩上限 / GF_KT_NM_PER_A (A);
 *     cur 字段 = (电流 / GF_I_FS_A) * 10000  <-- DIAO MIAN ZHI x10000, FAN WEI 0~10000!
 *       (yuan xian xie cheng "an pei x10000" shi CUO DE, 2026-10-10 gai)
 *       依据: guan fang DM_CAN.py: control_pos_force() 注释
 *       "i_des: desired current rang 0-10000 qi wang dian liu biao yao zhi fang da 10000 bei"
 *     实现见 USER/Task/DMmotor/gripper_force.c 。
 *   旧的 tau = cmd/100 * DM7_TAU_MAX 已随 MIT 路径下线。
     *   动机: 支持"主臂第7路角度 -> 夹爪力矩"的连续映射(死区 + 比例 + 限幅),
     *         取代原来"二值开关 + 恒定力矩"的做法。
     *   注意: 字节数不变(仍 1 字节), 所以 0x21 的 LEN 仍是 27, 协议长度不用改。
     *   旧语义对照: 0 = 张开(tau 正), 1 = 闭合(tau 负) —— 见 DMmotor_task.c:700 起。 */
    int8_t gripper_torque_cmd;
    int8_t pc_ctrl_process_state;
};


typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_ADDRESS  = 1,
    STEP_ID  = 2,
    STEP_LEN  = 3,
    STEP_DATA  = 4,
    STEP_SC  = 5,
    STEP_AC  = 6,

} unpack_transmission_step_e;


#endif //CTRBOARD_H7_ALL_TRANSMISSION_TASK_H
