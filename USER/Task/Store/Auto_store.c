#include "Auto_store.h"
#include "robot_task.h"
#include "transmission_task.h"

/* 键盘触发变量 */
volatile uint8_t auto_store_kb_trigger = 0;
volatile Auto_ctrl_mode auto_store_kb_target ;
int8_t  pc_ctrl_process_last_state;
uint8_t found = 0;
Store_mode_e slot ;
/* 当前自动模式 */
volatile Auto_ctrl_mode auto_ctrl_mode ;
extern struct pc_cmd_arm_msg dm_receive_pc_cmd_arm_msg_data;
/* 内部状态机 */
typedef enum {
    STORE_STEP_IDLE,
    STORE_STEP_MOVING,
    STORE_STEP_READY,
} store_step_e;

static store_step_e auto_store_step = STORE_STEP_IDLE;
static uint8_t auto_store_skip = 0;
static uint8_t auto_store_slot_idx = 0;

void auto_store_init(void)
{
    auto_store_step = STORE_STEP_IDLE;
    auto_store_skip = 0;
    auto_store_slot_idx = 0;
    auto_store_kb_trigger = 0;
}

/**
 * @brief 触发存储罐旋转（键盘或上位机调用）
 */
void auto_store_trigger(Auto_ctrl_mode target)
{
    if (auto_store_step != STORE_STEP_IDLE) return;


    StoreUnit *unit = NULL;

    if (target == AUTO_RIGHT_PLACE) {
        unit = &store_unit1;
        found = store_has_empty_slot(unit);
        if (found) slot = store_find_empty_slot(unit);
    } else if (target == AUTO_RIGHT_GRAB) {
        unit = &store_unit1;
        found = store_has_occupied_slot(unit);
        if (found) slot = store_find_occupied_slot(unit);
    } else if (target == AUTO_LEFT_PLACE) {
        unit = &store_unit2;
        found = store_has_empty_slot(unit);
        if (found) slot = store_find_empty_slot(unit);
    } else if (target == AUTO_LEFT_GRAB) {
        unit = &store_unit2;
        found = store_has_occupied_slot(unit);
        if (found) slot = store_find_occupied_slot(unit);
    }

    if (found) {
        auto_ctrl_mode = target;
        auto_store_slot_idx = (uint8_t)slot;
        store_rotate_to(unit, slot);
        auto_store_step = STORE_STEP_MOVING;
        auto_store_skip = 0;
    } else {
        auto_store_skip = 1;
        auto_store_step = STORE_STEP_READY;
    }
}

/**
 * @brief 每周期调用，等待存储罐就位
 * @return 1=执行机械臂动作, 0=无动作
 */
uint8_t auto_store_update(void)
{
    uint8_t execute = 0;

    if (auto_store_step == STORE_STEP_MOVING) {
        StoreUnit *u = (auto_ctrl_mode <= AUTO_RIGHT_GRAB) ? &store_unit1 : &store_unit2;
        if (store_ready(u)) {
            auto_store_step = STORE_STEP_READY;
        }
    } else if (auto_store_step == STORE_STEP_READY) {
        if (auto_store_skip) {
            auto_store_step = STORE_STEP_IDLE;
            auto_store_skip = 0;
        } else {
            execute = 1;
            /* 不重置step，由auto_store_complete()负责重置 */
        }
    }

    return execute;
}

/**
 * @brief 机械臂动作完成后调用，更新槽位状态
 */
void auto_store_complete(void)
{
    if(dm_receive_pc_cmd_arm_msg_data.pc_ctrl_process_state == 6 && pc_ctrl_process_last_state !=6)
    {
        if (auto_ctrl_mode == AUTO_RIGHT_PLACE) {
            store_unit1.slot_status[auto_store_slot_idx] = 1;
        } else if (auto_ctrl_mode == AUTO_RIGHT_GRAB) {
            store_unit1.slot_status[auto_store_slot_idx] = 0;
        } else if (auto_ctrl_mode == AUTO_LEFT_PLACE) {
            store_unit2.slot_status[auto_store_slot_idx] = 1;
        } else if (auto_ctrl_mode == AUTO_LEFT_GRAB) {
            store_unit2.slot_status[auto_store_slot_idx] = 0;
        }
        auto_store_step = STORE_STEP_IDLE;
        auto_store_kb_trigger = 0;
        auto_ctrl_mode = AUTO_WAIT;

    }
    pc_ctrl_process_last_state = dm_receive_pc_cmd_arm_msg_data.pc_ctrl_process_state;
}
