#ifndef CTRBOARD_H7_ALL_STORE_H
#define CTRBOARD_H7_ALL_STORE_H

#include "cmsis_os.h"
#include "tim.h"

#define STORE_MOVE_DELAY_MS  1000  // 舵机旋转就位延时(ms)

typedef enum
{
    Store_NO1,
    Store_NO2,
    Store_NO3,
} Store_mode_e;

typedef struct {
    Store_mode_e current_pos;   // 舵机当前位置
    uint8_t slot_status[3];     // 每个槽: 0=空, 1=有货 (索引0=NO1, 1=NO2, 2=NO3)
    uint8_t moving;             // 1=舵机正在旋转
    uint32_t move_start_tick;   // 开始旋转的时间戳
} StoreUnit;

extern StoreUnit store_unit1;
extern StoreUnit store_unit2;

void store_init(void);
void store_ctrl(void);
Store_mode_e store_find_empty_slot(StoreUnit *unit);
Store_mode_e store_find_occupied_slot(StoreUnit *unit);
uint8_t store_has_empty_slot(StoreUnit *unit);
uint8_t store_has_occupied_slot(StoreUnit *unit);
void store_rotate_to(StoreUnit *unit, Store_mode_e target);
uint8_t store_ready(StoreUnit *unit);

#endif //CTRBOARD_H7_ALL_STORE_H
