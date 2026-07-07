//
// Created by Áõ¼Î¿¡ on 25-4-9.
//

#ifndef CTRBOARD_H7_ALL_USART_TASK_H
#define CTRBOARD_H7_ALL_USART_TASK_H

#include "stdint.h"

void usart_rx_semaphore_init(void);
void usart_tx_semaphore_init(void);
void USART7_DebugPrintf(const char *format, ...);
void IoT_data_unpack(uint8_t *data, uint16_t len);
#endif //CTRBOARD_H7_ALL_USART_TASK_H
