//
// Created by Áõ¼Î¿¡ on 25-3-21.
//

#ifndef CTRBOARD_H7_ALL_ALGORITHM_TASK_H
#define CTRBOARD_H7_ALL_ALGORITHM_TASK_H
#include <stdint.h>
#include <stdbool.h>

#define SEQ_RIGHT_GRAB   1
#define SEQ_RIGHT_PLACE  2
#define SEQ_LEFT_GRAB    3
#define SEQ_LEFT_PLACE   4

bool AlgorithmTask_RunSequence(uint8_t seq_id);

#endif //CTRBOARD_H7_ALL_ALGORITHM_TASK_H
