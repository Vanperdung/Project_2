/**
 * @file button.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-26
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define BUTTON_CONFIG_PIN GPIO_NUM_23
#define TIME_DOWN_SET 3000

void button_task(void *param);

typedef struct
{
    uint64_t time_down;
    uint8_t pin;
    uint64_t time_set;
} button_t;

#endif