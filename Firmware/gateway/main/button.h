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
#define TIME_HOLD (3000 / portTICK_RATE_MS)
#define TIME_CLICK_MIN (20 / portTICK_RATE_MS)
#define TIME_CLICK_MAX (1000 / portTICK_RATE_MS)
#define TIME_RESET (1000 / portTICK_RATE_MS)

#define BUTTON_TRIGGER 0
#define BUTTON_NOT_TRIGGER 1

void button_task(void *param);

typedef struct
{
    uint32_t time_down;
    uint8_t pin;
    uint32_t time_up;
    uint32_t deltaT;
    uint8_t click_cnt;
    uint32_t time_stamp;
} button_t;

#endif