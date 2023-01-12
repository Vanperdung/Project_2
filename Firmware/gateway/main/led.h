/**
 * @file led.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _LED_H_
#define _LED_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define LED_ON 1
#define LED_OFF 0
#define LED_STATUS_RED GPIO_NUM_33
#define LED_STATUS_BLUE GPIO_NUM_32

void led_red_task(void *param);
void led_blue_task(void *param);

#endif
