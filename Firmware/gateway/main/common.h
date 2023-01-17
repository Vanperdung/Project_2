/**
 * @file common.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-26
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define ENABLE_SC 11
#define DISABLE_SC 20

typedef enum
{
    LOCAL_MODE,
    NORMAL_MODE,
} status_red_t;

typedef enum
{
    NOT_STATE,
    POWER_ON_PROVISIONING,
    SMARTCONFIG,
    FOTA,
    PROVISIONING,
} status_blue_t;

typedef struct
{
    char action[15];
    int unicast_addr;
    int state;
    char url[100];
    // char uuid[16];
    // char node_type[10];
    // int switch_num;
    // int runtime;
    int timeout;
    
} mqtt_obj_t;

esp_err_t mqtt_parse_data(char *mqtt_data, mqtt_obj_t *mqtt_obj);

#endif