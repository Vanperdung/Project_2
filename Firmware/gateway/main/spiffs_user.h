/**
 * @file spiffs_user.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-01-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _SPIFFS_USER_H_
#define _SPIFFS_USER_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

void mount_SPIFFS(void);
esp_err_t write_to_file(char *file_name, char *buf);
esp_err_t read_from_file(char *file_name, char *buf);
void remove_file(char *file_name);

#endif