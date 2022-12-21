/**
 * @file nvs_user.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _NVS_USER_H_
#define _NVS_USER_H

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"

esp_err_t ble_mesh_nvs_open(nvs_handle_t *handle);
esp_err_t ble_mesh_nvs_store(nvs_handle_t handle, const char *key, const void *data, size_t length);
esp_err_t ble_mesh_nvs_get_length(nvs_handle_t handle, const char *key, size_t *length);
esp_err_t ble_mesh_nvs_restore(nvs_handle_t handle, const char *key, void *data, size_t length, bool *exist);
esp_err_t ble_mesh_nvs_erase(nvs_handle_t handle, const char *key);

#endif