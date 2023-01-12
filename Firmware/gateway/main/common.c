/**
 * @file common.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-26
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_device.h"
#include "mqtt_client.h"

#include "driver/gpio.h"

#include "cJSON.h"
#include "mqtt.h"
#include "wifi_sta.h"
#include "button.h"
#include "ble_mesh_user.h"
#include "common.h"
#include "heartbeat.h"

static const char *TAG = "COMMON";

esp_err_t mqtt_parse_data(char *mqtt_data, mqtt_obj_t *mqtt_obj)
{
    cJSON *root = cJSON_Parse(mqtt_data);
    if (root == NULL)
        return ESP_FAIL;
    cJSON *cur_elem = NULL;
    cJSON_ArrayForEach(cur_elem, root)
    {
        if (cur_elem->string)
        {
            const char *cur_str = cur_elem->string;
            if (strcmp(cur_str, "action") == 0)
                memcpy(mqtt_obj->action, cur_elem->valuestring, strlen(cur_elem->valuestring) + 1);
            else if (strcmp(cur_str, "url") == 0)
                memcpy(mqtt_obj->url, cur_elem->valuestring, strlen(cur_elem->valuestring) + 1);
            else if (strcmp(cur_str, "state") == 0)
                mqtt_obj->state = cur_elem->valueint;
            else if (strcmp(cur_str, "unicast_addr") == 0)
                mqtt_obj->unicast_addr = cur_elem->valueint;
            else if (strcmp(cur_str, "timeout") == 0)
                mqtt_obj->timeout = cur_elem->valueint;
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}
