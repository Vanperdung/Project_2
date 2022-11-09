#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_wifi.h"
#include "esp_tls.h"
#include "esp_smartconfig.h"
#include "esp_attr.h"
#include "mqtt_client.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#include "cJSON.h"
#include "ble_hub.h"
#include "button.h"
#include "fota.h"
#include "led.h"
#include "mqtt.h"
#include "smartconfig.h"
#include "spiffs_user.h"
#include "wifi_sta.h"
#include "common.h"
#include "main.h"

bool ble_cJSON_process(char *data, ble_object_t *ble_object)
{
    cJSON *root = cJSON_Parse(data);
    if(root == NULL)
        return false;
    cJSON *cur_element = NULL;
    cJSON_ArrayForEach(cur_element, root)
    {
        if(cur_element->string)
        {
            const char *cur_string = cur_element->string;
            if(strcmp(cur_string, "type_node") == 0)
                memcpy(ble_object->type_node, cur_element->valuestring, strlen(cur_element->valuestring) + 1);
            else if(strcmp(cur_string, "mac") == 0)
                sprintf(ble_object->mac_addr, "%02x%02x%02x%02x%02x%02x", cur_element->valuestring[0],
                                                                        cur_element->valuestring[1],
                                                                        cur_element->valuestring[2],
                                                                        cur_element->valuestring[3],
                                                                        cur_element->valuestring[4],
                                                                        cur_element->valuestring[5]);
            else if(strcmp(cur_string, "node_id") == 0)
                ble_object->node_id = cur_element->valueint;
        }
    }
    cJSON_Delete(root);
    return true;
}

bool mqtt_cJSON_process(char *data, mqtt_object_t *mqtt_object)
{
    cJSON *root = cJSON_Parse(data);
    if(root == NULL)
        return false;
    cJSON *cur_element = NULL;
    cJSON_ArrayForEach(cur_element, root)
    {
        if(cur_element->string)
        {
            const char *cur_string = cur_element->string;
            if(strcmp(cur_string, "action") == 0)
                memcpy(mqtt_object->action, cur_element->valuestring, strlen(cur_element->valuestring) + 1);
            else if(strcmp(cur_string, "node_id") == 0)
                memcpy(mqtt_object->node_id, cur_element->valuestring, strlen(cur_element->valuestring) + 1);
            else if(strcmp(cur_string, "end_point") == 0)
                mqtt_object->end_point = cur_element->valueint;
            else if(strcmp(cur_string, "control") == 0)
                mqtt_object->control = cur_element->valueint;
            else if(strcmp(cur_string, "url") == 0)
                memcpy(mqtt_object->url, cur_element->valuestring, strlen(cur_element->valuestring) + 1);
            else if(strcmp(cur_string, "duration") == 0)
                mqtt_object->duration = cur_element->valueint;
        }
    }
    cJSON_Delete(root);
    return true;
}