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

extern char hub_id[15];
extern char topic_commands_version[50];
extern char topic_commands_status[50];
extern char topic_commands_control[50];
extern char topic_commands_process[50];
extern char topic_commands_ping[50];
extern char topic_commands_node_connected[50];
extern char topic_messages_control[50];
extern char topic_messages_update[50];
extern char topic_messages_status[50];
extern esp_mqtt_client_handle_t client;
extern status_t status;
extern const uint8_t github_cert_pem_start[] asm("_binary_git_ota_pem_start");
extern const uint8_t github_cert_pem_end[] asm("_binary_git_ota_pem_end");
static const char *TAG = "FOTA";

esp_err_t ota_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key = %s, value = %s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len = %d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

void fota_task(void *param)
{
    char ota_url[100] = {0};
    strcpy(ota_url, (char *)param);
    ESP_LOGI(TAG, "FOTA start, url: %s", ota_url);
    status = FOTA;
    esp_http_client_config_t ota_cfg = {
        .url = ota_url,
        .event_handler = ota_event_handler,
        .keep_alive_enable = true,
        .cert_pem = (char *)github_cert_pem_start};
    esp_err_t ret = esp_https_ota(&ota_cfg);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "OTA done, restarting...");
        esp_mqtt_client_publish(client, (char *)topic_commands_process, "{\"update\":\"fota done\"}", strlen("{\"update\":\"fota done\"}"), 0, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG, "OTA failed...");
    }
}