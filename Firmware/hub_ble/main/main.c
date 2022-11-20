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

RTC_NOINIT_ATTR int smartconfig_flag;
static const char *TAG = "MAIN";
status_t status = LOCAL_MODE;
char hub_id[15] = {0};
char topic_commands_version[50] = {0};
char topic_commands_status[50] = {0};
char topic_commands_control[50] = {0};
char topic_commands_process[50] = {0};
char topic_commands_ping[50] = {0};
char topic_commands_node_connected[50] = {0};
char topic_messages_control[50] = {0};
char topic_messages_update[50] = {0};
char topic_messages_status[50] = {0};
const char *VERSION = "0.0.2";
unsigned char node_list; 
extern esp_mqtt_client_handle_t client; 

void app_main(void)
{
    esp_err_t ret;
    TickType_t tick = 0;
    char ping_data[20] = "{\"ping\":1}";
    ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    char hub_hex_id[6] = {0};
    esp_efuse_mac_get_default((uint8_t*)hub_hex_id);
    sprintf(hub_id, "%02x%02x%02x%02x%02x%02x", hub_hex_id[0],
                                                hub_hex_id[1],
                                                hub_hex_id[2],
                                                hub_hex_id[3],
                                                hub_hex_id[4],
                                                hub_hex_id[5]);
    ESP_LOGI(TAG, "Hub ID: %s", hub_id);
    sprintf(topic_commands_version, "commands/%s/version", hub_id);
    sprintf(topic_commands_control, "commands/%s/control", hub_id);
    sprintf(topic_commands_process, "commands/%s/process", hub_id);
    sprintf(topic_commands_status, "commands/%s/status", hub_id);
    sprintf(topic_commands_ping, "commands/%s/ping", hub_id);
    sprintf(topic_messages_control, "messages/%s/control", hub_id);
    sprintf(topic_messages_update, "messages/%s/update", hub_id);
    sprintf(topic_messages_status, "messages/%s/status", hub_id);
    sprintf(topic_commands_node_connected, "messages/%s/node_connected", hub_id);
    mount_SPIFFS();
    xTaskCreate(&button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(&led_task, "led_task", 2048, NULL, 5, NULL);
    wifi_init();
    ESP_ERROR_CHECK(ret);
    if(smartconfig_flag == ENABLE_SC)
    {
        smartconfig_flag = DISABLE_SC;
        status = SMARTCONFIG;
        smartconfig_init();
    }
    else
    {
        wifi_config_t wifi_cfg = {
            .sta = {
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                .pmf_cfg = {
					.capable = true,
					.required = false
				}
            }
        };
        if(esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_cfg) == ESP_OK)
        {
            ESP_LOGI(TAG, "Wifi configuration already stored in flash partition called NVS");
            ESP_LOGI(TAG, "%s", wifi_cfg.sta.ssid);
            ESP_LOGI(TAG, "%s", wifi_cfg.sta.password);
            ble_init();
            wifi_sta(wifi_cfg, WIFI_MODE_STA);
            while(status != NORMAL_MODE)
            {
                vTaskDelay(100 / portTICK_RATE_MS);
            }
            mqtt_client_sta();
            // xTaskCreate(&ble_task, "ble_task", 4096, NULL, 10, NULL);
        }
        while(1)
        {
            if(status == NORMAL_MODE && xTaskGetTickCount() - tick >= 5000 / portTICK_RATE_MS)
            {
                tick = xTaskGetTickCount();
                esp_mqtt_client_publish(client, topic_commands_ping, ping_data, strlen(ping_data), 0, 0);
            }
            else
                vTaskDelay(100 / portTICK_RATE_MS);
        }
    }
}

