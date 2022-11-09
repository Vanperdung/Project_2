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

static const char *TAG = "LED";
extern status_t status;    

void led_status_init(void)
{
    gpio_config_t led_status_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0, 
        .pin_bit_mask = (1ULL << LED_STATUS_PIN)
    };
    gpio_config(&led_status_cfg);
    gpio_set_level(LED_STATUS_PIN, LED_OFF);
    ESP_LOGI(TAG, "Led status init");
}

void led_task(void *param)
{
    led_status_init();
    while(1)
    {
        switch(status)
        {
            case LOCAL_MODE:
                gpio_set_level(LED_STATUS_PIN, LED_ON);
                vTaskDelay(100 / portTICK_RATE_MS);
                gpio_set_level(LED_STATUS_PIN, LED_OFF);
                vTaskDelay(100 / portTICK_RATE_MS);
                break;
            case NORMAL_MODE:
                gpio_set_level(LED_STATUS_PIN, LED_ON);
                vTaskDelay(100 / portTICK_RATE_MS);
                break;
            case SMARTCONFIG:   
                gpio_set_level(LED_STATUS_PIN, LED_ON);
                vTaskDelay(100 / portTICK_RATE_MS);
                gpio_set_level(LED_STATUS_PIN, LED_OFF);
                vTaskDelay(100 / portTICK_RATE_MS);
                break;
            case FOTA:
                gpio_set_level(LED_STATUS_PIN, LED_ON);
                vTaskDelay(1000 / portTICK_RATE_MS);
                gpio_set_level(LED_STATUS_PIN, LED_OFF);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            default:
                gpio_set_level(LED_STATUS_PIN, LED_OFF);
                gpio_set_level(LED_STATUS_PIN, LED_OFF);
                vTaskDelay(100 / portTICK_RATE_MS);
                break;  
        }
    }
}