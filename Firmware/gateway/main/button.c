/**
 * @file button.c
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
#include "esp_attr.h"
#include "mqtt_client.h"

#include "driver/gpio.h"

#include "mqtt.h"
#include "wifi_sta.h"
#include "button.h"
#include "ble_mesh_user.h"
#include "smartconfig.h"
#include "led.h"
#include "common.h"


static const char *TAG = "BUTTON";
extern RTC_NOINIT_ATTR int smartconfig_flag;
void button_task(void *param)
{
    TickType_t pre_tick = 0;
    button_t config = {
        .pin = BUTTON_CONFIG_PIN,
        .time_down = 0,
        .time_set = TIME_DOWN_SET,
    };

    gpio_config_t config_io;
    config_io.intr_type = GPIO_INTR_DISABLE;
    config_io.mode = GPIO_MODE_INPUT;
    config_io.pull_up_en = GPIO_PULLUP_ONLY;
    config_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config_io.pin_bit_mask = (1ULL << config.pin);
    gpio_config(&config_io);

    while (1)
    {
        if (!gpio_get_level(config.pin))
        {
            if (pre_tick == 0)
                pre_tick = xTaskGetTickCount();
            config.time_down += xTaskGetTickCount() - pre_tick;
            pre_tick = xTaskGetTickCount();
            if (config.time_down >= (config.time_set / portTICK_RATE_MS))
            {
                ESP_LOGI(TAG, "Trigger smartconfig");
                smartconfig_flag = ENABLE_SC;
                esp_restart();
            }
        }
        else
        {
            config.time_down = 0;
            pre_tick = 0;
        }
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}
