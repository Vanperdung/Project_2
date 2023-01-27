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
extern RTC_NOINIT_ATTR int gateway_mode_flag;
void button_task(void *param)
{
    button_t button = {
        .pin = BUTTON_CONFIG_PIN,
        .time_down = 0,
        .time_up = 0,
        .deltaT = 0,
        .click_cnt = 0,
        .time_stamp = 0,
    };

    gpio_config_t config_io;
    config_io.intr_type = GPIO_INTR_DISABLE;
    config_io.mode = GPIO_MODE_INPUT;
    config_io.pull_up_en = GPIO_PULLUP_ONLY;
    config_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config_io.pin_bit_mask = (1ULL << button.pin);
    gpio_config(&config_io);

    while (1)
    {
        if (gpio_get_level(button.pin) == BUTTON_TRIGGER)
        {
            if (button.time_up == 0)
                button.time_up = (uint32_t)(xTaskGetTickCount() / portTICK_RATE_MS);
            else
            {
                button.deltaT = (uint32_t)(xTaskGetTickCount() / portTICK_RATE_MS) - button.time_up;
            }
        }
        else if (gpio_get_level(button.pin) == BUTTON_NOT_TRIGGER && button.time_up != 0 && button.deltaT > TIME_CLICK_MIN)
        {
            button.time_down = (uint32_t)(xTaskGetTickCount() / portTICK_RATE_MS);
            button.deltaT = button.time_down - button.time_up;
            ESP_LOGI(TAG, "DeltaT: %d", button.deltaT);
            if (button.deltaT > TIME_HOLD)
            {
                ESP_LOGI(TAG, "Trigger Smartconfig");
                gateway_mode_flag = SMARTCONFIG_MODE;
                esp_restart();
            }
            else if (button.deltaT > TIME_CLICK_MIN && button.deltaT < TIME_CLICK_MAX)
            {
                button.click_cnt++;
                ESP_LOGI(TAG, "Button counter: %d", button.click_cnt);
                if (button.click_cnt == 5)
                {
                    ESP_LOGI(TAG, "Trigger SoftAP");
                    gateway_mode_flag = WIFI_SOFTAP_MODE;
                    esp_restart();
                }
                else
                {
                    button.time_stamp = button.time_up;
                    button.time_up = 0;
                    button.time_down = 0;
                    button.deltaT = 0;
                }
            }
        }
        else if (gpio_get_level(button.pin) == BUTTON_NOT_TRIGGER && (uint32_t)(xTaskGetTickCount() / portTICK_RATE_MS) - button.time_stamp > TIME_RESET && button.time_stamp != 0)
        {
            button.time_up = 0;
            button.time_down = 0;
            button.deltaT = 0;
            button.click_cnt = 0;
            button.time_stamp = 0;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}
