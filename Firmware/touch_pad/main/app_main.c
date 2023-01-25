/**
 * @file app_main.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "nvs_flash.h"

#include "driver/gpio.h"

#define ON 1
#define OFF 0

static const char *TAG = "MAIN";

typedef struct
{
    uint8_t current_state;
    uint8_t target_state;
    uint8_t pin;
} smart_switch_t;

typedef struct
{
    uint8_t pin;
    uint8_t state;
} touch_pad_t;

smart_switch_t smart_switchs[4] = {
    [0] = {.pin = GPIO_NUM_19, .current_state = ON},
    [1] = {.pin = GPIO_NUM_18, .current_state = ON},
    [2] = {.pin = GPIO_NUM_17, .current_state = ON},
    [3] = {.pin = GPIO_NUM_16, .current_state = ON},
};

touch_pad_t touch_pads[4] = {
    [0] = {.pin = GPIO_NUM_4},
    [1] = {.pin = GPIO_NUM_21},
    [2] = {.pin = GPIO_NUM_22},
    [3] = {.pin = GPIO_NUM_23},
};

void gpio_init_input(gpio_num_t num)
{
    gpio_config_t config_io;
    config_io.intr_type = GPIO_INTR_DISABLE;
    config_io.mode = GPIO_MODE_INPUT;
    config_io.pull_up_en = GPIO_PULLUP_DISABLE;
    config_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config_io.pin_bit_mask = (1ULL << num);
    gpio_config(&config_io);
}

void touch_pad_task(void *param)
{
    gpio_init_input(GPIO_NUM_4);
    gpio_init_input(GPIO_NUM_21);
    gpio_init_input(GPIO_NUM_22);
    gpio_init_input(GPIO_NUM_23);
    while (1)
    {
        ESP_LOGI(TAG, "GPIO NUM %d: %d    GPIO NUM %d: %d    GPIO NUM %d: %d    GPIO NUM %d: %d", touch_pads[0].pin, gpio_get_level(touch_pads[0].pin), touch_pads[1].pin, gpio_get_level(touch_pads[1].pin), touch_pads[2].pin, gpio_get_level(touch_pads[2].pin), touch_pads[3].pin, gpio_get_level(touch_pads[3].pin));
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTaskCreate(&touch_pad_task, "touch_pad_task", 8192, NULL, 10, NULL);
}
