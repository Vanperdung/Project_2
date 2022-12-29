/**
 * @file wifi_sta.c
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
#include "mqtt_client.h"
#include "esp_wifi.h"

#include "driver/gpio.h"

#include "cJSON.h"
#include "mqtt.h"
#include "wifi_sta.h"
#include "button.h"
#include "ble_mesh_user.h"
#include "common.h"
#include "led.h"
#include "button.h"
#include "smartconfig.h"

static const char *TAG = "WIFI";
static EventGroupHandle_t wifi_event_group;
extern status_t status;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
		ESP_LOGI(TAG, "WiFi start connect to AP");
        status = LOCAL_MODE;
		esp_wifi_connect();
	}
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        ESP_LOGI(TAG, "WiFi disconnected");
        status = LOCAL_MODE;
		esp_wifi_connect();
	} 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) 
    {
		ESP_LOGI(TAG, "WiFi connected");
	} 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
	}
}

void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
	esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_cfg);
}

void wifi_sta(wifi_config_t wifi_cfg, wifi_mode_t wifi_mode)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
	ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));
	ESP_ERROR_CHECK(esp_wifi_start());

    // EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
}