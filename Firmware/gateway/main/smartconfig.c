/**
 * @file smartconfig.c
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
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "mqtt_client.h"
#include "esp_attr.h"

#include "driver/gpio.h"

#include "cJSON.h"
#include "mqtt.h"
#include "wifi_sta.h"
#include "button.h"
#include "ble_mesh_user.h"
#include "smartconfig.h"
#include "common.h"

#define CONNECTED_BIT       BIT0
#define ESPTOUCH_DONE_BIT   BIT1

static const char *TAG = "SMARTCONFIG";
static EventGroupHandle_t smartconfig_event_group;
extern status_red_t status_red;
extern status_blue_t status_blue;

static void smartconfig_task(void *param);

static void smartconfig_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        xTaskCreate(&smartconfig_task, "smartconfig_task", 4096, NULL, 10, NULL);
    } 
    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        esp_wifi_connect();
        xEventGroupClearBits(smartconfig_event_group, CONNECTED_BIT);
    } 
    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        xEventGroupSetBits(smartconfig_event_group, CONNECTED_BIT);
    } 
    else if(event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) 
    {
        ESP_LOGI(TAG, "Scan done");
    } 
    else if(event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) 
    {
        ESP_LOGI(TAG, "Found channel");
    } 
    else if(event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) 
    {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = {0};
        uint8_t password[65] = {0};
        uint8_t rvd_data[33] = {0};
        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if(wifi_config.sta.bssid_set == true) 
        {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        if(evt->type == SC_TYPE_ESPTOUCH_V2) 
        {
			ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
			ESP_LOGI(TAG, "RVD_DATA:");
			for (int i=0; i<33; i++) 
            {
				printf("%02x ", rvd_data[i]);
			}
			printf("\n");
		}

        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        esp_wifi_connect();
    } 
    else if(event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) 
    {
        xEventGroupSetBits(smartconfig_event_group, ESPTOUCH_DONE_BIT);
    }

}

static void smartconfig_task(void *param)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while(1) 
    {
        uxBits = xEventGroupWaitBits(smartconfig_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
        if(uxBits & CONNECTED_BIT) 
        {
            ESP_LOGI(TAG, "WiFi connected to AP");
            // status = NORMAL_MODE;
        }
        if(uxBits & ESPTOUCH_DONE_BIT) 
        {
            ESP_LOGI(TAG, "Smartconfig over");
            esp_smartconfig_stop();
            esp_restart();
            vTaskDelete(NULL);
        }
    }
}

void smartconfig_init(void)
{
    smartconfig_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &smartconfig_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &smartconfig_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &smartconfig_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}