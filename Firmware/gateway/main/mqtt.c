/**
 * @file mqtt.c
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

#include "driver/gpio.h"

#include "cJSON.h"
#include "mqtt.h"
#include "wifi_sta.h"
#include "button.h"
#include "ble_mesh_user.h"
#include "smartconfig.h"
#include "common.h"

static const char *TAG = "MQTT";
RingbufHandle_t mqtt_ring_buf;
esp_mqtt_client_handle_t client;
extern char version[10];
extern char topic_commands_set[50];
extern char topic_commands_get[50];
extern char topic_commands_status[50];
extern char topic_commands_heartbeat[50];
extern char topic_commands_network[50];
extern char topic_commands_process[50];
extern char topic_commands_version[50];
extern char topic_commands_fota[50];
extern status_t status;
extern node_info_t nodes[MAXIMUM_NODE];
extern esp_ble_mesh_client_t config_client;
extern esp_ble_mesh_client_t onoff_client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        char ver_json[50] = {0};
        status = NORMAL_MODE;
        ESP_LOGI(TAG, "MQTT event connected");
        esp_mqtt_client_subscribe(client, topic_commands_set, 0);
        esp_mqtt_client_subscribe(client, topic_commands_fota, 0);
        esp_mqtt_client_subscribe(client, topic_commands_get, 0);
        sprintf(ver_json, "{\"action\":\"version\",\"firm_ver\":\"%s\"}", version);
        esp_mqtt_client_publish(client, topic_commands_version, ver_json, strlen(ver_json), 0, 0);
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        status = LOCAL_MODE;
        ESP_LOGI(TAG, "MQTT event disconnected");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event subcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT event unsubcribed, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT event published, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
    {
        UBaseType_t res = xRingbufferSend(mqtt_ring_buf, event->data, event->data_len, portMAX_DELAY);
        if (res != pdTRUE)
            ESP_LOGE(TAG, "Failed to send item\n");
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT event error");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_task(void *param)
{
    char *mess_recv = NULL;
    size_t mess_size = 0;
    mqtt_obj_t mqtt_obj;
    esp_ble_mesh_generic_client_set_state_t set_state;
    esp_ble_mesh_client_common_param_t common;
    while (1)
    {
        mess_recv = (char *)xRingbufferReceive(mqtt_ring_buf, &mess_size, portMAX_DELAY);
        if (mess_recv)
        {
            mess_recv[mess_size] = '\0';
            ESP_LOGI(TAG, "Recv payload: %s", mess_recv);
            memset(&mqtt_obj, 0, sizeof(mqtt_obj));
            mqtt_parse_data(mess_recv, &mqtt_obj);
            if (strcmp(mqtt_obj.action, "set") == 0)
            {
                ble_mesh_set_msg_common(&common, (uint16_t)mqtt_obj.unicast_addr, onoff_client.model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET);
                set_state.onoff_set.op_en = false;
                set_state.onoff_set.onoff = (uint8_t)mqtt_obj.state;
                set_state.onoff_set.tid = 0;
                esp_err_t err = esp_ble_mesh_generic_client_set_state(&common, &set_state);
                if (err)
                {
                    ESP_LOGE(TAG, "%s: Generic OnOff Set failed", __func__);
                }
            }
            else if (strcmp(mqtt_obj.action, "get") == 0)
            {
            }
            else if (strcmp(mqtt_obj.action, "upgrade") == 0)
            {
                // xTaskCreate(&fota_task, "fota_task", 8192, mqtt_obj.url, 10, NULL);
            }
            vRingbufferReturnItem(mqtt_ring_buf, (void *)mess_recv);
        }
    }
}

void mqtt_client_sta(void)
{
    uint8_t broker[50] = {0};
    ESP_LOGI(TAG, "MQTT init");
    ESP_LOGI(TAG, "Broker: %s", MQTT_BROKER);
    sprintf((char *)broker, "mqtt://%s", MQTT_BROKER);
    mqtt_ring_buf = xRingbufferCreate(4096, RINGBUF_TYPE_NOSPLIT);
    if (mqtt_ring_buf == NULL)
        ESP_LOGE(TAG, "Failed to create ring buffer");
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = (char *)broker,
        .keepalive = 60,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    xTaskCreate(&mqtt_task, "mqtt_task", 8192, NULL, 9, NULL);
}
