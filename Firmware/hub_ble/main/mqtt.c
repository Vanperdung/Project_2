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

static const char *TAG = "MQTT";
RingbufHandle_t mqtt_ring_buf;
esp_mqtt_client_handle_t client;
extern const char *VERSION;
extern gattc_profile_inst node_profile_tab[PROFILE_NUM];
extern ble_object_t node_object[7];

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        char version[20] = {0};
        ESP_LOGI(TAG, "MQTT event connected");
        esp_mqtt_client_subscribe(client, topic_messages_control, 0);
        esp_mqtt_client_subscribe(client, topic_messages_update, 0);
        esp_mqtt_client_subscribe(client, topic_messages_status, 0);
        sprintf(version, "{\"version\":\"%s\"}", VERSION);
        esp_mqtt_client_publish(client, topic_commands_version, version, strlen(version), 0, 0);
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
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
    mqtt_object_t mqtt_object;
    while (1)
    {
        mess_recv = (char *)xRingbufferReceive(mqtt_ring_buf, &mess_size, portMAX_DELAY);
        if (mess_recv)
        {
            mess_recv[mess_size] = '\0';
            ESP_LOGI(TAG, "Recv payload: %s", mess_recv);
            memset(&mqtt_object, 0, sizeof(mqtt_object));
            mqtt_cJSON_process(mess_recv, &mqtt_object);
            if (strcmp(mqtt_object.action, "control") == 0)
            {
                int index = 0;
                for (index = 0; index < PROFILE_NUM; index++)
                {
                    if (strcmp(mqtt_object.node_id, node_object[index].mac_addr) == 0)
                        break;
                }
                if (index < PROFILE_NUM)
                {

                    esp_ble_gattc_write_char(node_profile_tab[index].gattc_if,
                                             node_profile_tab[index].conn_id,
                                               node_profile_tab[index].char_handle,
                                             strlen(mqtt_object.control),
                                             (uint8_t *)mqtt_object.control,
                                             ESP_GATT_WRITE_TYPE_NO_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
                }

            }
            else if (strcmp(mqtt_object.action, "update") == 0)
            {
                // xTaskCreate(&fota_task, "fota_task", 8192, mqtt_object.url, 10, NULL);
            }
            else if (strcmp(mqtt_object.action, "start_scan") == 0)
            {
                start_scanning(mqtt_object.duration);
            }
            else if (strcmp(mqtt_object.action, "disconnect") == 0)
            {
            }
            else if (strcmp(mqtt_object.action, "disconnect_all") == 0)
            {
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
    mqtt_ring_buf = xRingbufferCreate(2048, RINGBUF_TYPE_NOSPLIT);
    if (mqtt_ring_buf == NULL)
        ESP_LOGE(TAG, "Failed to create ring buffer");
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = (char *)broker,
        .keepalive = 60};
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    xTaskCreate(&mqtt_task, "mqtt_task", 8192, NULL, 9, NULL);
}
