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
#include "esp_spiffs.h"

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

#define REMOTE_SERVICE_UUID 0x00FF
#define REMOTE_NOTIFY_CHAR_UUID 0xFF01

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
extern unsigned char node_list; 

static const char *PROFILE_A_TAG = "PROFILE_A";
static const char *PROFILE_B_TAG = "PROFILE_B";
static const char *PROFILE_C_TAG = "PROFILE_C";
static const char *PROFILE_D_TAG = "PROFILE_D";
static const char *PROFILE_E_TAG = "PROFILE_E";
static const char *PROFILE_F_TAG = "PROFILE_F";
static const char *PROFILE_G_TAG = "PROFILE_G";
static const char *TAG = "BLE_HUB";

static bool get_service_a = false;
static bool get_service_b = false;
static bool get_service_c = false;
static bool get_service_d = false;
static bool get_service_e = false;
static bool get_service_f = false;
static bool get_service_g = false;

static bool connect_a = false;
static bool connect_b = false;
static bool connect_c = false;
static bool connect_d = false;
static bool connect_e = false;
static bool connect_f = false;
static bool connect_g = false;

extern esp_mqtt_client_handle_t client;
extern status_t status;

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = REMOTE_SERVICE_UUID,
    },
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = REMOTE_NOTIFY_CHAR_UUID,
    },
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

static void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_c_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_d_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_e_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_f_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_g_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
gattc_profile_inst node_profile_tab[PROFILE_NUM] =
    {
        [PROFILE_A_APP_INDEX] =
            {
                .gattc_cb = gattc_profile_a_event_handler,
                .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
            },

        [PROFILE_B_APP_INDEX] =
            {
                .gattc_cb = gattc_profile_b_event_handler,
                .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
            },

        [PROFILE_C_APP_INDEX] =
            {
                .gattc_cb = gattc_profile_c_event_handler,
                .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
            },

        [PROFILE_D_APP_INDEX] =
            {
                .gattc_cb = gattc_profile_d_event_handler,
                .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
            },

        [PROFILE_E_APP_INDEX] =
            {
                .gattc_cb = gattc_profile_e_event_handler,
                .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
            },

        [PROFILE_F_APP_INDEX] =
            {
                .gattc_cb = gattc_profile_f_event_handler,
                .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
            },

        [PROFILE_G_APP_INDEX] =
            {
                .gattc_cb = gattc_profile_g_event_handler,
                .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
            },
};

ble_object_t node_object[7];

void ble_send_node(esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle, uint8_t *value)
{
    esp_ble_gattc_write_char(gattc_if, conn_id, handle, strlen((char *)value), value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
}

void start_scanning(int duration)
{
    ESP_LOGI(TAG, "Duration: %d", duration);
    esp_err_t ret = esp_ble_gap_start_scanning((uint32_t)duration);
    if (ret)
        ESP_LOGE(TAG, "Start scanning failed, error code %x", ret);
}

bool check_list(char *mac_addr)
{
    for (int i = 0; i < PROFILE_NUM; i++)
    {
        if(strcmp(node_object[i].mac_addr, mac_addr) == 0)
        {
            return false;
        }
    }
    return true;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    ble_object_t ble_object;
    // esp_err_t ret;
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
        // esp_err_t ret = esp_ble_gap_start_scanning(30);
        // if (ret)
        //     ESP_LOGE(TAG, "Start scanning failed, error code %x", ret);
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            ESP_LOGE(TAG, "Scan start failed, error code %x", param->scan_start_cmpl.status);
        else
            ESP_LOGI(TAG, "Scan start success");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (adv_name != NULL)
            {
                if (adv_name[0] == '$' && adv_name[adv_name_len - 1] == '*')
                {
                    // ESP_LOGI(TAG, "ADV DATA: %s", adv_name);
                    // ESP_LOGI(TAG, "RSP DATA: %s", rsp_node_data);
                    memset((void *)&ble_object, '\0', sizeof(ble_object_t));
                    sscanf((const char *)adv_name, "$,%[^,],%[^,],%d,*", ble_object.node_type, ble_object.mac_addr, &ble_object.total_switch);
                    char mac_buf[6] = {0};
                    strcpy(mac_buf, ble_object.mac_addr);
                    sprintf(ble_object.mac_addr, "%02x%02x%02x%02x%02x%02x", mac_buf[0], mac_buf[1], mac_buf[2], mac_buf[3], mac_buf[4], mac_buf[5]);
                    if (strstr(ble_object.node_type, "switch") != NULL && strlen(ble_object.mac_addr) > 0 && ble_object.total_switch > 0 && check_list(ble_object.mac_addr))
                    {
                        // ESP_LOGI(TAG, "Node type: %s, MAC: %s, Total switch: %d", ble_object.node_type, ble_object.mac_addr, &ble_object.total_switch);
                        if (connect_a == false)
                        {
                            connect_a = true;
                            ESP_LOGI(TAG, "Searched node_type: %s, MAC: %s, total_switch: %d", ble_object.node_type, ble_object.mac_addr, ble_object.total_switch);
                            ESP_LOGI(TAG, "Connect in slot A");
                            printf("\r\n");
                            esp_ble_gap_stop_scanning();
                            strcpy(node_object[PROFILE_A_APP_INDEX].node_type, ble_object.node_type);
                            strcpy(node_object[PROFILE_A_APP_INDEX].mac_addr, ble_object.mac_addr);
                            node_object[PROFILE_A_APP_INDEX].total_switch = ble_object.total_switch;
                            esp_ble_gattc_open(node_profile_tab[PROFILE_A_APP_INDEX].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                        else if (connect_b == false)
                        {
                            connect_b = true;
                            ESP_LOGI(TAG, "Searched node_type: %s, MAC: %s, total_switch: %d", ble_object.node_type, ble_object.mac_addr, ble_object.total_switch);
                            ESP_LOGI(TAG, "Connect in slot B");
                            printf("\r\n");
                            esp_ble_gap_stop_scanning();
                            strcpy(node_object[PROFILE_B_APP_INDEX].node_type, ble_object.node_type);
                            strcpy(node_object[PROFILE_B_APP_INDEX].mac_addr, ble_object.mac_addr);
                            node_object[PROFILE_B_APP_INDEX].total_switch = ble_object.total_switch;
                            esp_ble_gattc_open(node_profile_tab[PROFILE_B_APP_INDEX].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                        else if (connect_c == false)
                        {
                            connect_c = true;
                            ESP_LOGI(TAG, "Searched node_type: %s, MAC: %s, total_switch: %d", ble_object.node_type, ble_object.mac_addr, ble_object.total_switch);
                            ESP_LOGI(TAG, "Connect in slot C");
                            printf("\r\n");
                            esp_ble_gap_stop_scanning();
                            strcpy(node_object[PROFILE_C_APP_INDEX].node_type, ble_object.node_type);
                            strcpy(node_object[PROFILE_C_APP_INDEX].mac_addr, ble_object.mac_addr);
                            node_object[PROFILE_C_APP_INDEX].total_switch = ble_object.total_switch;
                            esp_ble_gattc_open(node_profile_tab[PROFILE_C_APP_INDEX].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                        else if (connect_d == false)
                        {
                            connect_d = true;
                            ESP_LOGI(TAG, "Searched node_type: %s, MAC: %s, total_switch: %d", ble_object.node_type, ble_object.mac_addr, ble_object.total_switch);
                            ESP_LOGI(TAG, "Connect in slot D");
                            printf("\r\n");
                            esp_ble_gap_stop_scanning();
                            strcpy(node_object[PROFILE_D_APP_INDEX].node_type, ble_object.node_type);
                            strcpy(node_object[PROFILE_D_APP_INDEX].mac_addr, ble_object.mac_addr);
                            node_object[PROFILE_D_APP_INDEX].total_switch = ble_object.total_switch;
                            esp_ble_gattc_open(node_profile_tab[PROFILE_D_APP_INDEX].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                        else if (connect_e == false)
                        {
                            connect_e = true;
                            ESP_LOGI(TAG, "Searched node_type: %s, MAC: %s, total_switch: %d", ble_object.node_type, ble_object.mac_addr, ble_object.total_switch);
                            ESP_LOGI(TAG, "Connect in slot E");
                            printf("\r\n");
                            esp_ble_gap_stop_scanning();
                            strcpy(node_object[PROFILE_E_APP_INDEX].node_type, ble_object.node_type);
                            strcpy(node_object[PROFILE_E_APP_INDEX].mac_addr, ble_object.mac_addr);
                            node_object[PROFILE_E_APP_INDEX].total_switch = ble_object.total_switch;
                            esp_ble_gattc_open(node_profile_tab[PROFILE_E_APP_INDEX].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                        else if (connect_f == false)
                        {
                            connect_f = true;
                            ESP_LOGI(TAG, "Searched node_type: %s, MAC: %s, total_switch: %d", ble_object.node_type, ble_object.mac_addr, ble_object.total_switch);
                            ESP_LOGI(TAG, "Connect in slot F");
                            printf("\r\n");
                            esp_ble_gap_stop_scanning();
                            strcpy(node_object[PROFILE_F_APP_INDEX].node_type, ble_object.node_type);
                            strcpy(node_object[PROFILE_F_APP_INDEX].mac_addr, ble_object.mac_addr);
                            node_object[PROFILE_F_APP_INDEX].total_switch = ble_object.total_switch;
                            esp_ble_gattc_open(node_profile_tab[PROFILE_F_APP_INDEX].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                        else if (connect_g == false)
                        {
                            connect_g = true;
                            ESP_LOGI(TAG, "Searched node_type: %s, MAC: %s, total_switch: %d", ble_object.node_type, ble_object.mac_addr, ble_object.total_switch);
                            ESP_LOGI(TAG, "Connect in slot G");
                            printf("\r\n");
                            esp_ble_gap_stop_scanning();
                            strcpy(node_object[PROFILE_G_APP_INDEX].node_type, ble_object.node_type);
                            strcpy(node_object[PROFILE_G_APP_INDEX].mac_addr, ble_object.mac_addr);
                            node_object[PROFILE_G_APP_INDEX].total_switch = ble_object.total_switch;
                            esp_ble_gattc_open(node_profile_tab[PROFILE_G_APP_INDEX].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT");
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Scan stop failed, error code %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Stop scan successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
        ESP_LOGI(TAG, "Update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d", param->update_conn_params.status,
                                                                                                                                    param->update_conn_params.min_int,
                                                                                                                                    param->update_conn_params.max_int,
                                                                                                                                    param->update_conn_params.conn_int,
                                                                                                                                    param->update_conn_params.latency,
                                                                                                                                    param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = param;
    esp_err_t ret;
    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_REG_EVT");
        ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (ret)
            ESP_LOGE(PROFILE_A_TAG, "Set scan params error, error code %x", ret);
        break;
    // After opening the connection, an ESP_GATTC_CONNECT_EVT event is triggered
    case ESP_GATTC_CONNECT_EVT:
        // ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_CONNECT_EVT conn_id: %d, if: %d", p_data->connect.conn_id, gattc_if);
        break;
    // The connection opening also triggers an ESP_GATTC_OPEN_EVT, which is used to check that the opening of the connection was done successfully, otherwise print an error and exit.
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_A_TAG, "Open failed, status %d", p_data->open.status);
            connect_a = false;
            break;
        }
        ESP_LOGI(PROFILE_A_TAG, "Open success");
        node_profile_tab[PROFILE_A_APP_INDEX].conn_id = p_data->open.conn_id;
        memcpy(node_profile_tab[PROFILE_A_APP_INDEX].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_A_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(PROFILE_A_TAG, node_profile_tab[PROFILE_A_APP_INDEX].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
        if (mtu_ret)
            ESP_LOGE(PROFILE_A_TAG, "Config MTU error, error code %x", mtu_ret);
        break;
    // When the MTU is exchanged, an ESP_GATTC_CFG_MTU_EVT is triggered, which in this example is used to print the new MTU size.
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_A_TAG, "config mtu failed, error status %x", param->cfg_mtu.status);
            break;
        }
        ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_A_TAG, "Discover service failed, status %x", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT, status: %d, MTU: %d, conn_id: %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    // The resulting service found, if there is any, will be returned from an ESP_GATTC_SEARCH_RES_EVT. For each service found, the event is triggered to print information about the service discovered, depending on the size of the UUID
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        ESP_LOGI(PROFILE_A_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(PROFILE_A_TAG, "Start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(PROFILE_A_TAG, "Service found");
            get_service_a = true;
            node_profile_tab[PROFILE_A_APP_INDEX].service_start_handle = p_data->search_res.start_handle;
            node_profile_tab[PROFILE_A_APP_INDEX].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(PROFILE_A_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    // After all service results are returned, the search is completed and an ESP_GATTC_SEARCH_CMPL_EVT event is triggered.
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (param->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_A_TAG, "Search service failed, error status %x", param->search_cmpl.status);
            break;
        }
        if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
            ESP_LOGI(PROFILE_A_TAG, "Get service information from remote device");
        else if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
            ESP_LOGI(PROFILE_A_TAG, "Get service information from flash");
        else
            ESP_LOGI(PROFILE_A_TAG, "Unknown service source");
        if (get_service_a)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    node_profile_tab[PROFILE_A_APP_INDEX].service_start_handle,
                                                                    node_profile_tab[PROFILE_A_APP_INDEX].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_A_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_A_TAG, "Characteristic count: %d", (int)count);
                esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                    ESP_LOGE(PROFILE_A_TAG, "Gattc no mem");
                else
                {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            node_profile_tab[PROFILE_A_APP_INDEX].service_start_handle,
                                                            node_profile_tab[PROFILE_A_APP_INDEX].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_A_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    /*  Every service have only one char in our demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        node_profile_tab[PROFILE_A_APP_INDEX].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, node_profile_tab[PROFILE_A_APP_INDEX].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                free(char_elem_result);
            }
            else
                ESP_LOGE(PROFILE_A_TAG, "No char found");
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
            ESP_LOGE(PROFILE_A_TAG, "Reg for notify failed: error status = %d", p_data->reg_for_notify.status);
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        node_profile_tab[PROFILE_A_APP_INDEX].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        node_profile_tab[PROFILE_A_APP_INDEX].service_start_handle,
                                                                        node_profile_tab[PROFILE_A_APP_INDEX].service_end_handle,
                                                                        node_profile_tab[PROFILE_A_APP_INDEX].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_A_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_A_TAG, "Attribute count: %d", (int)count);
                esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                    ESP_LOGE(PROFILE_A_TAG, "Malloc error, gattc no mem");
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        node_profile_tab[PROFILE_A_APP_INDEX].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_A_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    /* Every char has only one descriptor in our demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    node_profile_tab[PROFILE_A_APP_INDEX].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                        if (ret_status != ESP_GATT_OK)
                            ESP_LOGE(PROFILE_A_TAG, "esp_ble_gattc_write_char_descr error");
                    }
                }
                free(descr_elem_result);
            }
            else
                ESP_LOGE(PROFILE_A_TAG, "descr not found");
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
            ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        else
            ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        esp_log_buffer_hex(PROFILE_A_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_A_TAG, "Write descr failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_A_TAG, "Write descr success");
        char node_status[200] = {0};
        sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"join\",\"total_switch\":%d}", node_object[PROFILE_A_APP_INDEX].mac_addr, node_object[PROFILE_A_APP_INDEX].node_type, node_object[PROFILE_A_APP_INDEX].total_switch);
        if (status == NORMAL_MODE)
            esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
        break;
    }
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(PROFILE_A_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_A_TAG, "Write char failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_A_TAG, "Write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    {
        if (memcmp(p_data->disconnect.remote_bda, node_profile_tab[PROFILE_A_APP_INDEX].remote_bda, 6) == 0)
        {
            connect_a = false;
            get_service_a = false;
            ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            char node_status[200] = {0};
            sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"leave\"}", node_object[PROFILE_A_APP_INDEX].mac_addr, node_object[PROFILE_A_APP_INDEX].node_type);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
            memset((void *)&node_object[PROFILE_A_APP_INDEX], '\0', sizeof(ble_object_t));
        }
        break;
    }
    default:
        break;
    }
}

static void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = param;
    // esp_err_t ret;
    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_REG_EVT");
        break;
    // After opening the connection, an ESP_GATTC_CONNECT_EVT event is triggered
    case ESP_GATTC_CONNECT_EVT:
        // ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_CONNECT_EVT conn_id: %d, if: %d", p_data->connect.conn_id, gattc_if);
        break;
    // The connection opening also triggers an ESP_GATTC_OPEN_EVT, which is used to check that the opening of the connection was done successfully, otherwise print an error and exit.
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_B_TAG, "Open failed, status %d", p_data->open.status);
            connect_b = false;
            break;
        }
        ESP_LOGI(PROFILE_B_TAG, "Open success");
        node_profile_tab[PROFILE_B_APP_INDEX].conn_id = p_data->open.conn_id;
        memcpy(node_profile_tab[PROFILE_B_APP_INDEX].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_B_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(PROFILE_B_TAG, node_profile_tab[PROFILE_B_APP_INDEX].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
        if (mtu_ret)
            ESP_LOGE(PROFILE_B_TAG, "Config MTU error, error code %x", mtu_ret);
        break;
    // When the MTU is exchanged, an ESP_GATTC_CFG_MTU_EVT is triggered, which in this example is used to print the new MTU size.
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_B_TAG, "config mtu failed, error status %x", param->cfg_mtu.status);
            break;
        }
        ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_B_TAG, "Discover service failed, status %x", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT, status: %d, MTU: %d, conn_id: %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    // The resulting service found, if there is any, will be returned from an ESP_GATTC_SEARCH_RES_EVT. For each service found, the event is triggered to print information about the service discovered, depending on the size of the UUID
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        ESP_LOGI(PROFILE_B_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(PROFILE_B_TAG, "Start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(PROFILE_B_TAG, "Service found");
            get_service_b = true;
            node_profile_tab[PROFILE_B_APP_INDEX].service_start_handle = p_data->search_res.start_handle;
            node_profile_tab[PROFILE_B_APP_INDEX].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(PROFILE_B_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    // After all service results are returned, the search is completed and an ESP_GATTC_SEARCH_CMPL_EVT event is triggered.
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (param->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_B_TAG, "Search service failed, error status %x", param->search_cmpl.status);
            break;
        }
        if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
            ESP_LOGI(PROFILE_B_TAG, "Get service information from remote device");
        else if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
            ESP_LOGI(PROFILE_B_TAG, "Get service information from flash");
        else
            ESP_LOGI(PROFILE_B_TAG, "Unknown service source");
        if (get_service_b)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    node_profile_tab[PROFILE_B_APP_INDEX].service_start_handle,
                                                                    node_profile_tab[PROFILE_B_APP_INDEX].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_B_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_B_TAG, "Characteristic count: %d", (int)count);
                esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                    ESP_LOGE(PROFILE_B_TAG, "Gattc no mem");
                else
                {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            node_profile_tab[PROFILE_B_APP_INDEX].service_start_handle,
                                                            node_profile_tab[PROFILE_B_APP_INDEX].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_B_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    /*  Every service have only one char in our demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        node_profile_tab[PROFILE_B_APP_INDEX].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, node_profile_tab[PROFILE_B_APP_INDEX].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                free(char_elem_result);
            }
            else
                ESP_LOGE(PROFILE_B_TAG, "No char found");
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
            ESP_LOGE(PROFILE_B_TAG, "Reg for notify failed: error status = %d", p_data->reg_for_notify.status);
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        node_profile_tab[PROFILE_B_APP_INDEX].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        node_profile_tab[PROFILE_B_APP_INDEX].service_start_handle,
                                                                        node_profile_tab[PROFILE_B_APP_INDEX].service_end_handle,
                                                                        node_profile_tab[PROFILE_B_APP_INDEX].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_B_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_B_TAG, "Attribute count: %d", (int)count);
                esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                    ESP_LOGE(PROFILE_B_TAG, "Malloc error, gattc no mem");
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        node_profile_tab[PROFILE_B_APP_INDEX].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_B_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    /* Every char has only one descriptor in our demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    node_profile_tab[PROFILE_B_APP_INDEX].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                        if (ret_status != ESP_GATT_OK)
                            ESP_LOGE(PROFILE_B_TAG, "esp_ble_gattc_write_char_descr error");
                    }
                }
                free(descr_elem_result);
            }
            else
                ESP_LOGE(PROFILE_B_TAG, "descr not found");
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
            ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        else
            ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        esp_log_buffer_hex(PROFILE_B_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_B_TAG, "Write descr failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_B_TAG, "Write descr success");
        char node_status[200] = {0};
        sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"join\",\"total_switch\":%d}", node_object[PROFILE_B_APP_INDEX].mac_addr, node_object[PROFILE_B_APP_INDEX].node_type, node_object[PROFILE_B_APP_INDEX].total_switch);
        if (status == NORMAL_MODE)
            esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
        break;
    }
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(PROFILE_B_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_B_TAG, "Write char failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_B_TAG, "Write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    {
        if (memcmp(p_data->disconnect.remote_bda, node_profile_tab[PROFILE_B_APP_INDEX].remote_bda, 6) == 0)
        {
            connect_b = false;
            get_service_b = false;
            ESP_LOGI(PROFILE_B_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            char node_status[200] = {0};
            sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"leave\"}", node_object[PROFILE_B_APP_INDEX].mac_addr, node_object[PROFILE_B_APP_INDEX].node_type);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
            memset((void *)&node_object[PROFILE_B_APP_INDEX], '\0', sizeof(ble_object_t));
        }
        break;
    }
    default:
        break;
    }
}

static void gattc_profile_c_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = param;
    // esp_err_t ret;
    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_REG_EVT");
        break;
    // After opening the connection, an ESP_GATTC_CONNECT_EVT event is triggered
    case ESP_GATTC_CONNECT_EVT:
        // ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_CONNECT_EVT conn_id: %d, if: %d", p_data->connect.conn_id, gattc_if);
        break;
    // The connection opening also triggers an ESP_GATTC_OPEN_EVT, which is used to check that the opening of the connection was done successfully, otherwise print an error and exit.
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_C_TAG, "Open failed, status %d", p_data->open.status);
            connect_c = false;
            break;
        }
        ESP_LOGI(PROFILE_C_TAG, "Open success");
        node_profile_tab[PROFILE_C_APP_INDEX].conn_id = p_data->open.conn_id;
        memcpy(node_profile_tab[PROFILE_C_APP_INDEX].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_C_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(PROFILE_C_TAG, node_profile_tab[PROFILE_C_APP_INDEX].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
        if (mtu_ret)
            ESP_LOGE(PROFILE_C_TAG, "Config MTU error, error code %x", mtu_ret);
        break;
    // When the MTU is exchanged, an ESP_GATTC_CFG_MTU_EVT is triggered, which in this example is used to print the new MTU size.
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_C_TAG, "config mtu failed, error status %x", param->cfg_mtu.status);
            break;
        }
        ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_C_TAG, "Discover service failed, status %x", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT, status: %d, MTU: %d, conn_id: %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    // The resulting service found, if there is any, will be returned from an ESP_GATTC_SEARCH_RES_EVT. For each service found, the event is triggered to print information about the service discovered, depending on the size of the UUID
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        ESP_LOGI(PROFILE_C_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(PROFILE_C_TAG, "Start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(PROFILE_C_TAG, "Service found");
            get_service_c = true;
            node_profile_tab[PROFILE_C_APP_INDEX].service_start_handle = p_data->search_res.start_handle;
            node_profile_tab[PROFILE_C_APP_INDEX].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(PROFILE_C_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    // After all service results are returned, the search is completed and an ESP_GATTC_SEARCH_CMPL_EVT event is triggered.
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (param->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_C_TAG, "Search service failed, error status %x", param->search_cmpl.status);
            break;
        }
        if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
            ESP_LOGI(PROFILE_C_TAG, "Get service information from remote device");
        else if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
            ESP_LOGI(PROFILE_C_TAG, "Get service information from flash");
        else
            ESP_LOGI(PROFILE_C_TAG, "Unknown service source");
        if (get_service_c)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    node_profile_tab[PROFILE_C_APP_INDEX].service_start_handle,
                                                                    node_profile_tab[PROFILE_C_APP_INDEX].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_C_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_C_TAG, "Characteristic count: %d", (int)count);
                esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                    ESP_LOGE(PROFILE_C_TAG, "Gattc no mem");
                else
                {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            node_profile_tab[PROFILE_C_APP_INDEX].service_start_handle,
                                                            node_profile_tab[PROFILE_C_APP_INDEX].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_C_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    /*  Every service have only one char in our demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        node_profile_tab[PROFILE_C_APP_INDEX].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, node_profile_tab[PROFILE_C_APP_INDEX].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                free(char_elem_result);
            }
            else
                ESP_LOGE(PROFILE_C_TAG, "No char found");
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
            ESP_LOGE(PROFILE_C_TAG, "Reg for notify failed: error status = %d", p_data->reg_for_notify.status);
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        node_profile_tab[PROFILE_C_APP_INDEX].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        node_profile_tab[PROFILE_C_APP_INDEX].service_start_handle,
                                                                        node_profile_tab[PROFILE_C_APP_INDEX].service_end_handle,
                                                                        node_profile_tab[PROFILE_C_APP_INDEX].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_C_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_C_TAG, "Attribute count: %d", (int)count);
                esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                    ESP_LOGE(PROFILE_C_TAG, "Malloc error, gattc no mem");
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        node_profile_tab[PROFILE_C_APP_INDEX].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_C_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    /* Every char has only one descriptor in our demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    node_profile_tab[PROFILE_C_APP_INDEX].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                        if (ret_status != ESP_GATT_OK)
                            ESP_LOGE(PROFILE_C_TAG, "esp_ble_gattc_write_char_descr error");
                    }
                }
                free(descr_elem_result);
            }
            else
                ESP_LOGE(PROFILE_C_TAG, "descr not found");
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
            ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        else
            ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        esp_log_buffer_hex(PROFILE_C_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_C_TAG, "Write descr failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_C_TAG, "Write descr success");
        char node_status[200] = {0};
        sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"join\",\"total_switch\":%d}", node_object[PROFILE_C_APP_INDEX].mac_addr, node_object[PROFILE_C_APP_INDEX].node_type, node_object[PROFILE_C_APP_INDEX].total_switch);
        if (status == NORMAL_MODE)
            esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
        break;
    }
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(PROFILE_C_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_C_TAG, "Write char failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_C_TAG, "Write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    {
        if (memcmp(p_data->disconnect.remote_bda, node_profile_tab[PROFILE_C_APP_INDEX].remote_bda, 6) == 0)
        {
            connect_c = false;
            get_service_c = false;
            ESP_LOGI(PROFILE_C_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            char node_status[200] = {0};
            sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"leave\"}", node_object[PROFILE_C_APP_INDEX].mac_addr, node_object[PROFILE_C_APP_INDEX].node_type);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
            memset((void *)&node_object[PROFILE_C_APP_INDEX], '\0', sizeof(ble_object_t));
        }
        break;
    }
    default:
        break;
    }
}

static void gattc_profile_d_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = param;
    // esp_err_t ret;
    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_REG_EVT");
        break;
    // After opening the connection, an ESP_GATTC_CONNECT_EVT event is triggered
    case ESP_GATTC_CONNECT_EVT:
        // ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_CONNECT_EVT conn_id: %d, if: %d", p_data->connect.conn_id, gattc_if);
        break;
    // The connection opening also triggers an ESP_GATTC_OPEN_EVT, which is used to check that the opening of the connection was done successfully, otherwise print an error and exit.
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_D_TAG, "Open failed, status %d", p_data->open.status);
            connect_d = false;
            break;
        }
        ESP_LOGI(PROFILE_D_TAG, "Open success");
        node_profile_tab[PROFILE_D_APP_INDEX].conn_id = p_data->open.conn_id;
        memcpy(node_profile_tab[PROFILE_D_APP_INDEX].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_D_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(PROFILE_D_TAG, node_profile_tab[PROFILE_D_APP_INDEX].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
        if (mtu_ret)
            ESP_LOGE(PROFILE_D_TAG, "Config MTU error, error code %x", mtu_ret);
        break;
    // When the MTU is exchanged, an ESP_GATTC_CFG_MTU_EVT is triggered, which in this example is used to print the new MTU size.
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_D_TAG, "config mtu failed, error status %x", param->cfg_mtu.status);
            break;
        }
        ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_D_TAG, "Discover service failed, status %x", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT, status: %d, MTU: %d, conn_id: %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    // The resulting service found, if there is any, will be returned from an ESP_GATTC_SEARCH_RES_EVT. For each service found, the event is triggered to print information about the service discovered, depending on the size of the UUID
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        ESP_LOGI(PROFILE_D_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(PROFILE_D_TAG, "Start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(PROFILE_D_TAG, "Service found");
            get_service_d = true;
            node_profile_tab[PROFILE_D_APP_INDEX].service_start_handle = p_data->search_res.start_handle;
            node_profile_tab[PROFILE_D_APP_INDEX].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(PROFILE_D_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    // After all service results are returned, the search is completed and an ESP_GATTC_SEARCH_CMPL_EVT event is triggered.
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (param->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_D_TAG, "Search service failed, error status %x", param->search_cmpl.status);
            break;
        }
        if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
            ESP_LOGI(PROFILE_D_TAG, "Get service information from remote device");
        else if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
            ESP_LOGI(PROFILE_D_TAG, "Get service information from flash");
        else
            ESP_LOGI(PROFILE_D_TAG, "Unknown service source");
        if (get_service_d)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    node_profile_tab[PROFILE_D_APP_INDEX].service_start_handle,
                                                                    node_profile_tab[PROFILE_D_APP_INDEX].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_D_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_D_TAG, "Characteristic count: %d", (int)count);
                esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                    ESP_LOGE(PROFILE_D_TAG, "Gattc no mem");
                else
                {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            node_profile_tab[PROFILE_D_APP_INDEX].service_start_handle,
                                                            node_profile_tab[PROFILE_D_APP_INDEX].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_D_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    /*  Every service have only one char in our demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        node_profile_tab[PROFILE_D_APP_INDEX].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, node_profile_tab[PROFILE_D_APP_INDEX].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                free(char_elem_result);
            }
            else
                ESP_LOGE(PROFILE_D_TAG, "No char found");
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
            ESP_LOGE(PROFILE_D_TAG, "Reg for notify failed: error status = %d", p_data->reg_for_notify.status);
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        node_profile_tab[PROFILE_D_APP_INDEX].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        node_profile_tab[PROFILE_D_APP_INDEX].service_start_handle,
                                                                        node_profile_tab[PROFILE_D_APP_INDEX].service_end_handle,
                                                                        node_profile_tab[PROFILE_D_APP_INDEX].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_D_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_D_TAG, "Attribute count: %d", (int)count);
                esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                    ESP_LOGE(PROFILE_D_TAG, "Malloc error, gattc no mem");
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        node_profile_tab[PROFILE_D_APP_INDEX].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_D_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    /* Every char has only one descriptor in our demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    node_profile_tab[PROFILE_D_APP_INDEX].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                        if (ret_status != ESP_GATT_OK)
                            ESP_LOGE(PROFILE_D_TAG, "esp_ble_gattc_write_char_descr error");
                    }
                }
                free(descr_elem_result);
            }
            else
                ESP_LOGE(PROFILE_D_TAG, "descr not found");
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
            ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        else
            ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        esp_log_buffer_hex(PROFILE_D_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_D_TAG, "Write descr failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_D_TAG, "Write descr success");
        char node_status[200] = {0};
        sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"join\",\"total_switch\":%d}", node_object[PROFILE_D_APP_INDEX].mac_addr, node_object[PROFILE_D_APP_INDEX].node_type, node_object[PROFILE_D_APP_INDEX].total_switch);
        if (status == NORMAL_MODE)
            esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
        break;
    }
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(PROFILE_D_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_D_TAG, "Write char failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_D_TAG, "Write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    {
        if (memcmp(p_data->disconnect.remote_bda, node_profile_tab[PROFILE_D_APP_INDEX].remote_bda, 6) == 0)
        {
            connect_d = false;
            get_service_d = false;
            ESP_LOGI(PROFILE_D_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            char node_status[200] = {0};
            sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"leave\"}", node_object[PROFILE_D_APP_INDEX].mac_addr, node_object[PROFILE_D_APP_INDEX].node_type);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
            memset((void *)&node_object[PROFILE_D_APP_INDEX], '\0', sizeof(ble_object_t));
        }
        break;
    }
    default:
        break;
    }
}

static void gattc_profile_e_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = param;
    // esp_err_t ret;
    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_REG_EVT");
        break;
    // After opening the connection, an ESP_GATTC_CONNECT_EVT event is triggered
    case ESP_GATTC_CONNECT_EVT:
        // ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_CONNECT_EVT conn_id: %d, if: %d", p_data->connect.conn_id, gattc_if);
        break;
    // The connection opening also triggers an ESP_GATTC_OPEN_EVT, which is used to check that the opening of the connection was done successfully, otherwise print an error and exit.
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_E_TAG, "Open failed, status %d", p_data->open.status);
            connect_e = false;
            break;
        }
        ESP_LOGI(PROFILE_E_TAG, "Open success");
        node_profile_tab[PROFILE_E_APP_INDEX].conn_id = p_data->open.conn_id;
        memcpy(node_profile_tab[PROFILE_E_APP_INDEX].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_E_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(PROFILE_E_TAG, node_profile_tab[PROFILE_E_APP_INDEX].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
        if (mtu_ret)
            ESP_LOGE(PROFILE_E_TAG, "Config MTU error, error code %x", mtu_ret);
        break;
    // When the MTU is exchanged, an ESP_GATTC_CFG_MTU_EVT is triggered, which in this example is used to print the new MTU size.
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_E_TAG, "config mtu failed, error status %x", param->cfg_mtu.status);
            break;
        }
        ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_E_TAG, "Discover service failed, status %x", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT, status: %d, MTU: %d, conn_id: %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    // The resulting service found, if there is any, will be returned from an ESP_GATTC_SEARCH_RES_EVT. For each service found, the event is triggered to print information about the service discovered, depending on the size of the UUID
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        ESP_LOGI(PROFILE_E_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(PROFILE_E_TAG, "Start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(PROFILE_E_TAG, "Service found");
            get_service_e = true;
            node_profile_tab[PROFILE_E_APP_INDEX].service_start_handle = p_data->search_res.start_handle;
            node_profile_tab[PROFILE_E_APP_INDEX].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(PROFILE_E_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    // After all service results are returned, the search is completed and an ESP_GATTC_SEARCH_CMPL_EVT event is triggered.
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (param->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_E_TAG, "Search service failed, error status %x", param->search_cmpl.status);
            break;
        }
        if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
            ESP_LOGI(PROFILE_E_TAG, "Get service information from remote device");
        else if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
            ESP_LOGI(PROFILE_E_TAG, "Get service information from flash");
        else
            ESP_LOGI(PROFILE_E_TAG, "Unknown service source");
        if (get_service_e)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    node_profile_tab[PROFILE_E_APP_INDEX].service_start_handle,
                                                                    node_profile_tab[PROFILE_E_APP_INDEX].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_E_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_E_TAG, "Characteristic count: %d", (int)count);
                esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                    ESP_LOGE(PROFILE_E_TAG, "Gattc no mem");
                else
                {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            node_profile_tab[PROFILE_E_APP_INDEX].service_start_handle,
                                                            node_profile_tab[PROFILE_E_APP_INDEX].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_E_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    /*  Every service have only one char in our demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        node_profile_tab[PROFILE_E_APP_INDEX].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, node_profile_tab[PROFILE_E_APP_INDEX].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                free(char_elem_result);
            }
            else
                ESP_LOGE(PROFILE_E_TAG, "No char found");
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
            ESP_LOGE(PROFILE_E_TAG, "Reg for notify failed: error status = %d", p_data->reg_for_notify.status);
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].service_start_handle,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].service_end_handle,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_E_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_E_TAG, "Attribute count: %d", (int)count);
                esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                    ESP_LOGE(PROFILE_E_TAG, "Malloc error, gattc no mem");
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_E_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    /* Every char has only one descriptor in our demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    node_profile_tab[PROFILE_E_APP_INDEX].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                        if (ret_status != ESP_GATT_OK)
                            ESP_LOGE(PROFILE_E_TAG, "esp_ble_gattc_write_char_descr error");
                    }
                }
                free(descr_elem_result);
            }
            else
                ESP_LOGE(PROFILE_E_TAG, "descr not found");
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
            ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        else
            ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        esp_log_buffer_hex(PROFILE_E_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_E_TAG, "Write descr failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_E_TAG, "Write descr success");
        char node_status[200] = {0};
        sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"join\",\"total_switch\":%d}", node_object[PROFILE_E_APP_INDEX].mac_addr, node_object[PROFILE_E_APP_INDEX].node_type, node_object[PROFILE_E_APP_INDEX].total_switch);
        if (status == NORMAL_MODE)
            esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
        break;
    }
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(PROFILE_E_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_E_TAG, "Write char failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_E_TAG, "Write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    {
        if (memcmp(p_data->disconnect.remote_bda, node_profile_tab[PROFILE_E_APP_INDEX].remote_bda, 6) == 0)
        {
            connect_e = false;
            get_service_e = false;
            ESP_LOGI(PROFILE_E_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            char node_status[200] = {0};
            sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"leave\"}", node_object[PROFILE_E_APP_INDEX].mac_addr, node_object[PROFILE_E_APP_INDEX].node_type);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
            memset((void *)&node_object[PROFILE_E_APP_INDEX], '\0', sizeof(ble_object_t));
        }
        break;
    }
    default:
        break;
    }
}

static void gattc_profile_f_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = param;
    // esp_err_t ret;
    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_REG_EVT");
        break;
    // After opening the connection, an ESP_GATTC_CONNECT_EVT event is triggered
    case ESP_GATTC_CONNECT_EVT:
        // ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_CONNECT_EVT conn_id: %d, if: %d", p_data->connect.conn_id, gattc_if);
        break;
    // The connection opening also triggers an ESP_GATTC_OPEN_EVT, which is used to check that the opening of the connection was done successfully, otherwise print an error and exit.
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_F_TAG, "Open failed, status %d", p_data->open.status);
            connect_f = false;
            break;
        }
        ESP_LOGI(PROFILE_F_TAG, "Open success");
        node_profile_tab[PROFILE_E_APP_INDEX].conn_id = p_data->open.conn_id;
        memcpy(node_profile_tab[PROFILE_E_APP_INDEX].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_F_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(PROFILE_F_TAG, node_profile_tab[PROFILE_E_APP_INDEX].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
        if (mtu_ret)
            ESP_LOGE(PROFILE_F_TAG, "Config MTU error, error code %x", mtu_ret);
        break;
    // When the MTU is exchanged, an ESP_GATTC_CFG_MTU_EVT is triggered, which in this example is used to print the new MTU size.
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_F_TAG, "config mtu failed, error status %x", param->cfg_mtu.status);
            break;
        }
        ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_F_TAG, "Discover service failed, status %x", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT, status: %d, MTU: %d, conn_id: %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    // The resulting service found, if there is any, will be returned from an ESP_GATTC_SEARCH_RES_EVT. For each service found, the event is triggered to print information about the service discovered, depending on the size of the UUID
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        ESP_LOGI(PROFILE_F_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(PROFILE_F_TAG, "Start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(PROFILE_F_TAG, "Service found");
            get_service_f = true;
            node_profile_tab[PROFILE_E_APP_INDEX].service_start_handle = p_data->search_res.start_handle;
            node_profile_tab[PROFILE_E_APP_INDEX].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(PROFILE_F_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    // After all service results are returned, the search is completed and an ESP_GATTC_SEARCH_CMPL_EVT event is triggered.
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (param->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_F_TAG, "Search service failed, error status %x", param->search_cmpl.status);
            break;
        }
        if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
            ESP_LOGI(PROFILE_F_TAG, "Get service information from remote device");
        else if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
            ESP_LOGI(PROFILE_F_TAG, "Get service information from flash");
        else
            ESP_LOGI(PROFILE_F_TAG, "Unknown service source");
        if (get_service_f)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    node_profile_tab[PROFILE_E_APP_INDEX].service_start_handle,
                                                                    node_profile_tab[PROFILE_E_APP_INDEX].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_F_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_F_TAG, "Characteristic count: %d", (int)count);
                esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                    ESP_LOGE(PROFILE_F_TAG, "Gattc no mem");
                else
                {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            node_profile_tab[PROFILE_E_APP_INDEX].service_start_handle,
                                                            node_profile_tab[PROFILE_E_APP_INDEX].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_F_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    /*  Every service have only one char in our demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        node_profile_tab[PROFILE_E_APP_INDEX].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, node_profile_tab[PROFILE_E_APP_INDEX].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                free(char_elem_result);
            }
            else
                ESP_LOGE(PROFILE_F_TAG, "No char found");
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
            ESP_LOGE(PROFILE_F_TAG, "Reg for notify failed: error status = %d", p_data->reg_for_notify.status);
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].service_start_handle,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].service_end_handle,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_F_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_F_TAG, "Attribute count: %d", (int)count);
                esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                    ESP_LOGE(PROFILE_F_TAG, "Malloc error, gattc no mem");
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        node_profile_tab[PROFILE_E_APP_INDEX].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_F_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    /* Every char has only one descriptor in our demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    node_profile_tab[PROFILE_E_APP_INDEX].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                        if (ret_status != ESP_GATT_OK)
                            ESP_LOGE(PROFILE_F_TAG, "esp_ble_gattc_write_char_descr error");
                    }
                }
                free(descr_elem_result);
            }
            else
                ESP_LOGE(PROFILE_F_TAG, "descr not found");
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
            ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        else
            ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        esp_log_buffer_hex(PROFILE_F_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_F_TAG, "Write descr failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_F_TAG, "Write descr success");
        char node_status[200] = {0};
        sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"join\",\"total_switch\":%d}", node_object[PROFILE_E_APP_INDEX].mac_addr, node_object[PROFILE_E_APP_INDEX].node_type, node_object[PROFILE_E_APP_INDEX].total_switch);
        if (status == NORMAL_MODE)
            esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
        break;
    }
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(PROFILE_F_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_F_TAG, "Write char failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_F_TAG, "Write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    {
        if (memcmp(p_data->disconnect.remote_bda, node_profile_tab[PROFILE_E_APP_INDEX].remote_bda, 6) == 0)
        {
            connect_f = false;
            get_service_f = false;
            ESP_LOGI(PROFILE_F_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            char node_status[200] = {0};
            sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"leave\"}", node_object[PROFILE_E_APP_INDEX].mac_addr, node_object[PROFILE_E_APP_INDEX].node_type);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
            memset((void *)&node_object[PROFILE_E_APP_INDEX], '\0', sizeof(ble_object_t));
        }
        break;
    }
    default:
        break;
    }
}

static void gattc_profile_g_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = param;
    // esp_err_t ret;
    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_REG_EVT");
        break;
    // After opening the connection, an ESP_GATTC_CONNECT_EVT event is triggered
    case ESP_GATTC_CONNECT_EVT:
        // ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_CONNECT_EVT conn_id: %d, if: %d", p_data->connect.conn_id, gattc_if);
        break;
    // The connection opening also triggers an ESP_GATTC_OPEN_EVT, which is used to check that the opening of the connection was done successfully, otherwise print an error and exit.
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_G_TAG, "Open failed, status %d", p_data->open.status);
            connect_g = false;
            break;
        }
        ESP_LOGI(PROFILE_G_TAG, "Open success");
        node_profile_tab[PROFILE_G_APP_INDEX].conn_id = p_data->open.conn_id;
        memcpy(node_profile_tab[PROFILE_G_APP_INDEX].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_G_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(PROFILE_G_TAG, node_profile_tab[PROFILE_G_APP_INDEX].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
        if (mtu_ret)
            ESP_LOGE(PROFILE_G_TAG, "Config MTU error, error code %x", mtu_ret);
        break;
    // When the MTU is exchanged, an ESP_GATTC_CFG_MTU_EVT is triggered, which in this example is used to print the new MTU size.
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_G_TAG, "config mtu failed, error status %x", param->cfg_mtu.status);
            break;
        }
        ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_G_TAG, "Discover service failed, status %x", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT, status: %d, MTU: %d, conn_id: %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    // The resulting service found, if there is any, will be returned from an ESP_GATTC_SEARCH_RES_EVT. For each service found, the event is triggered to print information about the service discovered, depending on the size of the UUID
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        ESP_LOGI(PROFILE_G_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(PROFILE_G_TAG, "Start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(PROFILE_G_TAG, "Service found");
            get_service_g = true;
            node_profile_tab[PROFILE_G_APP_INDEX].service_start_handle = p_data->search_res.start_handle;
            node_profile_tab[PROFILE_G_APP_INDEX].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(PROFILE_G_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    // After all service results are returned, the search is completed and an ESP_GATTC_SEARCH_CMPL_EVT event is triggered.
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (param->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_G_TAG, "Search service failed, error status %x", param->search_cmpl.status);
            break;
        }
        if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
            ESP_LOGI(PROFILE_G_TAG, "Get service information from remote device");
        else if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
            ESP_LOGI(PROFILE_G_TAG, "Get service information from flash");
        else
            ESP_LOGI(PROFILE_G_TAG, "Unknown service source");
        if (get_service_g)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    node_profile_tab[PROFILE_G_APP_INDEX].service_start_handle,
                                                                    node_profile_tab[PROFILE_G_APP_INDEX].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_G_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_G_TAG, "Characteristic count: %d", (int)count);
                esp_gattc_char_elem_t *char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                    ESP_LOGE(PROFILE_G_TAG, "Gattc no mem");
                else
                {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            node_profile_tab[PROFILE_G_APP_INDEX].service_start_handle,
                                                            node_profile_tab[PROFILE_G_APP_INDEX].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_G_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    /*  Every service have only one char in our demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        node_profile_tab[PROFILE_G_APP_INDEX].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, node_profile_tab[PROFILE_G_APP_INDEX].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                free(char_elem_result);
            }
            else
                ESP_LOGE(PROFILE_G_TAG, "No char found");
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
            ESP_LOGE(PROFILE_G_TAG, "Reg for notify failed: error status = %d", p_data->reg_for_notify.status);
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        node_profile_tab[PROFILE_G_APP_INDEX].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        node_profile_tab[PROFILE_G_APP_INDEX].service_start_handle,
                                                                        node_profile_tab[PROFILE_G_APP_INDEX].service_end_handle,
                                                                        node_profile_tab[PROFILE_G_APP_INDEX].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
                ESP_LOGE(PROFILE_G_TAG, "esp_ble_gattc_get_attr_count error");
            if (count > 0)
            {
                ESP_LOGI(PROFILE_G_TAG, "Attribute count: %d", (int)count);
                esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                    ESP_LOGE(PROFILE_G_TAG, "Malloc error, gattc no mem");
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        node_profile_tab[PROFILE_G_APP_INDEX].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                        ESP_LOGE(PROFILE_G_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    /* Every char has only one descriptor in our demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    node_profile_tab[PROFILE_G_APP_INDEX].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                        if (ret_status != ESP_GATT_OK)
                            ESP_LOGE(PROFILE_G_TAG, "esp_ble_gattc_write_char_descr error");
                    }
                }
                free(descr_elem_result);
            }
            else
                ESP_LOGE(PROFILE_G_TAG, "descr not found");
        }
        break;
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
            ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        else
            ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        esp_log_buffer_hex(PROFILE_G_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    {
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_G_TAG, "Write descr failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_G_TAG, "Write descr success");
        char node_status[200] = {0};
        sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"join\",\"total_switch\":%d}", node_object[PROFILE_G_APP_INDEX].mac_addr, node_object[PROFILE_G_APP_INDEX].node_type, node_object[PROFILE_G_APP_INDEX].total_switch);
        if (status == NORMAL_MODE)
            esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
        break;
    }
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(PROFILE_G_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(PROFILE_G_TAG, "Write char failed, error status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(PROFILE_G_TAG, "Write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    {
        if (memcmp(p_data->disconnect.remote_bda, node_profile_tab[PROFILE_G_APP_INDEX].remote_bda, 6) == 0)
        {
            connect_g = false;
            get_service_g = false;
            ESP_LOGI(PROFILE_G_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            char node_status[200] = {0};
            sprintf(node_status, "{\"node_id\":\"%s\",\"node_type\":\"%s\",\"status\":\"leave\"}", node_object[PROFILE_G_APP_INDEX].mac_addr, node_object[PROFILE_G_APP_INDEX].node_type);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, node_status, strlen(node_status), 0, 0);
            memset((void *)&node_object[PROFILE_G_APP_INDEX], '\0', sizeof(ble_object_t));
        }
        break;
    }
    default:
        break;
    }
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            ESP_LOGI(TAG, "ESP_GATTC_REG_EVT");
            node_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGE(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    for (int index = 0; index < PROFILE_NUM; index++)
    {
        if (gattc_if == ESP_GATT_IF_NONE || gattc_if == node_profile_tab[index].gattc_if)
        {
            if (node_profile_tab[index].gattc_cb)
            {
                node_profile_tab[index].gattc_cb(event, gattc_if, param);
            }
        }
    }
}

void ble_init(void)
{
    esp_err_t ret;
    FILE *node_list_file = fopen(NODE_LIST_FILE, "r");
    if(node_list_file = NULL)
        ESP_LOGE(TAG, "Cannot open file %s", NODE_LIST_FILE);
    if(fread(&node_list, 1, 1, node_list_file) == 0)
        ESP_LOGE(TAG, "No data yet");
    else
        
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "Init controller failed");
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "Enable controller failed");
        return;
    }
    ESP_LOGI(TAG, "Init controller successed");
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "Init BLE failed");
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "Enable BLE failed");
        return;
    }
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "Gatts register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "Gap register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "Gattc app register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_B_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "Gattc app register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_C_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "Gattc app register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_D_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "Gattc app register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_E_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "Gattc app register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_F_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "Gattc app register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_G_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "Gattc app register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret)
    {
        ESP_LOGE(TAG, "Set local MTU failed, error code %x", ret);
    }
}