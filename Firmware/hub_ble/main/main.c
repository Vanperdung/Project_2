
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
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

#include "freertos/FreeRTOS.h"

#include "cJSON.h"

#define SCAN_DURATION 30
#define MAX_DEVICES 50
#define ADDR_LEN 20

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0 
#define PROFILE_B_APP_ID 1

#define PROFILE_A_APP_INDEX PROFILE_A_APP_ID
#define PROFILE_B_APP_INDEX PROFILE_B_APP_ID

static const char *TAG = "MAIN";
static const char *PROFILE_A_TAG = "PROFILE_A";

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

typedef struct 
{
    char type_node[10];
    int node_id;
} cmd_object_t;

static char peer_addr[MAX_DEVICES][ADDR_LEN];
static int peer_addr = 0;

static void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst node_profile_tab[NODE_PROFILE_NUM] = 
{
    [PROFILE_A_APP_INDEX] = 
    {
        .gattc_cb = gattc_profile_a_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

    [PROFILE_B_APP_INDEX] = 
    {
        .gattc_cb = gattc_profile_b_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

esp_err_t cJson_process(char *data, cmd_object_t *cmd_object)
{
    cJSON *root = cJSON_Parse(data);
    if(root == NULL)
        return ESP_FAIL;
    cJSON *cur_element = NULL;
    cJSON_ArrayForEach(cur_element, root)
    {
        if(cur_element->string)
        {
            char *cur_string = cur_element->string;
            if(strcmp(cur_string, "type_node") == 0)
                memcpy(cmd_object->type_node, cur_element->valuestring, strlen(cur_element->valuestring));
            else if(strcmp(cur_string, "node_id") == 0)
                cmd_object->node_id = cur_element->valueint;
        }
    }
    cJSON_Delete(root);
    return ESP_OK;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    uint8_t adv_node_data[32] = {0};
    uint8_t rsp_node_data[32] = {0};
    cmd_object_t *cmd_object;
    esp_err_t ret;
    switch(event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ret = esp_ble_gap_start_scanning(SCAN_DURATION);
            if(ret)
                ESP_LOGE(TAG, "Start scanning failed, error code %x", ret);
            break;
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if(param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
                ESP_LOGE(TAG, "Scan start failed, error code %x", param->scan_start_cmpl.status);
            else
                ESP_LOGI(TAG, "Scan start success");
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            esp_ble_gap_cb_param_t *scan_result = param;
            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    adv_name = esp_ble_resolve_adv_data(scan_result->ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                    if(scan_result->scan_rst.adv_data_len > 0)
                    {
                        memcpy(adv_node_data, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
                        ESP_LOGI(TAG, "ADV DATA: %s", adv_node_data);
                    }
                    if(scan_result->scan_rst.scan_rsp_len > 0)
                    {
                        memcpy(rsp_node_data, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
                        ESP_LOGI(TAG, "RSP DATA: %s", rsp_node_data);
                    }
                    printf("\r\n");
                    int adv_node_len = strlen(adv_node_data);
                    if(adv_node_data[0] == '{' && adv_node_data[adv_node_len - 1] == '}')
                    {
                        if(cJson_process((char*)adv_node_data, cmd_object) == ESP_OK)
                        {
                            if(strstr(cmd_object->type_node, "switch") != NULL)
                            {
                                ESP_LOGI(TAG, "Searched node_type: %s, node_id: %d", cmd_object->type_node, cmd_object->node_id);
                                esp_ble_gap_stop_scanning();
                                esp_ble_gattc_open(node_profile_tab[PROFILE_A_APP_INDEX].gattc_if, scan_result->scant_rst.cda, scan_result->scan_rst.ble_addr_type, true);
                            }
                        }
                    }
                    break;
                case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                    ESP_LOGI(TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT");
                    break;
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
            if(param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Scan stop failed, error code %x", param->scan_stop_cmpl.status);
                break;
            }
            ESP_LOGI(GATTC_TAG, "Stop scan successfully");
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
            ESP_LOGI(TAG, "Update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                    param->update_conn_params.status,
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
    switch(event)
    {
        case ESP_GATTC_REG_EVT:
            ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_REG_EVT");
            esp_err_t ret = esp_ble_gap_set_scan_params(&ble_scan_params);
            if(ret)
                ESP_LOGE(PROFILE_A_TAG, "Set scan params error, error code = %x", ret);
            break;
        // After opening the connection, an ESP_GATTC_CONNECT_EVT event is triggered
        case ESP_GATTC_CONNECT_EVT:
            ESP_LOGI(PROFILE_A_TAG, "ESP_GATTC_CONNECT_EVT conn_id: %d, if: %d", p_data->connect.conn_id, gattc_if);
            node_profile_tab[PROFILE_A_APP_INDEX].conn_id = p_data->connect.conn_id;
            memcpy(node_profile_tab[PROFILE_A_APP_INDEX].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            ESP_LOGI(PROFILE_A_TAG, "REMOTE BDA:");
            esp_log_buffer_hex(PROFILE_A_TAG, node_profile_tab[PROFILE_A_APP_INDEX].remote_bda, sizeof(esp_bd_addr_t));
            esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
            if(mtu_ret)
                ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
            break;

        default:
            break;
    }
}

static void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{

}

static void gatts_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if(event == ESP_GATTC_REG_EVT)
    {
        if(param->reg.status == ESP_GATTC_OK)
        {
            ESP_LOGI(TAG, "ESP_GATTC_REG_EVT");
            node_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGE(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return
        }
    }

    for(int index = 0; index < PROFILE_NUM; index++)
    {
        if(gattc_if == ESP_GATT_IF_NODE || gattc_if == node_profile_tab[index].gattc_if)
        {
            if(node_profile_tab[index].gattc_cb)
            {
                node_profile_tab[index].gattc_cb(event, gattc_if, param);
            }
        }
    }
}

void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    uint8_t mac_hex[6] = {0};
    esp_efuse_mac_get_default(mac_hex);
    sprintf((char*)mac_addr, "%02x%02x%02x%02x%02x%02x", mac_hex[0], mac_hex[1], mac_hex[2], mac_hex[3], mac_hex[4], mac_hex[5]);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if(ret)
    {
        ESP_LOGE(TAG, "Init controller failed");
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if(ret)
    {
        ESP_LOGE(TAG, "Enable controller failed");
        return;
    }
    ESP_LOGI(TAG, "Init controller successed");
    ret = esp_bluedroid_init();
    if(ret)
    {
        ESP_LOGE(TAG, "Init BLE failed");
        return;
    }
    ret = esp_bluedroid_enable();
    if(ret)
    {
        ESP_LOGE(TAG, "Enable BLE failed");
        return;
    }
    ret = esp_ble_gattc_register_callback(gatts_event_handler);
    if(ret)
    {
        ESP_LOGE(TAG, "Gatts register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if(ret)
    {
        ESP_LOGE(TAG, "Gap register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(ESP_HEART_RATE_APP_ID);
    if(ret)
    {
        ESP_LOGE(TAG, "Gatts app register error, error code %x", ret);
        return;
    }
    ret = esp_ble_gatt_set_local_mtu(500);
    if(ret)
    {
        ESP_LOGE(TAG, "Set local MTU failed, error code %x", ret);
    }
}
