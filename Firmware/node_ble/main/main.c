#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
// #include "bta_api.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"

#define HEART_PROFILE_NUM                       1
#define HEART_PROFILE_APP_IDX                   0
#define ESP_HEART_RATE_APP_ID                   0x55
#define ADV_CONFIG_FLAG                         BIT0
#define SCAN_RSP_CONFIG_FLAG                    BIT1
#define GATTS_DEMO_CHAR_VAL_LEN_MAX             500
#define PREPARE_BUF_MAX_SIZE                    1024

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const char *TAG = "MAIN";
static const char *NAME = "project_2";
static const char *DEVICE_TYPE = "switch";
static const int NODE_ID = 4;
static const char *PROFILE_1_TAG = "SWITCH";
static uint8_t service_uuid[16] = {0x10, 0x83, 0xda, 0x90, 0xd7, 0xd8, 0x49, 0x64, 0x84, 0xfd, 0x51, 0x7b, 0xd9, 0x1d, 0x0d, 0x6d};
static uint8_t adv_config_done = 0;
static uint8_t mac_hex[6] = {0};
enum 
{
    HEART_RATE_INDEX_SERVICE = 0,

    HEART_RATE_MEAS_INDEX_CHAR,
    HEART_RATE_MEAS_INDEX_VAL,
    HEART_RATE_MEAS_NOTI_INDEX_CFG,

    SENSOR_LCT_INDEX_CHAR,
    SENSOR_LCT_INDEX_VAL,

    HEART_RATE_CTL_INDEX_CHAR,
    CONTROL_POINT_INDEX_VAL,

    HEART_RATE_TOTAL
};

static uint16_t heart_rate_handle_table[HEART_RATE_TOTAL];

typedef struct
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} gatts_profile_inst;

static gatts_profile_inst heart_rate_profile_tab[HEART_PROFILE_NUM] = {
    [HEART_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

static esp_ble_adv_params_t heart_rate_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

static esp_ble_adv_data_t heart_rate_adv_config = {
    .include_name = true,
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010, 
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL, 
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = ESP_BLE_ADV_FLAG_LIMIT_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT
};

static esp_ble_adv_data_t ble_rsp_data = {
    .include_name = true,
    .set_scan_rsp = true,
    .include_txpower = true,
    .min_interval = 0x0006, 
    .max_interval = 0x0010, 
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL, 
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = ESP_BLE_ADV_FLAG_LIMIT_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT
};

// Service + Characteristic UUID test
static const uint16_t GATTS_SERVICE_UUID_TEST       = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST_A        = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEST_B        = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TEST_C        = 0xFF03;

static const uint16_t primary_service_uuid          = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid    = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid  = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                 = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write                = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify    = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]       = {0x00, 0x00};
static const uint8_t char_value[4]                  = {0x11, 0x22, 0x33, 0x44};

// Full Database Descriptor - Used to add attributes into the database
static const esp_gatts_attr_db_t gatt_db[HEART_RATE_TOTAL] = 
{
    // Service Declaration
    [HEART_RATE_INDEX_SERVICE] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t*)&GATTS_SERVICE_UUID_TEST}},
    // Characteristic Declaration
    [HEART_RATE_MEAS_INDEX_CHAR] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t*)&char_prop_read_write_notify}},
    // Characteristic Value
    [HEART_RATE_MEAS_INDEX_VAL] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t*)char_value}},
    // Client Characteristic Configuration Descriptor
    [HEART_RATE_MEAS_NOTI_INDEX_CFG] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t*)heart_measurement_ccc}},
    // Characteristic Declaration
    [SENSOR_LCT_INDEX_CHAR] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t*)&char_prop_read}},
    // Chacracteristic Value
    [SENSOR_LCT_INDEX_VAL] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t*)char_value}},
    // Characteristic Declaration
    [HEART_RATE_CTL_INDEX_CHAR] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), (uint8_t*)&char_prop_write}},
    // Characteristic Value
    [CONTROL_POINT_INDEX_VAL] = 
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t*)char_value}},
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if(adv_config_done == 0)
            {
                ESP_LOGI(TAG, "Start advertising");
                esp_ble_gap_start_advertising(&heart_rate_adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT");
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if(adv_config_done == 0)
            {
                ESP_LOGI(TAG, "Start advertising");
                esp_ble_gap_start_advertising(&heart_rate_adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if(param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            else
            {
                ESP_LOGI(TAG, "Advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if(param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Advertising stop failed");
            }
            else
            {
                ESP_LOGI(TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d, conn_int = %d, latency = %d, timeout = %d",
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

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // When an Application Profile is registered, an ESP_GATTS_REG_EVT event is triggered(by calling esp_ble_gatts_app_register function)
    if(event == ESP_GATTS_REG_EVT)
    {
        ESP_LOGI(TAG, "ESP_GATTS_REG_EVT");
        if(param->reg.status == ESP_GATT_OK)
        {
            heart_rate_profile_tab[HEART_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    int index = 0;
    for(index = 0; index < HEART_PROFILE_NUM; index++)
    {
        /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
        if(gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[index].gatts_if)
        {
            if(heart_rate_profile_tab[index].gatts_cb) 
            {
                heart_rate_profile_tab[index].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch(event)
    {
        esp_err_t ret;
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_REG_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            uint8_t type_node[30] = {0};
            uint8_t node_id[30] = {0};
            sprintf((char*)type_node, "{\"type_node\":\"%s\"}", DEVICE_TYPE);
            sprintf((char*)node_id, "{\"mac\":\"%s\",\"node_id\":%d}", mac_hex, NODE_ID);
            // ESP_LOGI(PROFILE_1_TAG, "type_node: %s, node_id: %s", type_node, node_id);
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(type_node, strlen((char*)type_node));
            if(raw_adv_ret)
            {
                ESP_LOGE(PROFILE_1_TAG, "Config raw adv data failed, error code = %x", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(node_id, strlen((char*)node_id));
            if(raw_scan_ret)
            {
                ESP_LOGE(PROFILE_1_TAG, "Config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HEART_RATE_TOTAL, HEART_RATE_INDEX_SERVICE);
            if(ret)
            {
                ESP_LOGE(PROFILE_1_TAG, "Create attr table failed, error code %x", ret);
            }
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT");
            if(param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(PROFILE_1_TAG, "Create attribute table failed, error code %x", param->add_attr_tab.status);
            }
            else if(param->add_attr_tab.num_handle != HEART_RATE_TOTAL)
            {
                ESP_LOGE(PROFILE_1_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HEART_RATE_TOTAL);
            }
            else
            {
                ESP_LOGI(PROFILE_1_TAG, "Create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
                memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
                esp_ble_gatts_start_service(heart_rate_handle_table[HEART_RATE_INDEX_SERVICE]);
            }
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_CONF_EVT, status %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(PROFILE_1_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_READ_EVT:
        {
            ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            esp_gatt_rsp_t rsp;
            char data_read[10] = {"hehe lolo"};
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = strlen(data_read);
            strcpy((char*)rsp.attr_value.value, data_read);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT:
            if(!param->write.is_prep)
            {
                // the data length of gattc write must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_WRITE_EVT, handle = %d, value len = %d, value:", param->write.handle, param->write.len);
                esp_log_buffer_hex(PROFILE_1_TAG, param->write.value, param->write.len);
                if(heart_rate_handle_table[HEART_RATE_MEAS_NOTI_INDEX_CFG] == param->write.handle && param->write.len == 2)
                {
                    uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                    if(descr_value == 0x0001)
                    {
                        ESP_LOGI(PROFILE_1_TAG, "Notify enable");
                        uint8_t notify_data[15];
                        for(int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i;
                        }
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[HEART_RATE_MEAS_NOTI_INDEX_CFG],
                                                sizeof(notify_data), notify_data, false);
                    }
                    else if(descr_value == 0x0000)
                        ESP_LOGI(PROFILE_1_TAG, "Notify disable");
                    else
                    {
                        ESP_LOGE(PROFILE_1_TAG, "Unknown descr value");
                        esp_log_buffer_hex(PROFILE_1_TAG, param->write.value, param->write.len);
                    }
                }
                if(param->write.need_rsp)
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
        default:
            break;
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
    // sprintf((char*)mac_addr, "%02x%02x%02x%02x%02x%02x", mac_hex[0], mac_hex[1], mac_hex[2], mac_hex[3], mac_hex[4], mac_hex[5]);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
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
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
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
    ret = esp_ble_gatts_app_register(ESP_HEART_RATE_APP_ID);
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
