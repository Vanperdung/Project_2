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

#define HEART_PROFILE_NUM                       1
#define HEART_PROFILE_APP_IDX                   0
#define ESP_HEART_RATE_APP_ID                   0x55

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const char *TAG = "MAIN";
static const char *DEVICE_TYPE = "switch 4";
static const char *PROFILE_1_TAG = "HEART_PROFILE";
static const uint8_t service_uuid[16] = {0x10, 0x83, 0xda, 0x90, 0xd7, 0xd8, 0x49, 0x64, 0x84, 0xfd, 0x51, 0x7b, 0xd9, 0x1d, 0x0d, 0x6d};

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
}

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

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_NONCONN_IND,
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

// Service
static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST_A = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEST_B = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TEST_C = 0xFF03;

// Full Database Descriptor - Used to add attributes into the database


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch(event)
    {
        case 
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if(event == ESP_GATTS_REG_EVT)
    {
        ESP_LOGI(TAG, "ESP_GATTS_REG_EVT");
        if(param->reg.status = ESP_GATT_OK)
        {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if;
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
            if(heart_rate_profile_tab[idx].gatts_cb) 
            {
                heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(PROFILE_1_TAG, "ESP_GATTS_REG_EVT");
            esp_err_t ret = esp_ble_gap_config_adv_data(&heart_rate_adv_config);
            if(ret)
            {
                ESP_LOGE(PROFILE_1_TAG, "Config adv data failed, error code %x", ret);
            }
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if(ret)
            {
                ESP_LOGE(PROFILE_1_TAG, "Config scan response data failed, error code %x", ret);
            }
            ret = esp_ble_gatts_create_attr_tab()
            break;
        case 
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
    uint8_t device_name[50] = {0};
    esp_efuse_mac_get_default(mac_hex);
    sprintf((char*)device_name, "%s:%02x%02x%02x%02x%02x%02x", DEVICE_TYPE, mac_hex[0], mac_hex[1], mac_hex[2], mac_hex[3], mac_hex[4], mac_hex[5]);
    ESP_LOGI(TAG, "Device name: %s", device_name);
    ret = esp_ble_gap_set_device_name((char*)device_name);
    if(ret)
    {
        ESP_LOGE(TAG, "Set device name failed, error code %x", ret);
    }
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
    ret = esp_ble_gap_register(gap_event_handler);
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
    // ret = esp_ble_gap_config_adv_data(&heart_rate_adv_config);
    // if(ret)
    // {
    //     ESP_LOGE(TAG, "Gap config error, error code %x", ret);
    // }
}
