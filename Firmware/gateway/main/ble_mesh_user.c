/**
 * @file ble_mesh_user.c
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

#define MODEL_APP_BIND_BIT BIT0
#define MODEL_APP_BIND_REPEAT_BIT BIT1
#define GET_STATE_REPEAT_BIT BIT2
#define GET_STATE_BIT BIT3

static const char *TAG = "BLE MESH USER";
extern uint8_t dev_uuid[16];
extern EventGroupHandle_t mesh_evt_group;
extern TaskHandle_t mesh_evt_handle;
extern QueueHandle_t unprov_dev_queue;

static struct esp_ble_mesh_key
{
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t app_key[16];
} prov_key;

node_info_t nodes[MAXIMUM_NODE] = {
    [0 ... MAXIMUM_NODE - 1] = {
        .unicast_node = ESP_BLE_MESH_ADDR_UNASSIGNED,
        .elem_num = 0,
        .prov = false,
    },
};

static esp_ble_mesh_client_t config_client;
static esp_ble_mesh_client_t onoff_client;
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(NULL, &onoff_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid = dev_uuid,
    .prov_unicast_addr = PROV_OWN_ADDR,
    .prov_start_address = 0x0005,
    .prov_attention = 0x00,
    .prov_algorithm = 0x00,
    .prov_pub_key_oob = 0x00,
    .prov_static_oob_val = NULL,
    .prov_static_oob_len = 0x00,
    .flags = 0x00,
    .iv_index = 0x00,
};

void ble_mesh_get_dev_uuid(uint8_t *dev_uuid)
{
    if (dev_uuid == NULL)
    {
        ESP_LOGE(TAG, "%s, Invalid device uuid", __func__);
        return;
    }

    /* Copy device address to the device uuid with offset equals to 2 here.
     * The first two bytes is used for matching device uuid by Provisioner.
     * And using device address here is to avoid using the same device uuid
     * by different unprovisioned devices.
     */
    memcpy(dev_uuid + 2, esp_bt_dev_get_address(), BD_ADDR_LEN);
}

esp_err_t bluetooth_init(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s initialize controller failed", __func__);
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed", __func__);
        return ret;
    }
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return ret;
    }

    return ret;
}

esp_err_t ble_mesh_store_node_info_in_flash(node_info_t *comp)
{
    return ESP_OK;
}

esp_err_t ble_mesh_restore_node_info_in_flash(node_info_t *comp)
{
    return ESP_OK;
}

esp_err_t ble_mesh_delete_node_info_in_flash(node_info_t *comp)
{
    return ESP_OK;
}

void decode_comp_data(node_info_t *comp, uint8_t *comp_data, int elem_num)
{
    int pos = 0;
    comp->cid = (uint16_t)(comp_data[1] << 8 | comp_data[0]);
    comp->pid = (uint16_t)(comp_data[3] << 8 | comp_data[2]);
    comp->vid = (uint16_t)(comp_data[5] << 8 | comp_data[4]);
    comp->crpl = (uint16_t)(comp_data[7] << 8 | comp_data[6]);
    comp->feature = (uint16_t)(comp_data[9] << 8 | comp_data[8]);
    pos = 10;
    for (int i = 0; i < elem_num; i++)
    {
        comp->elem[i].loc = (uint16_t)(comp_data[pos + 1] << 8 | comp_data[pos]);
        pos += 2;
        comp->elem[i].numS = comp_data[pos++];
        comp->elem[i].numV = comp_data[pos++];
        for (int j = 0; j < comp->elem[i].numS; j++)
        {
            comp->elem[i].sig_models[j].model_id = (uint16_t)(comp_data[pos + 1] << 8 | comp_data[pos]);
            pos += 2;
            if (comp->elem[i].sig_models[j].model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV)
            {
                strcpy((char *)comp->elem[i].sig_models[j].model_name, "onoff_sv");
                comp->elem[i].sig_models[j].onoff_state = OFF;
            }
        }
    }
}

esp_err_t ble_mesh_store_node_info(const uint8_t uuid[16], uint16_t unicast, uint8_t elem_num)
{
    if (!uuid || !ESP_BLE_MESH_ADDR_IS_UNICAST(unicast))
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (!memcmp(nodes[i].uuid, uuid, 16))
        {
            ESP_LOGW(TAG, "%s: reprovisioned device 0x%04x", __func__, unicast);
            nodes[i].prov = false;
            nodes[i].unicast_node = unicast;
            nodes[i].elem_num = elem_num;
            for (int j = 0; j < elem_num; j++)
            {
                nodes[i].elem[j].unicast_elem = unicast + j;
            }
            return ESP_OK;
        }
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (nodes[i].unicast_node == ESP_BLE_MESH_ADDR_UNASSIGNED)
        {
            nodes[i].prov = false;
            memcpy(nodes[i].uuid, uuid, 16);
            nodes[i].unicast_node = unicast;
            nodes[i].elem_num = elem_num;
            for (int j = 0; j < elem_num; j++)
            {
                nodes[i].elem[j].unicast_elem = unicast + j;
            }
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

void ble_mesh_store_composition_info(node_info_t *comp, uint8_t *comp_data, int elem_num)
{
    decode_comp_data(comp, comp_data, elem_num);
}

node_info_t *ble_mesh_get_node_info_with_unicast(uint16_t unicast)
{
    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(unicast))
    {
        ESP_LOGE(TAG, "Error unicast address: 0x%04x", unicast);
        return NULL;
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (nodes[i].unicast_node <= unicast && nodes[i].unicast_node + nodes[i].elem_num > unicast)
        {
            return &nodes[i];
        }
    }
    ESP_LOGE(TAG, "Not found node with unicast address: 0x%04x", unicast);
    return NULL;
}

node_info_t *ble_mesh_get_node_info_with_uuid(uint8_t *uuid)
{
    if (!uuid || strlen((char *)uuid) != 16)
    {
        ESP_LOGE(TAG, "Error uuid");
        return NULL;
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (strstr((char *)nodes[i].uuid, (char *)uuid) != NULL)
        {
            return &nodes[i];
        }
    }
    ESP_LOGE(TAG, "Not found node with uuid: %s", bt_hex(uuid, 16));
    return NULL;
}

elem_info_t *ble_mesh_get_elem_info_with_unicast(uint16_t unicast)
{
    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(unicast))
    {
        ESP_LOGE(TAG, "Error unicast address: 0x%04x", unicast);
        return NULL;
    }

    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (nodes[i].unicast_node <= unicast && nodes[i].unicast_node + nodes[i].elem_num > unicast)
        {
            for (int j = 0; j < ARRAY_SIZE(nodes[i].elem); j++)
            {
                if (nodes[i].elem[j].unicast_elem == unicast)
                {
                    return &nodes[i].elem[j];
                }
            }
        }
    }
    return NULL;
}

model_info_t *ble_mesh_get_model_info_with_model_id(uint16_t unicast, uint16_t model_id)
{
    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(unicast))
    {
        ESP_LOGE(TAG, "Error unicast address: 0x%04x", unicast);
        return NULL;
    }

    elem_info_t *elem = NULL;
    elem = ble_mesh_get_elem_info_with_unicast(unicast);
    for (int i = 0; i < ARRAY_SIZE(elem->sig_models); i++)
    {
        if (elem->sig_models[i].model_id == model_id)
        {
            return &elem->sig_models[i];
        }
    }
    return NULL;
}

esp_err_t ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem,
                                  esp_ble_mesh_model_t *client_model, uint32_t opcode)
{
    if (!common || !client_model)
    {
        return ESP_ERR_INVALID_ARG;
    }

    common->opcode = opcode;
    common->model = client_model;
    common->ctx.net_idx = prov_key.net_idx;
    common->ctx.app_idx = prov_key.app_idx;
    common->ctx.addr = unicast_elem;
    common->ctx.send_ttl = MSG_SEND_TTL;
    common->ctx.send_rel = MSG_SEND_REL;
    common->msg_timeout = MSG_TIMEOUT;
    common->msg_role = MSG_ROLE;

    return ESP_OK;
}

esp_err_t ble_mesh_onoff_get_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model)
{
    esp_ble_mesh_generic_client_get_state_t get_state = {0};
    ble_mesh_set_msg_common(common, unicast_elem, onoff_client_model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET);
    esp_err_t err = esp_ble_mesh_generic_client_get_state(common, &get_state);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Generic OnOff Get failed", __func__);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_onoff_set_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model)
{
    return ESP_OK;
}

esp_err_t ble_mesh_app_bind(esp_ble_mesh_client_common_param_t *common, node_info_t *node, uint16_t unicast_elem, esp_ble_mesh_model_t *model)
{
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    ble_mesh_set_msg_common(common, node->unicast_node, model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
    set_state.model_app_bind.element_addr = unicast_elem;
    set_state.model_app_bind.model_app_idx = prov_key.app_idx;
    set_state.model_app_bind.model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
    set_state.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
    esp_err_t err = esp_ble_mesh_config_client_set_state(common, &set_state);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Config Model App Bind failed", __func__);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t prov_complete(int node_idx, const esp_ble_mesh_octet16_t uuid,
                               uint16_t unicast, uint8_t elem_num, uint16_t net_idx)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};
    node_info_t *node = NULL;
    char name[11] = {0};
    esp_err_t err;

    // ESP_LOGI(TAG, "node index: 0x%x, unicast address: 0x%02x, element num: %d, netkey index: 0x%02x",
    //          node_idx, unicast, elem_num, net_idx);
    // ESP_LOGI(TAG, "device uuid: %s", bt_hex(uuid, 16));

    sprintf(name, "%s%d", "NODE-", node_idx);
    err = esp_ble_mesh_provisioner_set_node_name(node_idx, name);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Set node name failed", __func__);
        return ESP_FAIL;
    }

    err = ble_mesh_store_node_info(uuid, unicast, elem_num);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Store node info failed", __func__);
        return ESP_FAIL;
    }

    node = ble_mesh_get_node_info_with_unicast(unicast);
    if (!node)
    {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "********************** Node Info **********************");
    ESP_LOGW(TAG, "Node index: 0x%02x, Unicast address: 0x%04x, Element num: %d, Netkey index: 0x%02x",
             node_idx, node->unicast_node, node->elem_num, net_idx);
    ESP_LOGW(TAG, "Device UUID: %s", bt_hex(uuid, 16));
    ESP_LOGW(TAG, "*******************************************************");
    ble_mesh_set_msg_common(&common, node->unicast_node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get_state.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Send config comp data get failed", __func__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void prov_link_open(esp_ble_mesh_prov_bearer_t bearer)
{
    ESP_LOGI(TAG, "%s link open", bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
}

static void prov_link_close(esp_ble_mesh_prov_bearer_t bearer, uint8_t reason)
{
    ESP_LOGI(TAG, "%s link close, reason 0x%02x",
             bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", reason);
}

static void recv_unprov_adv_pkt(uint8_t dev_uuid[16], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type, uint16_t oob_info,
                                uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    int err;

    /* Due to the API esp_ble_mesh_provisioner_set_dev_uuid_match, Provisioner will only
     * use this callback to report the devices, whose device UUID starts with 0xdd & 0xdd,
     * to the application layer.
     */

    // ESP_LOGI(TAG, "address: %s, address type: %d, adv type: %d", bt_hex(addr, BD_ADDR_LEN), addr_type, adv_type);
    // ESP_LOGI(TAG, "device uuid: %s", bt_hex(dev_uuid, 16));
    // ESP_LOGI(TAG, "oob info: %d, bearer: %s", oob_info, (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");

    memcpy(add_dev.addr, addr, BD_ADDR_LEN);
    add_dev.addr_type = (uint8_t)addr_type;
    memcpy(add_dev.uuid, dev_uuid, 16);
    add_dev.oob_info = oob_info;
    add_dev.bearer = (uint8_t)bearer;
    /* Note: If unprovisioned device adv packets have not been received, we should not add
             device with ADD_DEV_START_PROV_NOW_FLAG set. */
    err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
                                                  ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG);
    // if (err)
    // {
    //     ESP_LOGE(TAG, "%s: Add unprovisioned device into queue failed", __func__);
    // }

    return;
}

static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                     esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d", param->provisioner_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT, err_code %d", param->provisioner_prov_disable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
        recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid, param->provisioner_recv_unprov_adv_pkt.addr,
                            param->provisioner_recv_unprov_adv_pkt.addr_type, param->provisioner_recv_unprov_adv_pkt.oob_info,
                            param->provisioner_recv_unprov_adv_pkt.adv_type, param->provisioner_recv_unprov_adv_pkt.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        prov_link_open(param->provisioner_prov_link_open.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        prov_link_close(param->provisioner_prov_link_close.bearer, param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
        prov_complete(param->provisioner_prov_complete.node_idx, param->provisioner_prov_complete.device_uuid,
                      param->provisioner_prov_complete.unicast_addr, param->provisioner_prov_complete.element_num,
                      param->provisioner_prov_complete.netkey_idx);
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code %d", param->provisioner_add_unprov_dev_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT, err_code %d", param->provisioner_set_dev_uuid_match_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT:
    {
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT, err_code %d", param->provisioner_set_node_name_comp.err_code);
        if (param->provisioner_set_node_name_comp.err_code == ESP_OK)
        {
            const char *name = NULL;
            name = esp_ble_mesh_provisioner_get_node_name(param->provisioner_set_node_name_comp.node_index);
            if (!name)
            {
                ESP_LOGE(TAG, "Get node name failed");
                return;
            }
            // ESP_LOGI(TAG, "Node %d name is: %s", param->provisioner_set_node_name_comp.node_index, name);
        }
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
    {
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, err_code %d", param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == ESP_OK)
        {
            esp_err_t err = 0;
            prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
            err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx,
                                                                       ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, ESP_BLE_MESH_CID_NVAL);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Provisioner bind local model appkey failed");
                return;
            }
        }
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT, err_code %d", param->provisioner_bind_app_key_to_model_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_ENABLE_HEARTBEAT_RECV_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ENABLE_HEARTBEAT_RECV_COMP_EVT, err_code %d", param->provisioner_enable_heartbeat_recv_comp.err_code);

    default:
        break;
    }

    return;
}

static void ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                      esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_ble_mesh_client_common_param_t common = {0};
    node_info_t *node = NULL;
    uint32_t opcode;
    uint16_t addr;
    int err;

    opcode = param->params->opcode;
    addr = param->params->ctx.addr;

    // ESP_LOGI(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04x",
    //          __func__, param->error_code, event, param->params->ctx.addr, opcode);

    if (param->error_code)
    {
        ESP_LOGE(TAG, "Send config client message failed, opcode 0x%04x", opcode);
        return;
    }

    node = ble_mesh_get_node_info_with_unicast(addr);
    if (!node)
    {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return;
    }

    switch (event)
    {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET");
            ESP_LOGW(TAG, "********************** Composition Data **********************");
            // ESP_LOGI(TAG, "Composition data %s", bt_hex(param->status_cb.comp_data_status.composition_data->data, param->status_cb.comp_data_status.composition_data->len));
            ble_mesh_store_composition_info(node, param->status_cb.comp_data_status.composition_data->data, node->elem_num);
            ESP_LOGW(TAG, "CID: 0x%04x, PID: 0x%04x, VID: 0x%04x, CRPL: 0x%04x, Feature: 0x%04x",
                     node->cid, node->pid, node->vid, node->crpl, node->feature);
            for (uint8_t i = 0; i < node->elem_num; i++)
            {
                ESP_LOGW(TAG, "ELEMENT %u", i);
                ESP_LOGW(TAG, "      Unicast: 0x%04x, Location: 0x%04x, NumS: %u, NumV: %u",
                         node->elem[i].unicast_elem, node->elem[i].loc, node->elem[i].numS, node->elem[i].numV);
                for (uint8_t j = 0; j < node->elem[i].numS; j++)
                {
                    ESP_LOGW(TAG, "            MODEL %u, ID: 0x%04x", j, node->elem[i].sig_models[j].model_id);
                }
            }
            ESP_LOGW(TAG, "**************************************************************");
            esp_ble_mesh_cfg_client_set_state_t set_state = {0};
            ble_mesh_set_msg_common(&common, node->unicast_node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set_state.app_key_add.net_idx = prov_key.net_idx;
            set_state.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set_state.app_key_add.app_key, prov_key.app_key, 16);
            err = esp_ble_mesh_config_client_set_state(&common, &set_state);
            if (err)
            {
                ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
                return;
            }
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            xTaskCreate(&mesh_evt_task, "mesh_evt_task", 4096, (void *)node, 9, &mesh_evt_handle);
            xEventGroupSetBits(mesh_evt_group, MODEL_APP_BIND_BIT);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "Element Unicast Address :0x%04x", param->status_cb.model_app_status.element_addr);
            if (node->unicast_node + (uint16_t)node->elem_num > param->status_cb.model_app_status.element_addr + 1)
            {
                xEventGroupSetBits(mesh_evt_group, MODEL_APP_BIND_BIT);
            }
            else if (node->unicast_node + (uint16_t)node->elem_num == param->status_cb.model_app_status.element_addr + 1)
            {
                xEventGroupSetBits(mesh_evt_group, GET_STATE_BIT);
            }
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS:
            ESP_LOG_BUFFER_HEX("composition data %s", param->status_cb.comp_data_status.composition_data->data,
                               param->status_cb.comp_data_status.composition_data->len);
            break;
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_STATUS:
            break;
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET:
        {
            esp_ble_mesh_cfg_client_get_state_t get_state = {0};
            ble_mesh_set_msg_common(&common, node->unicast_node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
            get_state.comp_data_get.page = COMP_DATA_PAGE_0;
            err = esp_ble_mesh_config_client_get_state(&common, &get_state);
            if (err)
            {
                ESP_LOGE(TAG, "%s: Config Composition Data Get failed", __func__);
                return;
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        {
            esp_ble_mesh_cfg_client_set_state_t set_state = {0};
            ble_mesh_set_msg_common(&common, node->unicast_node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set_state.app_key_add.net_idx = prov_key.net_idx;
            set_state.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set_state.app_key_add.app_key, prov_key.app_key, 16);
            err = esp_ble_mesh_config_client_set_state(&common, &set_state);
            if (err)
            {
                ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
                return;
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        {
            if (node->prov == false)
            {
                xEventGroupSetBits(mesh_evt_group, MODEL_APP_BIND_REPEAT_BIT);
            }
            break;
        }
        default:
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Not a config client status message event");
        break;
    }
}

static void ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                       esp_ble_mesh_generic_client_cb_param_t *param)
{
    node_info_t *node = NULL;
    uint32_t opcode;
    uint16_t addr;

    opcode = param->params->opcode;
    addr = param->params->ctx.addr;

    // ESP_LOGI(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04x",
    //          __func__, param->error_code, event, param->params->ctx.addr, opcode);

    if (param->error_code)
    {
        ESP_LOGE(TAG, "Send generic client message failed, opcode 0x%04x", opcode);
        return;
    }

    node = ble_mesh_get_node_info_with_unicast(addr);
    if (!node)
    {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return;
    }

    switch (event)
    {
    case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        {
            model_info_t *onoff_model = NULL;
            onoff_model = ble_mesh_get_model_info_with_model_id(param->params->ctx.addr, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            if (!onoff_model)
            {
                ESP_LOGE(TAG, "%s: Get model info failed", __func__);
                return;
            }
            onoff_model->onoff_state = param->status_cb.onoff_status.present_onoff;
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET unicast: 0x%04x, onoff: 0x%02x", addr, onoff_model->onoff_state);
            if (node->prov == false && node->unicast_node + (uint16_t)node->elem_num > addr + 1)
            {
                xEventGroupSetBits(mesh_evt_group, GET_STATE_BIT);
            }
            else if (node->prov == false && node->unicast_node + (uint16_t)node->elem_num == addr + 1)
            {
                ESP_LOGW(TAG, "********************** Provision done **********************");
                node->prov = true;
                vTaskDelete(mesh_evt_handle);
            }
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
        {
            model_info_t *onoff_model = NULL;
            onoff_model = ble_mesh_get_model_info_with_model_id(param->params->ctx.addr, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            if (!onoff_model)
            {
                ESP_LOGE(TAG, "%s: Get model info failed", __func__);
                return;
            }
            onoff_model->onoff_state = param->status_cb.onoff_status.present_onoff;
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET unicast: 0x%04x, onoff: 0x%02x", addr, onoff_model->onoff_state);
            break;
        }
        default:
            break;
        }
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
        break;
    case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
        /* If failed to receive the responses, these messages will be resend */
        switch (opcode)
        {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET TIMEOUT unicast: 0x%04x", addr);
            if (node->prov == false)
            {
                xEventGroupSetBits(mesh_evt_group, GET_STATE_REPEAT_BIT);
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
        {
            break;
        }
        default:
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Not a generic client status message event");
        break;
    }
}

esp_err_t ble_mesh_init(void)
{
    uint8_t match[2] = {0xdd, 0xdd};
    esp_err_t err = ESP_OK;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(ble_mesh_config_client_cb);
    esp_ble_mesh_register_generic_client_callback(ble_mesh_generic_client_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set matching device uuid (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable mesh provisioner (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add local AppKey (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_recv_heartbeat(true);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable heartbeat");
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Provisioner initialized");

    return err;
}

void mesh_evt_task(void *param)
{
    EventBits_t evt_bit;
    node_info_t *node = (node_info_t *)param;
    uint16_t model_app_bind_addr = node->unicast_node - 1;
    uint16_t get_state_addr = node->unicast_node - 1;
    while (1)
    {
        evt_bit = xEventGroupWaitBits(mesh_evt_group, MODEL_APP_BIND_BIT | MODEL_APP_BIND_REPEAT_BIT | GET_STATE_REPEAT_BIT | GET_STATE_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (evt_bit & MODEL_APP_BIND_BIT)
        {
            model_app_bind_addr++;
            esp_ble_mesh_client_common_param_t common;
            ble_mesh_app_bind(&common, node, model_app_bind_addr, config_client.model);
        }
        else if (evt_bit & MODEL_APP_BIND_REPEAT_BIT)
        {
            // ESP_LOGW(TAG, "Model app bind repeat");
            esp_ble_mesh_client_common_param_t common;
            ble_mesh_app_bind(&common, node, model_app_bind_addr, config_client.model);
        }
        else if (evt_bit & GET_STATE_BIT)
        {
            get_state_addr++;
            esp_ble_mesh_client_common_param_t common;
            ble_mesh_onoff_get_state(&common, get_state_addr, onoff_client.model);
        }
        else if (evt_bit & GET_STATE_REPEAT_BIT)
        {
            // ESP_LOGW(TAG, "Get state repeat");
            esp_ble_mesh_client_common_param_t common;
            ble_mesh_onoff_get_state(&common, get_state_addr, onoff_client.model);
        }
        else
        {
            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
}

// void button_task(void *param)
// {
//     TickType_t bt_tick = 0;
//     _button button_boot = {
//         .pin = GPIO_NUM_0,
//         .time_down = 0,
//         .time_set = 1000,
//     };
//     gpio_config_t boot_cfg = {
//         .intr_type = GPIO_INTR_DISABLE,
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ONLY,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .pin_bit_mask = (1ULL << button_boot.pin),
//     };
//     esp_err_t err = gpio_config(&boot_cfg);
//     if (err != ESP_OK)
//     {
//         ESP_LOGE(TAG, "button boot config failed (err %d)", err);
//         return;
//     }

//     esp_ble_mesh_generic_client_set_state_t set_state;
//     esp_ble_mesh_client_common_param_t common;

//     while (1)
//     {
//         if (!gpio_get_level(button_boot.pin))
//         {
//             if (bt_tick == 0)
//                 bt_tick = xTaskGetTickCount();
//             button_boot.time_down += xTaskGetTickCount() - bt_tick;
//             bt_tick = xTaskGetTickCount();
//             if (button_boot.time_down >= (button_boot.time_set / portTICK_RATE_MS))
//             {
//                 ble_mesh_set_msg_common(&common, nodes[0].elem[1].unicast_elem, onoff_client.model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET);
//                 set_state.onoff_set.op_en = false;
//                 set_state.onoff_set.onoff = !nodes[0].elem[1].sig_models[0].onoff_state;
//                 set_state.onoff_set.tid = 0;
//                 esp_err_t err = esp_ble_mesh_generic_client_set_state(&common, &set_state);
//                 if (err)
//                 {
//                     ESP_LOGE(TAG, "%s: Generic OnOff Set failed", __func__);
//                 }
//                 while (!gpio_get_level(button_boot.pin))
//                     vTaskDelay(50 / portTICK_RATE_MS);
//             }
//         }
//         else
//         {
//             button_boot.time_down = 0;
//             bt_tick = 0;
//         }
//         vTaskDelay(50 / portTICK_RATE_MS);
//     }
// }

