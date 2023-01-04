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
#include "esp_spiffs.h"

#include "driver/gpio.h"

#include "cJSON.h"
#include "mqtt.h"
#include "wifi_sta.h"
#include "button.h"
#include "ble_mesh_user.h"
#include "smartconfig.h"
#include "common.h"
#include "spiffs_user.h"

#define MODEL_APP_BIND_BIT BIT0
#define GET_STATE_BIT BIT3
#define ONOFF_GROUP_ADDR 0xC000
static const char *TAG = "BLE MESH USER";
extern uint8_t dev_uuid[16];
extern EventGroupHandle_t prov_evt_group;
extern TaskHandle_t prov_dev_handle;
extern char topic_commands_set[50];
extern char topic_commands_get[50];
extern char topic_commands_status[50];
extern char topic_commands_heartbeat[50];
extern char topic_commands_network[50];
extern char topic_commands_process[50];
extern char topic_commands_version[50];
extern char topic_commands_fota[50];
extern char topic_commands_group[50];
extern esp_mqtt_client_handle_t client;
extern status_t status;

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
    },
};

prov_node_info_t prov_nodes[5] = {
    [0 ... 4] = {
        .node = NULL,
        .evt = NONE_EVT,
        .model_app_bind = {
            .elem_index = 0,
            .model_app_bind_flag = false,
        },
        .model_sub_add = {
            .elem_index = 0,
            .model_sub_add_flag = false,
        }},
};

esp_ble_mesh_client_t config_client;
esp_ble_mesh_client_t onoff_client;

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

esp_err_t ble_mesh_store_prov_node_info(node_info_t *node, prov_event_t evt)
{
    for (int i = 0; i < ARRAY_SIZE(prov_nodes); i++)
    {
        if (prov_nodes[i].node == node && prov_nodes[i].evt == evt)
        {
            ESP_LOGW(TAG, "%s: Already exist prov_node 0x%04x", __func__, node->unicast_node);
            return ESP_OK;
        }
        else if (prov_nodes[i].node == node && prov_nodes[i].evt != evt)
        {
            // prov_nodes[i].node = node;
            ESP_LOGW(TAG, "%s: Change event prov_node 0x%04x", __func__, node->unicast_node);
            prov_nodes[i].evt = evt;
            return ESP_OK;
        }
    }
    for (int i = 0; i < ARRAY_SIZE(prov_nodes); i++)
    {
        if (prov_nodes[i].node == NULL && prov_nodes[i].evt == NONE_EVT)
        {
            ESP_LOGW(TAG, "%s: Store prov_node 0x%04x", __func__, node->unicast_node);
            prov_nodes[i].node = node;
            prov_nodes[i].evt = evt;
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "%s: Full queue, can not add prov_node 0x%04x", __func__, node->unicast_node);
    return ESP_FAIL;
}

prov_node_info_t *ble_mesh_get_prov_node_info(node_info_t *node)
{
    for (int i = 0; i < ARRAY_SIZE(prov_nodes); i++)
    {
        if (prov_nodes[i].node == node)
        {
            return &prov_nodes[i];
        }
    }
    ESP_LOGE(TAG, "Can not get prov node info");
    return NULL;
}

void ble_mesh_delete_prov_node_info(prov_node_info_t *prov_node)
{
    ESP_LOGW(TAG, "%s: Delete prov_node 0x%04x", __func__, prov_node->node->unicast_node);
    prov_node->node = NULL;
    prov_node->evt = NONE_EVT;
    prov_node->model_app_bind.elem_index = 0;
    prov_node->model_app_bind.model_app_bind_flag = false;
    prov_node->model_sub_add.elem_index = 0;
    prov_node->model_sub_add.model_sub_add_flag = false;
}

esp_err_t ble_mesh_read_prov_node_queue(void)
{
    for (int i = 0; i < ARRAY_SIZE(prov_nodes); i++)
    {
        if (prov_nodes[i].node != NULL)
        {
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

prov_node_info_t *ble_mesh_read_prov_node_info(void)
{
    for (int i = 0; i < ARRAY_SIZE(prov_nodes); i++)
    {
        if (prov_nodes[i].node != NULL)
        {
            return &prov_nodes[i];
        }
    }
    return NULL;
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

void ble_mesh_get_number_node(void)
{
    int j = 1;
    for (int i = 0; i < ARRAY_SIZE(nodes); i++)
    {
        if (strlen((char *)nodes[i].uuid))
        {
            ESP_LOGW(TAG, "Node %d: unicast node 0x%04x", j, nodes[i].unicast_node);
            j++;
        }
    }
    if (j == 1)
        ESP_LOGW(TAG, "No node yet");
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

esp_err_t ble_mesh_onoff_set_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model, uint8_t state)
{
    esp_ble_mesh_generic_client_set_state_t set_state = {0};
    model_info_t *model = ble_mesh_get_model_info_with_model_id(unicast_elem, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
    model->target_state = state;
    ble_mesh_set_msg_common(common, unicast_elem, onoff_client_model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET);
    set_state.onoff_set.op_en = false;
    set_state.onoff_set.onoff = state;
    set_state.onoff_set.tid = 0;
    esp_err_t err = esp_ble_mesh_generic_client_set_state(common, &set_state);
    if (err)
    {
        ESP_LOGE(TAG, "%s: Generic OnOff Set failed", __func__);
    }
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

esp_err_t ble_mesh_sub_add_group(esp_ble_mesh_client_common_param_t *common, node_info_t *node, uint16_t unicast_elem, esp_ble_mesh_model_t *model)
{
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    ble_mesh_set_msg_common(common, node->unicast_node, model, ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD);
    set_state.model_sub_add.element_addr = unicast_elem;
    set_state.model_sub_add.sub_addr = ONOFF_GROUP_ADDR;
    set_state.model_sub_add.model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
    set_state.model_sub_add.company_id = ESP_BLE_MESH_CID_NVAL;
    esp_err_t err = esp_ble_mesh_config_client_set_state(common, &set_state);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send Config Model Subscription Add");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_mesh_deinit(void)
{
    esp_err_t err = ESP_OK;
    esp_ble_mesh_deinit_param_t param;
    param.erase_flash = false;
    err = esp_ble_mesh_deinit(&param);
    if (err == ESP_OK)
    {
        ESP_LOGW(TAG, "esp_ble_mesh_deinit success (err %d)", err);
    }
    else
    {
        ESP_LOGE(TAG, "esp_ble_mesh_deinit fail (err %d)", err);
    }
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    return err;
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
    esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
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

    // ESP_LOGW(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04x",
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
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: 0x%04x", addr);
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
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        {
            char group_payload[100] = {0};
            prov_node_info_t *prov_node = ble_mesh_get_prov_node_info(node);
            int index = prov_node->model_sub_add.elem_index;
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD: 0x%04x", prov_node->node->elem[index].unicast_elem);
            prov_node->model_sub_add.model_sub_add_flag = true;
            sprintf(group_payload, "{\"action\":\"add_group\",\"group_addr\":\"0x%04x\",\"unicast_addr\":%d}", ONOFF_GROUP_ADDR, prov_node->node->elem[index].unicast_elem);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_group, group_payload, strlen(group_payload), 0, 0);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD: 0x%04x", addr);
            ble_mesh_store_prov_node_info(node, MODEL_APP_BIND_EVT);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND: 0x%04x", param->status_cb.model_app_status.element_addr);
            prov_node_info_t *prov_node = ble_mesh_get_prov_node_info(node);
            prov_node->model_app_bind.model_app_bind_flag = true;
            ble_mesh_onoff_get_state(&common, param->status_cb.model_app_status.element_addr, onoff_client.model);
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
            ESP_LOGW(TAG, "Get composition data timeout, unicast address: 0x%04x", addr);
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
            ESP_LOGW(TAG, "Add app key timeout, unicast address: 0x%04x", addr);
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
            prov_node_info_t *prov_node = ble_mesh_get_prov_node_info(node);
            int index = prov_node->model_app_bind.elem_index;
            ble_mesh_app_bind(&common, node, prov_node->node->elem[index].unicast_elem, config_client.model);
            ESP_LOGW(TAG, "Model App Bind timeout, unicast address: 0x%04x", prov_node->node->elem[index].unicast_elem);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        {
            prov_node_info_t *prov_node = ble_mesh_get_prov_node_info(node);
            int index = prov_node->model_sub_add.elem_index;
            ble_mesh_sub_add_group(&common, prov_node->node, prov_node->node->elem[index].unicast_elem, config_client.model);
            ESP_LOGW(TAG, "Model Sub Add timeout, unicast address: 0x%04x", prov_node->node->elem[index].unicast_elem);
            break;
        }
        default:
            ESP_LOGE(TAG, "%s, Event not handle", __func__);
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
    addr = param->params->ctx.addr; // Element address

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
            char status_payload[200] = {0};
            onoff_model = ble_mesh_get_model_info_with_model_id(param->params->ctx.addr, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            if (!onoff_model)
            {
                ESP_LOGE(TAG, "%s: Get model info failed", __func__);
                return;
            }
            onoff_model->onoff_state = param->status_cb.onoff_status.present_onoff;
            sprintf(status_payload, "{\"action\":\"onoff\",\"status\":\"get\",\"unicast_addr\":%u,\"state\":%u}", addr, onoff_model->onoff_state);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, status_payload, strlen(status_payload), 0, 0);
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET unicast: 0x%04x, onoff: 0x%02x", addr, onoff_model->onoff_state);
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
            char status_payload[200] = {0};
            onoff_model = ble_mesh_get_model_info_with_model_id(addr, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            if (!onoff_model)
            {
                ESP_LOGE(TAG, "%s: Get model info failed", __func__);
                return;
            }
            onoff_model->onoff_state = param->status_cb.onoff_status.present_onoff;
            sprintf(status_payload, "{\"action\":\"onoff\",\"status\":\"set\",\"unicast_addr\":%u,\"state\":%u}", addr, onoff_model->onoff_state);
            if (status == NORMAL_MODE)
                esp_mqtt_client_publish(client, topic_commands_status, status_payload, strlen(status_payload), 0, 0);
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
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET TIMEOUT, unicast: 0x%04x", addr);
            esp_ble_mesh_client_common_param_t common;
            ble_mesh_onoff_get_state(&common, addr, onoff_client.model);
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
        {
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET TIMEOUT, unicast: 0x%04x", addr);
            esp_ble_mesh_client_common_param_t common;
            model_info_t *model = ble_mesh_get_model_info_with_model_id(addr, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            ble_mesh_onoff_set_state(&common, addr, onoff_client.model, model->target_state);
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

    ble_mesh_get_number_node();
    return err;
}

void prov_dev_task(void *param)
{
    BaseType_t ret;
    prov_node_info_t *prov_node = NULL;
    esp_ble_mesh_client_common_param_t common;
    while (1)
    {
        ret = ble_mesh_read_prov_node_queue();
        if (ret == ESP_OK)
        {
            prov_node = ble_mesh_read_prov_node_info();
            if (prov_node->evt == MODEL_APP_BIND_EVT)
            {
                char nw_payload[100] = {0};
                for (int i = 0; i < prov_node->node->elem_num; i++)
                {
                    prov_node->model_app_bind.model_app_bind_flag = false;
                    prov_node->model_app_bind.elem_index = i;
                    ble_mesh_app_bind(&common, prov_node->node, prov_node->node->elem[i].unicast_elem, config_client.model);
                    while (!prov_node->model_app_bind.model_app_bind_flag)
                    {
                        vTaskDelay(200 / portTICK_RATE_MS);
                    }
                }
                sprintf(nw_payload, "{\"action\":\"join\",\"uuid\":\"%s\",\"unicast_addr\":%u,\"element_num\":%u}", bt_hex(prov_node->node->uuid, strlen((char *)prov_node->node->uuid)), prov_node->node->unicast_node, prov_node->node->elem_num);
                if (status == NORMAL_MODE)
                    esp_mqtt_client_publish(client, topic_commands_network, nw_payload, strlen(nw_payload), 0, 0);
                ble_mesh_store_prov_node_info(prov_node->node, MODEL_SUB_ADD_EVT);
            }
            else if (prov_node->evt == MODEL_SUB_ADD_EVT)
            {
                for (int i = 0; i < prov_node->node->elem_num; i++)
                {
                    prov_node->model_sub_add.model_sub_add_flag = false;
                    prov_node->model_sub_add.elem_index = i;
                    ble_mesh_sub_add_group(&common, prov_node->node, prov_node->node->elem[i].unicast_elem, config_client.model);
                    while (!prov_node->model_sub_add.model_sub_add_flag)
                    {
                        vTaskDelay(200 / portTICK_RATE_MS);
                    }
                }
                ble_mesh_delete_prov_node_info(prov_node);
            }
            else
                ESP_LOGE(TAG, "Prov node event wrong");
        }
        else
        {
            vTaskDelay(50 / portTICK_RATE_MS);
        }
    }
}