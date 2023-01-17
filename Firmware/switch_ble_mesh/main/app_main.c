/**
 * @file app_main.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_device.h"

#include "driver/gpio.h"

#define CID_ESP 0x02E5
#define ON 1
#define OFF 0

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_HB        ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

static const char *TAG = "MAIN";
static uint8_t dev_uuid[16] = {0xdd, 0xdd};

typedef struct
{
    uint8_t current_state;
    uint8_t target_state;
    uint8_t pin;
} smart_switch_t;

smart_switch_t smart_switchs[4] = {
    [0] = {.pin = GPIO_NUM_19, .current_state = ON},
    [1] = {.pin = GPIO_NUM_18, .current_state = ON},
    [2] = {.pin = GPIO_NUM_17, .current_state = ON},
    [3] = {.pin = GPIO_NUM_16, .current_state = ON},
};

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .default_ttl = 10,
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 5, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_1, 5, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_1 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_2, 5, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_2 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_3, 5, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_3 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0),
};

static esp_ble_mesh_model_t extend_model_0[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_1, &onoff_server_1),
};

static esp_ble_mesh_model_t extend_model_1[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_2, &onoff_server_2),
};

static esp_ble_mesh_model_t extend_model_2[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_3, &onoff_server_3),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 1),
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_HB, 1);
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
    vnd_op, NULL, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
    ESP_BLE_MESH_ELEMENT(0, extend_model_0, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_1, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_2, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_actions = ESP_BLE_MESH_PUSH,
    .input_size = 4,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};

void led_init(gpio_num_t gpio_num)
{
    gpio_config_t led_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .pin_bit_mask = (1ULL << gpio_num)};
    gpio_config(&led_cfg);
    gpio_set_level(gpio_num, OFF);
}

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

void relay_change_output(uint8_t index, uint8_t state)
{
    if (smart_switchs[index].current_state != state)
    {
        gpio_set_level((gpio_num_t)smart_switchs[index].pin, state);
        smart_switchs[index].current_state = state;
    }
}

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08x", flags, iv_index);
}

static void ble_mesh_change_state(esp_ble_mesh_model_t *model, esp_ble_mesh_msg_ctx_t *ctx, uint8_t state)
{
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    uint8_t i;
    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst))
    {
        for (i = 0; i < elem_count; i++)
        {
            if (ctx->recv_dst == primary_addr + i)
            {
                relay_change_output(i, state);
            }
        }
    }
    else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst))
    {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst))
        {
            for (i = 0; i < elem_count; i++)
            {
                relay_change_output(i, state);
            }
        }
    }
    else if (ctx->recv_dst == 0xFFFF)
    {
        for (i = 0; i < elem_count; i++)
        {
            relay_change_output(i, state);
        }
    }
}

static void ble_mesh_handle_onoff_message(esp_ble_mesh_model_t *model, esp_ble_mesh_msg_ctx_t *ctx, esp_ble_mesh_server_recv_gen_onoff_set_t *set)
{
    esp_ble_mesh_gen_onoff_srv_t *srv = model->user_data;
    switch (ctx->recv_op)
    {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        esp_ble_mesh_server_model_send_msg(model, ctx, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        break;
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
        if (set->op_en == false)
        {
            srv->state.onoff = set->onoff;
        }
        else
        {
            /* TODO: Delay and state transition */
            srv->state.onoff = set->onoff;
        }
        esp_ble_mesh_server_model_send_msg(model, ctx, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        ble_mesh_change_state(model, ctx, srv->state.onoff);
        break;
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        if (set->op_en == false)
        {
            srv->state.onoff = set->onoff;
        }
        else
        {
            /* TODO: Delay and state transition */
            srv->state.onoff = set->onoff;
        }
        ble_mesh_change_state(model, ctx, srv->state.onoff);
        // esp_ble_mesh_server_model_send_msg(model, ctx, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        break;
    default:
        break;
    }
}

void ble_mesh_bind_app_key(uint16_t app_idx)
{
    ESP_ERROR_CHECK(esp_ble_mesh_node_bind_app_key_to_local_model(esp_ble_mesh_get_primary_element_address(), BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, app_idx));
    ESP_ERROR_CHECK(esp_ble_mesh_node_bind_app_key_to_local_model(esp_ble_mesh_get_primary_element_address() + 1, BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, app_idx));
    ESP_ERROR_CHECK(esp_ble_mesh_node_bind_app_key_to_local_model(esp_ble_mesh_get_primary_element_address() + 2, BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, app_idx));
    ESP_ERROR_CHECK(esp_ble_mesh_node_bind_app_key_to_local_model(esp_ble_mesh_get_primary_element_address() + 3, BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, app_idx));
}

void ble_mesh_publish_state_msg(uint16_t publish_addr)
{
    esp_ble_mesh_model_t *model = NULL;
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    // state = 0x0008 | 0x0007 | 0x0006 | 0x0005
    uint8_t state = (smart_switchs[elem_count - 1].current_state << 3) | (smart_switchs[elem_count - 2].current_state << 2) | (smart_switchs[elem_count - 3].current_state << 1) | (smart_switchs[elem_count - 4].current_state);
    model = esp_ble_mesh_find_sig_model(&elements[0], ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
    if (!model)
    {
        ESP_LOGE(TAG, "%s: Error find model", __func__);
        return;
    }
    if (model->pub == NULL || model->pub->publish_addr == ESP_BLE_MESH_ADDR_UNASSIGNED)
    {
        ESP_LOGW(TAG, "Signed publish address 0x%04x", publish_addr);
        model->pub->publish_addr = publish_addr;
    }
    esp_err_t err = esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, 1, &state, ROLE_NODE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: Error publish model", __func__);
        return;
    }
    ESP_LOGW(TAG, "Publish state: 0x%02x, group address: 0x%04x", state, publish_addr);
}

static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                     esp_ble_mesh_prov_cb_param_t *param)
{
    ESP_LOGW(TAG, "%s: event 0x%02x", __func__, event);
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                       esp_ble_mesh_generic_server_cb_param_t *param)
{
    esp_ble_mesh_gen_onoff_srv_t *srv;
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04x, src 0x%04x, dst 0x%04x",
             event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);
    switch (event)
    {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK)
        {
            ESP_LOGI(TAG, "onoff 0x%02x", param->value.state_change.onoff_set.onoff);
            ble_mesh_change_state(param->model, &param->ctx, param->value.state_change.onoff_set.onoff);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET)
        {
            srv = param->model->user_data;
            ESP_LOGI(TAG, "onoff 0x%02x", srv->state.onoff);
            ble_mesh_handle_onoff_message(param->model, &param->ctx, NULL);
        }
        break;
    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK)
        {
            ESP_LOGI(TAG, "onoff 0x%02x, tid 0x%02x", param->value.set.onoff.onoff, param->value.set.onoff.tid);
            if (param->value.set.onoff.op_en)
            {
                ESP_LOGI(TAG, "trans_time 0x%02x, delay 0x%02x",
                         param->value.set.onoff.trans_time, param->value.set.onoff.delay);
            }
            ble_mesh_handle_onoff_message(param->model, &param->ctx, &param->value.set.onoff);
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

static void ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                      esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
    {
        ESP_LOGW(TAG, "%s: event 0x%04x", __func__, param->ctx.recv_op);
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                     param->value.state_change.appkey_add.net_idx,
                     param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_app_bind.element_addr,
                     param->value.state_change.mod_app_bind.app_idx,
                     param->value.state_change.mod_app_bind.company_id,
                     param->value.state_change.mod_app_bind.model_id);
            ble_mesh_bind_app_key(param->value.state_change.mod_app_bind.app_idx);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_sub_add.element_addr,
                     param->value.state_change.mod_sub_add.sub_addr,
                     param->value.state_change.mod_sub_add.company_id,
                     param->value.state_change.mod_sub_add.model_id);
            esp_ble_mesh_model_subscribe_group_addr(esp_ble_mesh_get_primary_element_address(), BLE_MESH_CID_NVAL, ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, param->value.state_change.mod_sub_add.sub_addr);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET");
            ESP_LOGI(TAG, "elem_addr 0x%04x, pub_addr 0x%04x, app_dix 0x%04x, pub_ttl 0x%02x, pub_period 0x%02x, mod_id 0x%04x", param->value.state_change.mod_pub_set.element_addr,
                     param->value.state_change.mod_pub_set.pub_addr,
                     param->value.state_change.mod_pub_set.app_idx,
                     param->value.state_change.mod_pub_set.pub_ttl,
                     param->value.state_change.mod_pub_set.pub_period,
                     param->value.state_change.mod_pub_set.model_id);
            ble_mesh_publish_state_msg(0xC000);
            break;
        default:
            break;
        }
    }
}

static void ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *param)
{
    ESP_LOGW(TAG, "%s: event 0x%02x", __func__, event);
    switch (event)
    {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OPERATION_EVT");
            if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND)
            {

            }
            else if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_HB)
            {
                ESP_LOGW(TAG, "Receive Gateway heartbeat");
            }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_SEND_COMP_EVT");
        ESP_LOGW(TAG, "opcode: 0x%08x", param->model_operation.opcode);
        break;
    case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT");
        // ble_mesh_publish_state_msg(0xC000);
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT");
        ESP_LOGW(TAG, "msg: 0x%02x", *(param->client_recv_publish_msg.msg));
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT");
        break;
    case ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT");
        break;
    case ESP_BLE_MESH_SERVER_MODEL_UPDATE_STATE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_SERVER_MODEL_UPDATE_STATE_COMP_EVT");
        ESP_LOGW(TAG, "result %d, model id 0x%04x, type 0x%02x",
                 param->server_model_update_state.err_code,
                 param->server_model_update_state.model->model_id,
                 param->server_model_update_state.type);
        break;
    case ESP_BLE_MESH_MODEL_EVT_MAX:
        ESP_LOGE(TAG, "ESP_BLE_MESH_MODEL_EVT_MAX");
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(ble_mesh_config_server_cb);
    esp_ble_mesh_register_generic_server_callback(ble_mesh_generic_server_cb);
    esp_ble_mesh_register_custom_model_callback(ble_mesh_custom_model_cb);
    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    // err = esp_ble_mesh_set_unprovisioned_device_name("SWITCH 4");
    // if (err != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to initialize device name (err %d)", err);
    //     return err;
    // }
    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return err;
}

static esp_err_t bluetooth_init(void)
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

void gpio_init_input(gpio_num_t num)
{
    gpio_config_t config_io;
    config_io.intr_type = GPIO_INTR_DISABLE;
    config_io.mode = GPIO_MODE_INPUT;
    config_io.pull_up_en = GPIO_PULLUP_DISABLE;
    config_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config_io.pin_bit_mask = (1ULL << num);
    gpio_config(&config_io);
}

// void touch_pad_task(void *param)
// {
//     gpio_init_input(GPIO_NUM_4);
//     gpio_init_input(GPIO_NUM_21);
//     gpio_init_input(GPIO_NUM_22);
//     gpio_init_input(GPIO_NUM_23);
//     while (1)
//     {
//         if()
//     }
// }

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = bluetooth_init();
    if (ret)
    {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", ret);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    ret = ble_mesh_init();
    if (ret)
    {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", ret);
    }
    // xTaskCreate(&touch_pad_task, "touch_pad_task", 8192, NULL, 10, NULL);
}
