/**
 * @file ble_mesh_user.h
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _BLE_MESH_USER_H_
#define _BLE_MESH_USER_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define MAXIMUM_NODE 20
#define CID_ESP 0x02E5
#define PROV_OWN_ADDR 0x0001
#define MSG_SEND_TTL 3
#define MSG_SEND_REL false
#define MSG_TIMEOUT 0
#define MSG_ROLE ROLE_PROVISIONER
#define COMP_DATA_PAGE_0 0x00
#define APP_KEY_IDX 0x0000
#define APP_KEY_OCTET 0x12
#define ON 0x1
#define OFF 0x0
#define UNPROV_DEV_QUEUE_SIZE 10;

typedef struct
{
    uint16_t model_id;
    uint8_t model_name[20];
    uint8_t onoff_state;
} model_info_t;

typedef struct
{
    uint16_t unicast_elem;
    uint16_t loc;
    uint8_t numS;
    uint8_t numV;
    model_info_t sig_models[10];
} elem_info_t;

typedef struct
{
    uint8_t uuid[16];
    uint16_t unicast_node;
    uint8_t elem_num;
    uint16_t cid;
    uint16_t pid;
    uint16_t vid;
    uint16_t crpl;
    uint16_t feature;
    elem_info_t elem[10];
} node_info_t;

typedef enum
{
    NONE_EVT,
    MODEL_APP_BIND_EVT,
    MODEL_GET_STATE_EVT,
} prov_event_t;

typedef struct 
{
    int elem_index;
    bool model_app_bind_flag;
} model_app_bind_t;

typedef struct 
{
    node_info_t *node;
    prov_event_t evt;
    model_app_bind_t model_app_bind;
} prov_node_info_t;

void prov_dev_task(void *param);
esp_err_t bluetooth_init(void);
void ble_mesh_get_dev_uuid(uint8_t *dev_uuid);
esp_err_t ble_mesh_init(void);
esp_err_t ble_mesh_app_bind(esp_ble_mesh_client_common_param_t *common, node_info_t *node, uint16_t unicast_elem, esp_ble_mesh_model_t *model);
esp_err_t ble_mesh_onoff_set_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model);
esp_err_t ble_mesh_onoff_get_state(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem, esp_ble_mesh_model_t *onoff_client_model);
esp_err_t ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common, uint16_t unicast_elem,
                                  esp_ble_mesh_model_t *client_model, uint32_t opcode);
model_info_t *ble_mesh_get_model_info_with_model_id(uint16_t unicast, uint16_t model_id);
elem_info_t *ble_mesh_get_elem_info_with_unicast(uint16_t unicast);
node_info_t *ble_mesh_get_node_info_with_uuid(uint8_t *uuid);
node_info_t *ble_mesh_get_node_info_with_unicast(uint16_t unicast);
void ble_mesh_store_composition_info(node_info_t *comp, uint8_t *comp_data, int elem_num);
void ble_mesh_get_number_node(void);
esp_err_t ble_mesh_store_node_info(const uint8_t uuid[16], uint16_t unicast, uint8_t elem_num);
prov_node_info_t *ble_mesh_read_prov_node_info(void);
esp_err_t ble_mesh_read_prov_node_queue(void);
void ble_mesh_delete_prov_node_info(prov_node_info_t *prov_node);
prov_node_info_t *ble_mesh_get_prov_node_info(node_info_t *node);
esp_err_t ble_mesh_store_prov_node_info(node_info_t *node, prov_event_t evt);
void decode_comp_data(node_info_t *comp, uint8_t *comp_data, int elem_num);
esp_err_t ble_mesh_delete_node_info_in_flash(node_info_t *comp);
esp_err_t ble_mesh_restore_node_info_in_flash(node_info_t *comp);
esp_err_t ble_mesh_store_node_info_in_flash(node_info_t *comp);

#endif

