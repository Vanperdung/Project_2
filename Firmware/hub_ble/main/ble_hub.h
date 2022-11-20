#ifndef _BLE_CLIENT_USER_H_
#define _BLE_CLIENT_USER_H_

#define SCAN_DURATION 30
#define MAX_DEVICES 50
#define INVALID_HANDLE 0

#define PROFILE_NUM 7
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define PROFILE_C_APP_ID 2
#define PROFILE_D_APP_ID 3
#define PROFILE_E_APP_ID 4
#define PROFILE_F_APP_ID 5
#define PROFILE_G_APP_ID 6

#define PROFILE_A_APP_INDEX PROFILE_A_APP_ID
#define PROFILE_B_APP_INDEX PROFILE_B_APP_ID
#define PROFILE_C_APP_INDEX PROFILE_C_APP_ID
#define PROFILE_D_APP_INDEX PROFILE_D_APP_ID
#define PROFILE_E_APP_INDEX PROFILE_E_APP_ID
#define PROFILE_F_APP_INDEX PROFILE_F_APP_ID
#define PROFILE_G_APP_INDEX PROFILE_G_APP_ID

typedef struct
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
} gattc_profile_inst;

#define NODE_LIST_FILE "node_list.bin"
#define NODE_0 "node_0.txt"
#define NODE_1 "node_1.txt"
#define NODE_2 "node_2.txt"
#define NODE_3 "node_3.txt"
#define NODE_4 "node_4.txt"
#define NODE_5 "node_5.txt"
#define NODE_6 "node_6.txt"

void ble_init(void);
void start_scanning(int duration);

#endif