#ifndef _COMMON_H_
#define _COMMON_H_


typedef struct 
{
    bool available;
    char node_type[15];
    char mac_addr[15];
    int total_switch;
} ble_object_t;

typedef struct 
{
    char action[10];
    char node_id[15];
    char control[10];
    char url[100];
    int duration;
} mqtt_object_t;

typedef enum
{
    LOCAL_MODE,
    NORMAL_MODE,
    SMARTCONFIG,
    FOTA
} status_t;

bool ble_cJSON_process(char *data, ble_object_t *ble_object);
bool mqtt_cJSON_process(char *data, mqtt_object_t *mqtt_object);

#endif