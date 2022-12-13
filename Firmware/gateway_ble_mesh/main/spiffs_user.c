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
#include "wifi_sta.h"
#include "common.h"
#include "main.h"
#include "spiffs_user.h"

static const char *TAG = "SPIFFS";

esp_err_t write_to_file(char *file_name, char *buf)
{
	char *base_path = "/spiffs";
	char file[64];
	sprintf(file, "%s/%s", base_path, file_name);
	FILE *f = NULL;
	f = fopen(file, "w");
	if (f == NULL)
	{
		ESP_LOGE(TAG, "Failed to open file for writing");
		return ESP_FAIL;
	}
	fputs(buf, f);
	fclose(f);
	return ESP_OK;
}

esp_err_t read_from_file(char *file_name, char *buf)
{
	char *base_path = "/spiffs";
	char file[64];
	char line[1024];
	sprintf(file, "%s/%s", base_path, file_name);
	FILE *f = NULL;
	f = fopen(file, "r");
	if (f == NULL)
	{
		ESP_LOGE(TAG, "Failed to open file for writing");
		return ESP_FAIL;
	}
	fgets(line, sizeof(line), f);
	fclose(f);
	char *pos = strchr(line, '\n');
	if (pos)
		*pos = '\0';
	strcpy(buf, line);
	// ESP_LOGI(TAG, "Read from file: %s", buf);
	return ESP_OK;
}

void mount_SPIFFS(void)
{
	ESP_LOGI(TAG, "SPIFFS init");
	esp_vfs_spiffs_conf_t spiffs_cfg = {
		.base_path = "/spiffs",
		.partition_label = "storage",
		.max_files = 5,
		.format_if_mount_failed = true};
	esp_err_t ret = esp_vfs_spiffs_register(&spiffs_cfg);
	if (ret != ESP_OK)
	{
		if (ret == ESP_FAIL)
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		else if (ret == ESP_ERR_NOT_FOUND)
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		else
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
	}
}

void remove_file(char *file_name)
{
	struct stat st;
	if (stat(file_name, &st) == 0)
		unlink(file_name);
}
