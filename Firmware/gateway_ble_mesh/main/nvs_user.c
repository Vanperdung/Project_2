/**
 * @file nvs_user.c
 * @author Vanperdung (dung.nv382001@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "nvs_user.h"

#define NVS_NAME "nvs_gateway"

static const char *TAG = "NVS_USER";

esp_err_t ble_mesh_nvs_open(nvs_handle_t *handle)
{
    esp_err_t err = ESP_OK;
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Open, invalid nvs handle");
        return ESP_ERR_INVALID_ARG;
    }

    err = nvs_open(NVS_NAME, NVS_READWRITE, handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Open, nvs_open failed, err %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Open namespace done, name \"%s\"", NVS_NAME);
    return err;
}

esp_err_t ble_mesh_nvs_store(nvs_handle_t handle, const char *key, const void *data, size_t length)
{
    esp_err_t err = ESP_OK;

    if (key == NULL || data == NULL || length == 0)
    {
        ESP_LOGE(TAG, "Store, invalid parameter");
        return ESP_ERR_INVALID_ARG;
    }

    err = nvs_set_blob(handle, key, data, length);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Store, nvs_set_blob failed, err %d", err);
        return err;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Store, nvs_commit failed, err %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Store, key \"%s\", length %u", key, length);
    ESP_LOG_BUFFER_HEX("NVS_GATEWAY: Store, data", data, length);

    return err;
}

esp_err_t ble_mesh_nvs_get_length(nvs_handle_t handle, const char *key, size_t *length)
{
    esp_err_t err = ESP_OK;

    if (key == NULL || length == NULL)
    {
        ESP_LOGE(TAG, "Get length, invalid parameter");
        return ESP_ERR_INVALID_ARG;
    }

    err = nvs_get_blob(handle, key, NULL, length);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "Get length, key \"%s\" not exists", key);
        *length = 0;
        return ESP_OK;
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Get length, nvs_get_blob failed, err %d", err);
    }
    else
    {
        ESP_LOGI(TAG, "Get length, key \"%s\", length %u", key, *length);
    }

    return err;
}

esp_err_t ble_mesh_nvs_restore(nvs_handle_t handle, const char *key, void *data, size_t length, bool *exist)
{
    esp_err_t err = ESP_OK;

    if (key == NULL || data == NULL || length == 0)
    {
        ESP_LOGE(TAG, "Restore, invalid parameter");
        return ESP_ERR_INVALID_ARG;
    }

    err = nvs_get_blob(handle, key, data, &length);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "Restore, key \"%s\" not exists", key);
        if (exist)
        {
            *exist = false;
        }
        return ESP_OK;
    }

    if (exist)
    {
        *exist = true;
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Restore, nvs_get_blob failed, err %d", err);
    }
    else
    {
        ESP_LOGI(TAG, "Restore, key \"%s\", length %u", key, length);
        ESP_LOG_BUFFER_HEX("NVS_GATEWAY: Restore, data", data, length);
    }

    return err;
}

esp_err_t ble_mesh_nvs_erase(nvs_handle_t handle, const char *key)
{
    esp_err_t err = ESP_OK;

    if (key)
    {
        err = nvs_erase_key(handle, key);
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGI(TAG, "Erase, key \"%s\" not exists", key);
            return ESP_OK;
        }
    }
    else
    {
        err = nvs_erase_all(handle);
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erase, nvs_erase_%s failed, err %d", key ? "key" : "all", err);
        return err;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erase, nvs_commit failed, err %d", err);
        return err;
    }

    if (key)
    {
        ESP_LOGI(TAG, "Erase done, key \"%s\"", key);
    }
    else
    {
        ESP_LOGI(TAG, "Erase namespace done, name \"%s\"", NVS_NAME);
    }
    return err;
}