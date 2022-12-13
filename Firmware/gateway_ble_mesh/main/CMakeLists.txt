idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "wifi_sta.c" "common.c" "button.c" "mqtt.c" "ble_hub.c" "main.c" "fota.c" "led.c" "smartconfig.c" "spiffs_user.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES ${project_dir}/certs/git_ota.pem)
