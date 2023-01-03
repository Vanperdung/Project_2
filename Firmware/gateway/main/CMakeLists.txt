idf_component_register(SRCS "fota.c" "button.c" "smartconfig.c" "common.c" "ble_mesh_user.c" "mqtt.c" "wifi_sta.c" "main.c" "led.c" "heartbeat.c" "spiffs_user.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES ${project_dir}/certs/git_ota.pem)
