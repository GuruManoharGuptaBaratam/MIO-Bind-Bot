idf_component_register(
    SRCS "src/wifi_client.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_http_client espressif__esp32-camera
)
