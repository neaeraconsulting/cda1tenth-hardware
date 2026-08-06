idf_component_register(
    SRCS "main.c" "j2735_messages.c" "mqtt_publisher.c" "v2x_messages.c"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_netif esp_wifi json mqtt nvs_flash)
