idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        esp_netif
        esp_event
        esp_wifi
        esp_http_client
        driver
        esp_lcd
        esp_psram
        esp_lcd_st7735
)
