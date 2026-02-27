# ESP-IDF v5.x: idf_component_register() replaces the old set(COMPONENT_SRCS ...) syntax.
# Listing main.c explicitly here is what makes the linker find app_main.

idf_component_register(
    SRCS
        "main.c" "dht.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        esp_wifi
        spiffs
        driver
        esp_timer

)
