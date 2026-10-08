idf_component_register(
    SRCS 
        "main.c"
        "udp_receiver.c"
        "control_loop.c"
        "wifi_init.c"

    INCLUDE_DIRS 
        "."                  # Where your .h header files live (usually current folder)
        "include"

    REQUIRES 
        esp_driver_gpio
        esp_driver_gptimer
        esp_driver_ledc
        nvs_flash
        esp_wifi
        esp_event
)
