idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES freertos esp_common
                                     esp_driver_gpio esp_driver_uart
                                     esp_driver_gptimer esp_driver_ledc
                                     esp_adc
                                     esp_timer
                                     log
                       INCLUDE_DIRS "")
