idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES rtc_manager bgt_sensor_manager sd_manager lora_manager sleep_manager)