idf_component_register(SRCS "sd_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES fatfs sdmmc driver rtc_manager bgt_sensor_manager)