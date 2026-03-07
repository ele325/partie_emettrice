idf_component_register(
    SRCS
        "main.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp_common
        esp_system

    PRIV_REQUIRES
        nvs_flash           # nvs_flash.h
        freertos            # FreeRTOS.h, task.h
        log                 # esp_log.h
        driver              # spi_master.h, spi_common.h, uart, gpio
        bgt_sensor_manager  # composant capteur sol
        lora_manager        # composant LoRa Ra-02
        sd_manager          # composant carte SD
        rtc_manager         # composant RTC DS3231
        sleep_manager       # composant deep sleep
)