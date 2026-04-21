#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "esp_vfs_fat.h"

#include "app_config.h"
#include "rtc_manager.h"
#include "sd_manager.h"
#include "lora_manager.h"
#include "bgt_sensor_manager.h"
#include "sleep_manager.h"

static const char *TAG = "ROBOCARE_SYSTEM";

esp_err_t spi2_bus_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num     = PIN_SPI_MISO,
        .mosi_io_num     = PIN_SPI_MOSI,
        .sclk_io_num     = PIN_SPI_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096
    };
    return spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}

static void lora_wait_for_slot(void)
{
    datetime_t now;
    if (!rtc_manager_get_datetime(&now)) {
        ESP_LOGW(TAG, "[SLOT] RTC illisible — émission immédiate");
        return;
    }

    int target = LORA_SLOT_SECONDS;
    int current = (int)now.second;
    int wait_sec = target - current;

    if (wait_sec < 0) {
        wait_sec += 60;
    }

    if (wait_sec == 0) {
        ESP_LOGI(TAG, "[SLOT] Node %d — slot immédiat (:%.2d)", NODE_ID, target);
        return;
    }

    ESP_LOGI(TAG, "[SLOT] Node %d — attente %ds (slot :%.2d, maintenant :%.2d)",
             NODE_ID, wait_sec, target, current);

    vTaskDelay(pdMS_TO_TICKS(wait_sec * 1000));
    ESP_LOGI(TAG, "[SLOT] Node %d — émission autorisée", NODE_ID);
}

void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  DÉMARRAGE DU NŒUD ÉMETTEUR (ID: %d)", NODE_ID);
    ESP_LOGI(TAG, "==============================================");

    if (spi2_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "[SPI] ERREUR CRITIQUE — redémarrage");
        esp_restart();
    }

    rtc_manager_init(PIN_I2C_SDA, PIN_I2C_SCL);
    sd_manager_init(PIN_SD_CS);
    lora_manager_init(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_IRQ);
    bgt_sensor_manager_init(PIN_MODBUS_RX, PIN_MODBUS_TX, PIN_MODBUS_EN, -1, 0x01);

    datetime_t now;
    bgt_sensor_data_t data;
    char lora_msg[160];

    if (!rtc_manager_get_datetime(&now)) {
        ESP_LOGW(TAG, "[RTC] Heure indisponible");
        memset(&now, 0, sizeof(now));
    }

    if (bgt_sensor_manager_read_all(&data)) {
        ESP_LOGI(TAG, "[DATA] Hum:%.1f%% Temp:%.1f°C pH:%.2f",
                 data.humidity, data.temperature, data.ph);
        ESP_LOGI(TAG, "[DATA] EC:%.0f N:%.0f P:%.0f K:%.0f",
                 data.ec, data.nitrogen, data.phosphorus, data.potassium);

        sd_manager_log_sensor_data(&now, &data);

        /* FORMAT OFFICIEL LORA :
         * NODE_ID;humidity;temperature;ph;ec;N;P;K;date;time
         */
        snprintf(lora_msg, sizeof(lora_msg),
                 "%d;%.1f;%.1f;%.2f;%.0f;%.0f;%.0f;%.0f;%02d/%02d/%04d;%02d:%02d:%02d",
                 NODE_ID,
                 data.humidity,
                 data.temperature,
                 data.ph,
                 data.ec,
                 data.nitrogen,
                 data.phosphorus,
                 data.potassium,
                 now.day, now.month, now.year,
                 now.hour, now.minute, now.second);

        ESP_LOGI(TAG, "[LORA] Payload = %s", lora_msg);

        lora_wait_for_slot();

        if (lora_manager_send_message(lora_msg)) {
            ESP_LOGI(TAG, "[LORA] Transmission OK");
        } else {
            ESP_LOGW(TAG, "[LORA] Échec transmission");
        }
    } else {
        ESP_LOGW(TAG, "[SENSOR] Lecture capteur échouée");
    }

    sleep_manager_configure_timer(SLEEP_BASE_US);
    sleep_manager_enter_deep_sleep();
}