#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "esp_vfs_fat.h"      // ← pour unmount SD

#include "app_config.h"
#include "rtc_manager.h"
#include "sd_manager.h"
#include "lora_manager.h"
#include "bgt_sensor_manager.h"
#include "sleep_manager.h"

static const char *TAG = "ROBOCARE_SYSTEM";

esp_err_t spi2_bus_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num   = PIN_SPI_MISO,
        .mosi_io_num   = PIN_SPI_MOSI,
        .sclk_io_num   = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    return spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}

void app_main(void)
{
    // --- PHASE 1 : INITIALISATION ---
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  DÉMARRAGE DU NŒUD ÉMETTEUR (ID: %d)", NODE_ID);
    ESP_LOGI(TAG, "==============================================");

    ESP_LOGI(TAG, "[SPI] Initialisation bus SPI2...");
    if (spi2_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "[SPI] ERREUR CRITIQUE — redémarrage");
        esp_restart();
    }
    ESP_LOGI(TAG, "[SPI] Bus SPI2 prêt (SCK:%d MISO:%d MOSI:%d)",
             PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    ESP_LOGI(TAG, "[RTC] Initialisation DS3231...");
    if (!rtc_manager_init(PIN_I2C_SDA, PIN_I2C_SCL)) {
        ESP_LOGW(TAG, "[RTC] DS3231 non détecté");
    }

    ESP_LOGI(TAG, "[SD] Montage carte mémoire...");
    if (!sd_manager_init(PIN_SD_CS)) {
        ESP_LOGW(TAG, "[SD] Carte SD absente ou défectueuse");
    }

    ESP_LOGI(TAG, "[LORA] Configuration SX1278...");
    if (!lora_manager_init(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_IRQ)) {
        ESP_LOGE(TAG, "[LORA] Communication LoRa impossible");
    }

    ESP_LOGI(TAG, "[MODBUS] Initialisation capteur 7-in-1...");
    if (!bgt_sensor_manager_init(PIN_MODBUS_RX, PIN_MODBUS_TX,
                                  PIN_MODBUS_EN, -1, 0x01)) {
        ESP_LOGW(TAG, "[MODBUS] Capteur non détecté au démarrage");
    }

    // --- PHASE 2 : MESURE ET TRANSMISSION ---
    ESP_LOGI(TAG, "----------------------------------------------");
    ESP_LOGI(TAG, "DÉBUT DU CYCLE DE MESURE");

    datetime_t now;
    bgt_sensor_data_t data;
    char lora_msg[128];

    if (rtc_manager_get_datetime(&now)) {
        ESP_LOGI(TAG, "[TIME] %02d/%02d/%04d %02d:%02d:%02d",
                 now.day, now.month, now.year,
                 now.hour, now.minute, now.second);
    }

    ESP_LOGI(TAG, "[SENSOR] Interrogation RS485...");
    if (bgt_sensor_manager_read_all(&data)) {
        ESP_LOGI(TAG, "[DATA] Hum:%.1f%% Temp:%.1f°C pH:%.2f",
                 data.humidity, data.temperature, data.ph);
        ESP_LOGI(TAG, "[DATA] EC:%.0f N:%.0f P:%.0f K:%.0f",
                 data.ec, data.nitrogen, data.phosphorus, data.potassium);

        ESP_LOGI(TAG, "[SD] Écriture capteur_bgt.csv...");
        sd_manager_log_sensor_data(&now, &data);

        snprintf(lora_msg, sizeof(lora_msg),
                 "%d;%.1f;%.1f;%.2f;%.0f;%.0f;%.0f;%.0f",
                 NODE_ID,
                 data.humidity, data.temperature, data.ph,
                 data.ec, data.nitrogen, data.phosphorus, data.potassium);

        ESP_LOGI(TAG, "[LORA] Envoi : %s", lora_msg);
        if (lora_manager_send_message(lora_msg)) {
            ESP_LOGI(TAG, "[LORA] Transmission confirmée (TxDone)");
        } else {
            ESP_LOGW(TAG, "[LORA] Échec transmission (Timeout)");
        }
    } else {
        ESP_LOGE(TAG, "[SENSOR] Échec lecture — vérifier RS485 et alim 5V");
    }

    // --- PHASE 3 : SOMMEIL ---
    ESP_LOGI(TAG, "----------------------------------------------");
    lora_manager_deinit();
    // ✅ CORRECTION 3 : démonter SD proprement avant sleep
    esp_vfs_fat_sdcard_unmount("/sdcard", NULL);

    ESP_LOGI(TAG, "[POWER] Deep Sleep %d sec. Bye Alaa !", SLEEP_DURATION_SEC);
    sleep_manager_configure_timer(SLEEP_DURATION_US);
    sleep_manager_enter_deep_sleep();
}