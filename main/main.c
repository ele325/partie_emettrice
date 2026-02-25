#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "rtc_manager.h"
#include "bgt_sensor_manager.h"
#include "sd_manager.h"
#include "lora_manager.h"
#include "sleep_manager.h"

static const char *TAG = "MAIN_CAPTEUR";

// Configuration des broches
#define LORA_CS     10
#define LORA_RST    6
#define LORA_IRQ    5
#define SD_CS       12
#define SCK         7
#define MISO        9
#define MOSI        11

#define MODBUS_RX   4
#define MODBUS_TX   3
#define MODBUS_EN   2

#define I2C_SDA     8
#define I2C_SCL     13

#define NODE_ID     1
#define SLEEP_TIME_US (14400 * 1000000ULL)  // 4 heures en microsecondes

void app_main(void)
{
    ESP_LOGI(TAG, "Démarrage capteur BGT-SMPS (ID: %d)", NODE_ID);
    
    // 1. Initialiser le bus SPI (partagé entre LoRa et SD)
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur initialisation SPI: %s", esp_err_to_name(ret));
    }
    
    // 2. Initialiser le RTC
    if (!rtc_manager_init(I2C_SDA, I2C_SCL)) {
        ESP_LOGW(TAG, "RTC non détecté, horodatage indisponible");
    }
    
    // 3. Initialiser la carte SD
    if (!sd_manager_init(SD_CS)) {
        ESP_LOGW(TAG, "Carte SD non détectée");
    }
    
    // 4. Initialiser le capteur BGT-SMPS
    if (!bgt_sensor_manager_init(MODBUS_RX, MODBUS_TX, MODBUS_EN, 1)) {
        ESP_LOGE(TAG, "Échec initialisation capteur BGT-SMPS");
    }
    
    // 5. Laisser le temps au capteur de démarrer
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 6. Lire les données du capteur
    bgt_sensor_data_t sensor_data = {0};
    bool read_success = bgt_sensor_manager_read_all(&sensor_data);
    
    if (read_success) {
        ESP_LOGI(TAG, "Lecture capteur réussie");
    } else {
        ESP_LOGE(TAG, "Échec lecture capteur");
    }
    
    // 7. Récupérer date/heure pour horodatage
    datetime_t now;
    bool rtc_ok = rtc_manager_get_datetime(&now);
    
    // 8. Enregistrer sur SD
    if (rtc_ok) {
        sd_manager_log_sensor_data(&now, &sensor_data);
    } else {
        ESP_LOGW(TAG, "Pas d'horodatage - données non enregistrées sur SD");
    }
    
    // 9. Initialiser LoRa
    if (lora_manager_init(MOSI, MISO, SCK, LORA_CS, LORA_RST, LORA_IRQ)) {
        
        // 10. Préparer et envoyer le message LoRa
        char message[128];
        bgt_sensor_manager_format_message(NODE_ID, &sensor_data, message, sizeof(message));
        
        if (lora_manager_send_message(message)) {
            ESP_LOGI(TAG, "Message LoRa envoyé avec succès");
        } else {
            ESP_LOGE(TAG, "Échec envoi LoRa");
        }
    } else {
        ESP_LOGE(TAG, "Échec initialisation LoRa");
    }
    
    // 11. Attendre un peu pour que les logs série soient transmis
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 12. Configurer et entrer en deep sleep
    ESP_LOGI(TAG, "Cycle terminé, passage en deep sleep pour %llu secondes", 
             SLEEP_TIME_US / 1000000);
    
    sleep_manager_configure_timer(SLEEP_TIME_US);
    sleep_manager_enter_deep_sleep();
    
    // Ne devrait jamais arriver ici
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}