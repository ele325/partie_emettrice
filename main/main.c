#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// Gestionnaires de périphériques
#include "rtc_manager.h"
#include "bgt_sensor_manager.h"
#include "sd_manager.h"
#include "lora_manager.h"
#include "sleep_manager.h"

static const char *TAG = "MAIN_CAPTEUR";

/**
 * CONFIGURATION DES PINS - STRICTEMENT BASÉE SUR L'ANALYSE CSV & SCHÉMA
 */

// --- BUS SPI (Partagé entre LoRa Ra-01 et Carte SD) ---
#define SPI_MOSI      11  // IO11
#define SPI_MISO      13  // IO13
#define SPI_SCLK      12  // IO12 (Marqué CLK)
#define SD_CS         34  // IO34 (Chip Select Carte SD)
#define LORA_CS       14  // IO14 (NSS / Chip Select LoRa)

// --- MODULE LORA (Signaux additionnels) ---
#define LORA_RST      10  // IO10 (D'après Analyse Pin_last.csv)
#define LORA_IRQ      -1  // À définir si une interruption est câblée (souvent IO5 ou IO15)

// --- BUS I2C (Horloge RTC DS3231MZ) ---
#define I2C_SDA       21  // IO21
#define I2C_SCL       26  // IO26

// --- UART / MODBUS (Capteur BGT-SMPS) ---
#define MODBUS_RX     17  // IO17
#define MODBUS_TX     18  // IO18
#define MODBUS_EN     19  // IO19 (Utilisé comme GPIO Input/Output sur schéma)

// --- PARAMÈTRES SYSTÈME ---
#define NODE_ID       1
#define SLEEP_TIME_US (14400 * 1000000ULL) // 4 heures en microsecondes

void app_main(void)
{
    ESP_LOGI(TAG, "Démarrage Node ID: %d", NODE_ID);

    // 1. Initialisation du Bus SPI commun
    // L'ESP32-S2 utilise le SPI2_HOST pour les pins personnalisés
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur fatale initialisation SPI: %s", esp_err_to_name(ret));
    }

    // 2. Initialisation du RTC DS3231 (I2C)
    if (rtc_manager_init(I2C_SDA, I2C_SCL)) {
        ESP_LOGI(TAG, "RTC prêt sur IO21/IO26");
    } else {
        ESP_LOGE(TAG, "RTC non détecté !");
    }

    // 3. Initialisation de la Carte SD (SPI)
    if (sd_manager_init(SD_CS)) {
        ESP_LOGI(TAG, "Carte SD montée avec succès sur IO34");
    } else {
        ESP_LOGW(TAG, "Échec montage Carte SD");
    }

    // 4. Initialisation du capteur Modbus
    if (bgt_sensor_manager_init(MODBUS_RX, MODBUS_TX, MODBUS_EN, 1)) {
        
        // Laisser le capteur se stabiliser après mise sous tension
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        bgt_sensor_data_t sensor_data = {0};
        if (bgt_sensor_manager_read_all(&sensor_data)) {
            ESP_LOGI(TAG, "Lecture capteur Modbus réussie");

            // 5. Horodatage et stockage SD
            datetime_t now;
            if (rtc_manager_get_datetime(&now)) {
                sd_manager_log_sensor_data(&now, &sensor_data);
            }

            // 6. Envoi des données par LoRa
            // Note : On passe les pins SPI au manager pour configuration du device SPI LoRa
            if (lora_manager_init(SPI_MOSI, SPI_MISO, SPI_SCLK, LORA_CS, LORA_RST, LORA_IRQ)) {
                char message[128];
                bgt_sensor_manager_format_message(NODE_ID, &sensor_data, message, sizeof(message));
                
                if (lora_manager_send_message(message)) {
                    ESP_LOGI(TAG, "Transmission LoRa terminée");
                } else {
                    ESP_LOGE(TAG, "Erreur transmission LoRa");
                }
            }
        } else {
            ESP_LOGE(TAG, "Capteur Modbus ne répond pas");
        }
    }

    // 7. Mise en sommeil profond (Deep Sleep)
    ESP_LOGI(TAG, "Entrée en Deep Sleep pour 4h...");
    vTaskDelay(pdMS_TO_TICKS(500)); // Laisse le temps aux logs série de sortir
    
    sleep_manager_configure_timer(SLEEP_TIME_US);
    sleep_manager_enter_deep_sleep();
}