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

static const char *TAG = "MAIN_TEST_SD";

/**
 * CONFIGURATION DES PINS - VALIDÉE PAR SCHÉMA ET EXCEL
 */

// --- BUS SPI (Partagé) ---
#define SPI_MOSI      11  // IO11
#define SPI_MISO      13  // IO13
#define SPI_SCLK      12  // IO12
#define SD_CS         34  // IO34 (Chip Select Carte SD)
#define LORA_CS       14  // IO14 (NSS LoRa - Doit être HIGH pour tester la SD)

void app_main(void)
{
    ESP_LOGI(TAG, "=== DÉMARRAGE DU TEST CIBLÉ SD SEULE ===");

    // 1. NEUTRALISATION DU MODULE LORA
    // On force NSS_LoRa à HIGH pour éviter qu'il ne perturbe le bus SPI
    gpio_reset_pin(LORA_CS);
    gpio_set_direction(LORA_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_CS, 1);
    ESP_LOGI(TAG, "LoRa désactivé (CS HIGH sur IO14)");

    // 2. INITIALISATION DU BUS SPI COMMUN
    // Pour l'ESP32-S2, on utilise SPI2_HOST pour les pins personnalisés
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
        return;
    }
    ESP_LOGI(TAG, "Bus SPI2 initialisé (11, 12, 13)");

    // 3. INITIALISATION DE LA CARTE SD VIA MANAGER
    // On passe le CS_PIN (34) au manager
    if (sd_manager_init(SD_CS)) {
        ESP_LOGI(TAG, "✅ SUCCÈS : Carte SD montée");

        // 4. TEST D'ÉCRITURE PHYSIQUE
        ESP_LOGI(TAG, "Ouverture de /sdcard/data.txt...");
        FILE *f = fopen("/sdcard/data.txt", "w");
        if (f) {
            for (int i = 1; i <= 5; i++) {
                fprintf(f, "Ligne de test %d - ESP32-S2 OK\n", i);
                printf("  -> Écriture ligne %d\n", i);
            }
            fclose(f);
            ESP_LOGI(TAG, "💾 FICHIER ENREGISTRÉ AVEC SUCCÈS !");
        } else {
            ESP_LOGE(TAG, "❌ ERREUR : Impossible de créer le fichier sur la SD");
        }
    } else {
        ESP_LOGE(TAG, "❌ ÉCHEC : Le manager n'a pas pu monter la SD");
    }

    ESP_LOGI(TAG, "=== FIN DU TEST ===");
    
    // Boucle infinie pour maintenir le monitor ouvert
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}