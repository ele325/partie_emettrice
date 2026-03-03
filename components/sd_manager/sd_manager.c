#include "sd_manager.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "SD_MANAGER";
static sdmmc_card_t *card = NULL;
static bool sd_initialized = false;

/**
 * Initialise la carte SD en utilisant un bus SPI déjà existant
 */
bool sd_manager_init(int cs_pin)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initialisation de la carte SD (CS: %d)...", cs_pin);

    // Configuration du montage FATFS
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, // Ne pas formater automatiquement
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    // Configuration du Host SD sur le bus SPI2 (établi dans le main)
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST; 
    // Note: On ne réinitialise pas le bus ici, on l'utilise simplement.

    // Configuration du Slot (Chip Select et liaison au bus)
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs_pin;
    slot_config.host_id = SPI2_HOST; // Crucial pour éviter l'erreur "host_id not initialized"
    
    // Tentative de montage du système de fichiers
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Échec du montage du système de fichiers. "
                         "Vérifiez que la carte est formatée en FAT32.");
        } else {
            ESP_LOGE(TAG, "Erreur d'initialisation SD (%s). Code: 0x%x", 
                          esp_err_to_name(ret), ret);
        }
        return false;
    }
    
    sd_initialized = true;
    ESP_LOGI(TAG, "✅ Carte SD montée avec succès !");
    
    // Vérification/Création du fichier CSV pour tes données
    FILE *f = fopen("/sdcard/capteur_bgt.csv", "r");
    if (!f) {
        ESP_LOGI(TAG, "Création du fichier capteur_bgt.csv...");
        f = fopen("/sdcard/capteur_bgt.csv", "w");
        if (f) {
            fprintf(f, "Date;Heure;Humidite;Temperature;EC;pH;N;P;K\n");
            fclose(f);
        }
    } else {
        fclose(f);
    }
    
    return true;
}

/**
 * Enregistre les données du capteur dans le fichier CSV
 */
void sd_manager_log_sensor_data(const datetime_t *datetime, const bgt_sensor_data_t *data)
{
    if (!sd_initialized) {
        ESP_LOGW(TAG, "Tentative de log alors que la SD n'est pas prête");
        return;
    }
    
    if (!datetime || !data) return;
    
    FILE *f = fopen("/sdcard/capteur_bgt.csv", "a");
    if (f) {
        fprintf(f, "%02d/%02d/%04d;%02d:%02d;%.1f;%.1f;%.0f;%.1f;%.0f;%.0f;%.0f\n",
                datetime->day, datetime->month, datetime->year,
                datetime->hour, datetime->minute,
                data->humidity, data->temperature, data->ec, data->ph,
                data->nitrogen, data->phosphorus, data->potassium);
        fclose(f);
        ESP_LOGI(TAG, "💾 Données enregistrées : %.1f%% HR, %.1f°C", 
                      data->humidity, data->temperature);
    } else {
        ESP_LOGE(TAG, "❌ Erreur d'ouverture du fichier CSV sur la SD");
    }
}