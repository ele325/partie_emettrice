#include "sd_manager.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "bgt_sensor_manager.h"

static const char *TAG = "SD";
static sdmmc_card_t *card = NULL;
static bool sd_initialized = false;

bool sd_manager_init(int cs_pin)
{
    esp_err_t ret;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs_pin;
    slot_config.host_id = host.slot;
    
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Échec montage SD: %s", esp_err_to_name(ret));
        return false;
    }
    
    sd_initialized = true;
    ESP_LOGI(TAG, "Carte SD initialisée");
    
    // Créer l'en-tête du fichier CSV si nécessaire
    FILE *f = fopen("/sdcard/capteur_bgt.csv", "r");
    if (!f) {
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

void sd_manager_log_sensor_data(const datetime_t *datetime, const bgt_sensor_data_t *data)
{
    if (!sd_initialized || !datetime || !data) return;
    
    FILE *f = fopen("/sdcard/capteur_bgt.csv", "a");
    if (f) {
        fprintf(f, "%02d/%02d/%04d;%02d:%02d;%.1f;%.1f;%.0f;%.1f;%.0f;%.0f;%.0f\n",
                datetime->day, datetime->month, datetime->year,
                datetime->hour, datetime->minute,
                data->humidity, data->temperature, data->ec, data->ph,
                data->nitrogen, data->phosphorus, data->potassium);
        fclose(f);
        ESP_LOGI(TAG, "Données enregistrées sur SD");
    } else {
        ESP_LOGE(TAG, "Erreur ouverture fichier SD");
    }
}