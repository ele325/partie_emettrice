

#include "sd_manager.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "SD_MANAGER";
static sdmmc_card_t *card          = NULL;
static bool          sd_initialized = false;

/* =========================================================================
 * sd_manager_init
 *
 * PRÉREQUIS : spi_bus_initialize(SPI2_HOST, ...) doit avoir été appelé
 * AVANT cette fonction (dans spi2_bus_init() de main.c).
 * Cette fonction utilise le bus SPI2 déjà initialisé — elle ne le recrée pas.
 * ========================================================================= */
bool sd_manager_init(int cs_pin)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initialisation carte SD (CS=IO%d)...", cs_pin);

    /* Configuration montage FAT */
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,  /* ne pas reformater si erreur    */
        .max_files              = 5,
        .allocation_unit_size   = 16 * 1024,
    };

    /* Host SPI — SDSPI_HOST_DEFAULT() initialise flags correctement.
     *
     * CORRECTION : NE PAS écraser host.flags après SDSPI_HOST_DEFAULT().
     * L'ancienne ligne "host.flags = SDMMC_HOST_FLAG_SPI" supprimait
     * SDMMC_HOST_FLAG_DEINIT_ARG et causait un crash au démontage.        */
    sdmmc_host_t host   = SDSPI_HOST_DEFAULT();
    host.slot           = SPI2_HOST;
    /* host.flags → NE PAS MODIFIER — déjà correct via SDSPI_HOST_DEFAULT() */

    /* Slot SPI — CS et liaison au bus SPI2 */
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs  = cs_pin;
    slot_config.host_id  = SPI2_HOST;   /* crucial : bus déjà initialisé   */

    /* Montage du système de fichiers FAT32 */
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config,
                                   &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Echec montage FAT — vérifier format FAT32");
        } else {
            ESP_LOGE(TAG, "Erreur SD : %s (0x%x)",
                     esp_err_to_name(ret), ret);
        }
        return false;
    }

    sd_initialized = true;
    ESP_LOGI(TAG, "Carte SD montée sur /sdcard");

    /* Création du fichier CSV si absent (première utilisation) */
    FILE *f = fopen("/sdcard/capteur_bgt.csv", "r");
    if (!f) {
        ESP_LOGI(TAG, "Création capteur_bgt.csv + entête...");
        f = fopen("/sdcard/capteur_bgt.csv", "w");
        if (f) {
            fprintf(f, "Date;Heure;Humidite;Temperature;EC;pH;N;P;K\n");
            fclose(f);
            ESP_LOGI(TAG, "Fichier créé avec entête CSV");
        } else {
            ESP_LOGE(TAG, "Impossible de créer capteur_bgt.csv");
        }
    } else {
        fclose(f);
        ESP_LOGI(TAG, "Fichier capteur_bgt.csv existant — données ajoutées");
    }

    return true;
}

/* =========================================================================
 * sd_manager_log_sensor_data
 *
 * Ajoute une ligne CSV avec toutes les mesures du capteur NBL-S-TMC-7.
 * Format : DD/MM/YYYY;HH:MM;H%;T°C;EC;pH;N;P;K
 *
 * CORRECTION : pH formaté en %.2f (cohérent avec message LoRa).
 *              L'ancienne version utilisait %.1f → perte de précision.
 * ========================================================================= */
void sd_manager_log_sensor_data(const datetime_t        *datetime,
                                 const bgt_sensor_data_t *data)
{
    if (!sd_initialized) {
        ESP_LOGW(TAG, "Log SD ignoré — carte non initialisée");
        return;
    }
    if (!datetime || !data) return;

    FILE *f = fopen("/sdcard/capteur_bgt.csv", "a");
    if (f) {
        /* pH : %.2f pour cohérence avec le message LoRa (ex: 7.22)
         * Si pH invalide (< 0), on écrit -1.00                           */
        fprintf(f,
                "%02d/%02d/%04d;%02d:%02d;"
                "%.1f;%.1f;%.0f;%.2f;%.0f;%.0f;%.0f\n",
                datetime->day, datetime->month, datetime->year,
                datetime->hour, datetime->minute,
                data->humidity,
                data->temperature,
                data->ec,
                data->ph,          /* %.2f — CORRECTION v1.1              */
                data->nitrogen,
                data->phosphorus,
                data->potassium);
        fclose(f);
        ESP_LOGI(TAG, "Log SD : %.1f%% HR, %.1f C, pH=%.2f",
                 data->humidity, data->temperature, data->ph);
    } else {
        ESP_LOGE(TAG, "Erreur ouverture capteur_bgt.csv");
    }
}