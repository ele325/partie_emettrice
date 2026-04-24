#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"

#include "app_config.h"
#include "rtc_manager.h"
#include "sd_manager.h"
#include "lora_manager.h"
#include "bgt_sensor_manager.h"
#include "sleep_manager.h"

static const char *TAG = "ROBOCARE_SYSTEM";

static void spi2_deassert_shared_cs_lines(void)
{
    /* SD + SX1278 partagent SPI2. Si un CS flotte Ã  LOW, il peut perturber
     * l'autre pÃ©riphÃ©rique (MISO pilotÃ© en mÃªme temps) et faire Ã©chouer
     * l'init SD/LoRa. On force les CS Ã  HIGH avant toute init. */
    const int cs_pins[] = { PIN_SD_CS, PIN_LORA_CS };

    for (size_t i = 0; i < sizeof(cs_pins) / sizeof(cs_pins[0]); i++) {
        int pin = cs_pins[i];
        if (pin < 0) continue;
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        gpio_set_level((gpio_num_t)pin, 1);
    }
}

/* ─── Init bus SPI2 ──────────────────────────────────────────────── */
esp_err_t spi2_bus_init(void)
{
    printf("[DBG][SPI] spi2_bus_init()\n");
    printf("[DBG][SPI]   SCK  = IO%d\n", PIN_SPI_SCK);
    printf("[DBG][SPI]   MISO = IO%d\n", PIN_SPI_MISO);
    printf("[DBG][SPI]   MOSI = IO%d\n", PIN_SPI_MOSI);

    spi_bus_config_t buscfg = {
        .miso_io_num     = PIN_SPI_MISO,
        .mosi_io_num     = PIN_SPI_MOSI,
        .sclk_io_num     = PIN_SPI_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    printf("[DBG][SPI] spi_bus_initialize → %s (0x%X)\n",
           esp_err_to_name(ret), ret);
    return ret;
}

/* ─── Attente slot LoRa ──────────────────────────────────────────── */
static void lora_wait_for_slot(void)
{
    printf("[DBG][SLOT] lora_wait_for_slot() NODE_ID=%d "
           "LORA_SLOT_SECONDS=%d\n", NODE_ID, LORA_SLOT_SECONDS);

    datetime_t now;
    if (!rtc_manager_get_datetime(&now)) {
        ESP_LOGW(TAG, "[SLOT] RTC illisible — émission immédiate");
        printf("[DBG][SLOT] RTC illisible → pas d'attente\n");
        return;
    }

    printf("[DBG][SLOT] Heure RTC : %02d:%02d:%02d\n",
           now.hour, now.minute, now.second);

    int target   = LORA_SLOT_SECONDS;
    int current  = (int)now.second;
    int wait_sec = target - current;
    if (wait_sec < 0) wait_sec += 60;

    printf("[DBG][SLOT] target=%d current=%d wait=%ds\n",
           target, current, wait_sec);

    if (wait_sec == 0) {
        ESP_LOGI(TAG, "[SLOT] Node %d — slot immédiat (:%.2d)",
                 NODE_ID, target);
        return;
    }

    ESP_LOGI(TAG, "[SLOT] Node %d — attente %ds (slot :%.2d, maintenant :%.2d)",
             NODE_ID, wait_sec, target, current);
    vTaskDelay(pdMS_TO_TICKS(wait_sec * 1000));
    ESP_LOGI(TAG, "[SLOT] Node %d — émission autorisée", NODE_ID);
}

/* ─── app_main ───────────────────────────────────────────────────── */
void app_main(void)
{
    printf("\n\n");
    printf("========================================\n");
    printf("[DBG][MAIN] app_main() démarré\n");
    printf("[DBG][MAIN] NODE_ID       = %d\n", NODE_ID);
    printf("[DBG][MAIN] SLEEP_BASE_US = %llu µs\n", SLEEP_BASE_US);
    printf("[DBG][MAIN] PIN_SD_CS     = IO%d\n", PIN_SD_CS);
    printf("[DBG][MAIN] PIN_LORA_CS   = IO%d\n", PIN_LORA_CS);
    printf("[DBG][MAIN] PIN_LORA_RST  = IO%d\n", PIN_LORA_RST);
    printf("[DBG][MAIN] PIN_LORA_IRQ  = IO%d\n", PIN_LORA_IRQ);
    printf("[DBG][MAIN] PIN_MODBUS_TX = IO%d\n", PIN_MODBUS_TX);
    printf("[DBG][MAIN] PIN_MODBUS_RX = IO%d\n", PIN_MODBUS_RX);
    printf("[DBG][MAIN] PIN_MODBUS_EN = IO%d\n", PIN_MODBUS_EN);
    printf("========================================\n\n");

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  DÉMARRAGE DU NŒUD ÉMETTEUR (ID: %d)", NODE_ID);
    ESP_LOGI(TAG, "==============================================");

    /* ── SPI ──────────────────────────────────────────────────────── */
    printf("[DBG][MAIN] Étape 1/6 : Init SPI2\n");
    if (spi2_bus_init() != ESP_OK) {
        printf("[DBG][MAIN] ERREUR CRITIQUE SPI — restart\n");
        ESP_LOGE(TAG, "[SPI] ERREUR CRITIQUE — redémarrage");
        esp_restart();
    }
    spi2_deassert_shared_cs_lines();
    printf("[DBG][MAIN] SPI2 OK\n\n");

    /* ── RTC ──────────────────────────────────────────────────────── */
    printf("[DBG][MAIN] Étape 2/6 : Init RTC (SDA=IO%d SCL=IO%d)\n",
           PIN_I2C_SDA, PIN_I2C_SCL);
    bool rtc_ok = rtc_manager_init(PIN_I2C_SDA, PIN_I2C_SCL);
    printf("[DBG][MAIN] rtc_manager_init → %s\n", rtc_ok ? "OK" : "ÉCHEC");
    printf("\n");

    /* ── SD ───────────────────────────────────────────────────────── */
    printf("[DBG][MAIN] Étape 3/6 : Init SD (CS=IO%d)\n", PIN_SD_CS);
    if (PIN_SD_CS == 34) {
        printf("[DBG][MAIN] IO34 est input-only sur ESP32-S2 !\n");
        printf("[DBG][MAIN] PIN_SD_CS doit être sur un GPIO output "
               "(ex: IO5/IO8/IO15...)\n");
    }
    bool sd_ok = sd_manager_init(PIN_SD_CS);
    printf("[DBG][MAIN] sd_manager_init → %s\n", sd_ok ? "OK" : "ÉCHEC");
    if (!sd_ok) {
        printf("[DBG][MAIN] SD non disponible — log local désactivé\n");
        printf("[DBG][MAIN] Causes possibles :\n");
        printf("  1. PIN_SD_CS=IO%d invalide (input-only?)\n", PIN_SD_CS);
        printf("  2. Carte SD absente ou mal insérée\n");
        printf("  3. Alimentation SD insuffisante\n");
    }
    printf("\n");

    /* ── LoRa ─────────────────────────────────────────────────────── */
    printf("[DBG][MAIN] Étape 4/6 : Init LoRa "
           "(CS=IO%d RST=IO%d IRQ=IO%d)\n",
           PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_IRQ);
    bool lora_ok = lora_manager_init(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_IRQ);
    printf("[DBG][MAIN] lora_manager_init → %s\n",
           lora_ok ? "OK" : "ÉCHEC");
    if (!lora_ok) {
        printf("[DBG][MAIN] LoRa ÉCHEC — causes possibles :\n");
        printf("  1. SPI MOSI/MISO inversés (IO%d/IO%d)\n",
               PIN_SPI_MOSI, PIN_SPI_MISO);
        printf("  2. CS LoRa IO%d non actif\n", PIN_LORA_CS);
        printf("  3. Alimentation 3.3V du module Ra-01 absente\n");
        printf("  4. VERSION=0x00 → module non répondu\n");
    }
    printf("\n");

    /* ── Capteur ──────────────────────────────────────────────────── */
    printf("[DBG][MAIN] Étape 5/6 : Init capteur sol "
           "(RX=IO%d TX=IO%d EN=IO%d addr=0x01)\n",
           PIN_MODBUS_RX, PIN_MODBUS_TX, PIN_MODBUS_EN);
    bool sensor_ok = bgt_sensor_manager_init(
        PIN_MODBUS_RX, PIN_MODBUS_TX, PIN_MODBUS_EN, -1, 0x01);
    printf("[DBG][MAIN] bgt_sensor_manager_init → %s\n",
           sensor_ok ? "OK" : "ÉCHEC");
    printf("\n");

    /* ── Lecture + envoi ──────────────────────────────────────────── */
    printf("[DBG][MAIN] Étape 6/6 : Lecture données + envoi LoRa\n");

    datetime_t now;
    bgt_sensor_data_t data;
    char lora_msg[160];

    if (!rtc_manager_get_datetime(&now)) {
        ESP_LOGW(TAG, "[RTC] Heure indisponible");
        printf("[DBG][MAIN] RTC heure indisponible → zéro\n");
        memset(&now, 0, sizeof(now));
    } else {
        printf("[DBG][MAIN] RTC : %02d/%02d/%04d %02d:%02d:%02d\n",
               now.day, now.month, now.year,
               now.hour, now.minute, now.second);
    }

    if (sensor_ok && bgt_sensor_manager_read_all(&data)) {
        printf("[DBG][MAIN] ✓ Données capteur lues avec succès\n");
        ESP_LOGI(TAG, "[DATA] Hum:%.1f%% Temp:%.1f°C pH:%.2f",
                 data.humidity, data.temperature, data.ph);
        ESP_LOGI(TAG, "[DATA] EC:%.0f N:%.0f P:%.0f K:%.0f",
                 data.ec, data.nitrogen, data.phosphorus, data.potassium);

        if (sd_ok) {
            printf("[DBG][MAIN] Écriture log SD...\n");
            sd_manager_log_sensor_data(&now, &data);
        } else {
            printf("[DBG][MAIN] SD non dispo → log ignoré\n");
        }

        snprintf(lora_msg, sizeof(lora_msg),
                 "%d;%.1f;%.1f;%.2f;%.0f;%.0f;%.0f;%.0f;"
                 "%02d/%02d/%04d;%02d:%02d:%02d",
                 NODE_ID,
                 data.humidity, data.temperature, data.ph, data.ec,
                 data.nitrogen, data.phosphorus, data.potassium,
                 now.day, now.month, now.year,
                 now.hour, now.minute, now.second);

        printf("[DBG][MAIN] Payload LoRa (%d chars) : %s\n",
               (int)strlen(lora_msg), lora_msg);
        ESP_LOGI(TAG, "[LORA] Payload = %s", lora_msg);

        if (lora_ok) {
            printf("[DBG][MAIN] Attente slot LoRa...\n");
            lora_wait_for_slot();
            printf("[DBG][MAIN] Envoi LoRa...\n");
            if (lora_manager_send_message(lora_msg)) {
                ESP_LOGI(TAG, "[LORA] Transmission OK");
                printf("[DBG][MAIN] ✓ LoRa transmis\n");
            } else {
                ESP_LOGW(TAG, "[LORA] Échec transmission");
                printf("[DBG][MAIN] ✗ LoRa échec\n");
            }
        } else {
            printf("[DBG][MAIN] LoRa non init → envoi ignoré\n");
        }
    } else {
        ESP_LOGW(TAG, "[SENSOR] Lecture capteur échouée");
        printf("[DBG][MAIN] ✗ Capteur non lu\n");
        printf("[DBG][MAIN]   sensor_ok=%d\n", sensor_ok);
    }

    /* ── Deep sleep ───────────────────────────────────────────────── */
    printf("[DBG][MAIN] Entrée deep sleep dans 2s...\n");
    printf("[DBG][MAIN] Durée : %llu µs (%.1f heures)\n",
           SLEEP_BASE_US, (float)SLEEP_BASE_US / 3600000000.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));  /* Laisser le temps aux printf */

    sleep_manager_configure_timer(SLEEP_BASE_US);
    sleep_manager_enter_deep_sleep();
}
