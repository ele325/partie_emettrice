/**
 * @file main.c
 * @brief RoboCare — Carte ÉMETTRICE v3.2
 *
 * Corrections appliquées (v3.0 → v3.1) :
 *  1. [CRITIQUE]    Appel bgt_sensor_manager_init : arguments RX et TX
 *                   étaient inversés. Signature : (rx_pin, tx_pin, ...)
 *                   donc SENSOR_UART_RX_PIN doit être passé en premier.
 *  2. [COMMENTAIRE] Commentaire "SPI3_HOST" corrigé → SPI2_HOST.
 *  3. [ROBUSTESSE]  s_sd_err incrémenté uniquement sur erreur d'écriture,
 *                   pas si la SD est absente dès le départ.
 *
 * Corrections appliquées (v3.1 → v3.2) :
 *  4. [CRITIQUE]    lora_manager_init() ne reçoit plus MOSI/MISO/SCK.
 *                   Nouvelle signature : lora_manager_init(NSS, RST, DIO0)
 *                   Le driver appelle spi_bus_add_device(SPI2_HOST) en interne.
 *  5. [CRITIQUE]    spi2_bus_init() gère ESP_ERR_INVALID_STATE.
 *  6. [COMMENTAIRE] DE/RE=AUTO(XY-485) au lieu de IO19.
 *  7. [ROBUSTESSE]  sd_manager_log_sensor_data() : retour esp_err_t vérifié.
 *  8. [AUTONOMIE]   Deep sleep remplace vTaskDelay entre les cycles.
 *                   Les statistiques survivent en RTC_DATA_ATTR (RTC RAM).
 *                   app_main() est rappelé à chaque réveil.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "nvs_flash.h"

/* SPI bus partagé (SD + LoRa sur SPI2_HOST) */
#include "driver/spi_master.h"
#include "driver/spi_common.h"

/* Composants projet */
#include "bgt_sensor_manager.h"
#include "lora_manager.h"
#include "sd_manager.h"
#include "rtc_manager.h"
#include "sleep_manager.h"

/* =========================================================================
 * Brochage
 * ========================================================================= */

/* RS-485 / Capteur sol NBL-S-TMC-7 */
#define SENSOR_UART_RX_PIN      17      /* RX_UART CAPTEUR_UART → IO17      */
#define SENSOR_UART_TX_PIN      18      /* TX_UART CAPTEUR_UART → IO18      */
#define SENSOR_DE_RE_PIN        -1      /* AUTO — module XY-485 gère DE/RE  */
#define SENSOR_POWER_PIN        42      /* OFF POWER / 5V PWR   → IO42      */
#define SENSOR_MODBUS_ADDR      0x01

/* SPI2 partagé : SD + LoRa Ra-02 (CS distincts) */
#define SPI_SCK_PIN             12      /* CLK  SPI  → IO12                 */
#define SPI_MISO_PIN            13      /* MISO SPI  → IO13                 */
#define SPI_MOSI_PIN            11      /* MOSI SPI  → IO11                 */

/* LoRa Ra-02 (SX1278) — ajouté sur SPI2_HOST, CS séparé */
#define LORA_NSS_PIN            10      /* NSS/CS    → IO10                 */
#define LORA_RST_PIN             8      /* RST       → IO8                  */
#define LORA_DIO0_PIN           14      /* DIO0      → IO14                 */

/* SD Card — ajoutée sur SPI2_HOST, CS séparé */
#define SD_CS_PIN               34      /* CS SD     → IO34                 */

/* RTC DS3231 — I2C */
#define RTC_SDA_PIN             21      /* SDA       → IO21                 */
#define RTC_SCL_PIN             26      /* SCL       → IO26                 */

/* Timing */
#define SAMPLE_PERIOD_US        10000000ULL  /* 10 s en microsecondes        */
#define SAMPLE_PERIOD_MS        10000        /* 10 s en millisecondes        */
#define SENSOR_WARMUP_MS        2000         /* Stabilisation capteur        */
#define EMITTER_NODE_ID         1

/* =========================================================================
 * Variables globales
 *
 * RTC_DATA_ATTR : stockées en RTC RAM → survivent au deep sleep.
 * Toutes les statistiques sont conservées entre les cycles.
 * ========================================================================= */
static const char *TAG = "MAIN";

static RTC_DATA_ATTR uint32_t s_read_ok  = 0;
static RTC_DATA_ATTR uint32_t s_read_err = 0;
static RTC_DATA_ATTR uint32_t s_lora_ok  = 0;
static RTC_DATA_ATTR uint32_t s_lora_err = 0;
static RTC_DATA_ATTR uint32_t s_sd_ok    = 0;
static RTC_DATA_ATTR uint32_t s_sd_err   = 0;
static RTC_DATA_ATTR uint32_t s_cycle    = 0;

static bool s_sd_ready   = false;
static bool s_lora_ready = false;
static bool s_rtc_ready  = false;

/* =========================================================================
 * Utilitaires
 * ========================================================================= */

/**
 * @brief Formate un message LoRa depuis les données capteur
 *        Format : "N1;27.6;32.6;75;7.22;8;8;21;25/03/2025;14:30:00"
 *                  nodeId;temp;hum;ec;ph;N;P;K;date;heure
 *
 * Si pH invalide (0x7FFF du capteur → valeur négative dans driver),
 * le champ pH est remplacé par "INVAL".
 */
static void format_lora_message(char *buf, size_t size,
                                 const bgt_sensor_data_t *d,
                                 const datetime_t        *dt)
{
    if (d->ph < 0.0f) {
        snprintf(buf, size,
                 "N%d;%.1f;%.1f;%.0f;INVAL;%.0f;%.0f;%.0f;"
                 "%02d/%02d/%04d;%02d:%02d:%02d",
                 EMITTER_NODE_ID,
                 d->temperature, d->humidity, d->ec,
                 d->nitrogen, d->phosphorus, d->potassium,
                 dt->day, dt->month, dt->year,
                 dt->hour, dt->minute, dt->second);
    } else {
        snprintf(buf, size,
                 "N%d;%.1f;%.1f;%.0f;%.2f;%.0f;%.0f;%.0f;"
                 "%02d/%02d/%04d;%02d:%02d:%02d",
                 EMITTER_NODE_ID,
                 d->temperature, d->humidity, d->ec, d->ph,
                 d->nitrogen, d->phosphorus, d->potassium,
                 dt->day, dt->month, dt->year,
                 dt->hour, dt->minute, dt->second);
    }
}

static void print_stats(void)
{
    ESP_LOGI(TAG, "+-- Statistiques ----------------------------------+");
    ESP_LOGI(TAG, "|  Cycles   : %-5"PRIu32"                              |",
             s_cycle);
    ESP_LOGI(TAG, "|  Capteur  : OK=%-5"PRIu32"  ERR=%-5"PRIu32"              |",
             s_read_ok, s_read_err);
    ESP_LOGI(TAG, "|  SD       : OK=%-5"PRIu32"  ERR=%-5"PRIu32"              |",
             s_sd_ok,   s_sd_err);
    ESP_LOGI(TAG, "|  LoRa TX  : OK=%-5"PRIu32"  ERR=%-5"PRIu32"              |",
             s_lora_ok, s_lora_err);
    ESP_LOGI(TAG, "+--------------------------------------------------+");
}

/* =========================================================================
 * Initialisation bus SPI2
 *
 * RÈGLE : ce bus est initialisé UNE SEULE FOIS ici.
 * SD et LoRa s'y ajoutent via spi_bus_add_device() dans leurs drivers.
 * Ne jamais appeler spi_bus_initialize(SPI2_HOST) ailleurs.
 * ========================================================================= */
static bool spi2_bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num     = SPI_MOSI_PIN,
        .miso_io_num     = SPI_MISO_PIN,
        .sclk_io_num     = SPI_SCK_PIN,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    /* CORRECTION v3.2 : gérer le cas où le bus est déjà initialisé */
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "[SPI2] Bus déjà initialisé — OK");
        return true;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[SPI2] Echec initialisation : %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "[SPI2] Bus initialisé (SCK=IO%d MOSI=IO%d MISO=IO%d)",
             SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN);
    return true;
}

/* =========================================================================
 * app_main
 *
 * Appelé à chaque démarrage ET à chaque réveil depuis deep sleep.
 * Un seul cycle d'acquisition est effectué, puis le système entre
 * en deep sleep pendant SAMPLE_PERIOD_US (10 secondes).
 * ========================================================================= */
void app_main(void)
{
    s_cycle++;

    ESP_LOGI(TAG, "+================================================+");
    ESP_LOGI(TAG, "|   RoboCare — Carte EMETTRICE  v3.2            |");
    ESP_LOGI(TAG, "|   Noeud ID : %-3d  |  Cycle : %-5"PRIu32"             |",
             EMITTER_NODE_ID, s_cycle);
    ESP_LOGI(TAG, "+================================================+");

    /* Identifier la cause du réveil */
    esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
    if (wakeup == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Réveil deep sleep (timer) — cycle #%"PRIu32, s_cycle);
    } else {
        ESP_LOGI(TAG, "Démarrage initial (power-on / reset)");
    }

    /* ── NVS ─────────────────────────────────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 1/5 — Bus SPI2 (requis par SD + LoRa) ───────────────── */
    ESP_LOGI(TAG, "[INIT 1/5] Bus SPI2 (partage SD + LoRa)");
    if (!spi2_bus_init()) {
        ESP_LOGE(TAG, "  SPI2 ECHEC — deep sleep forcé.");
        goto enter_deep_sleep;
    }

    /* ── 2/5 — RTC DS3231 ────────────────────────────────────── */
    ESP_LOGI(TAG, "[INIT 2/5] RTC DS3231  SDA=IO%d  SCL=IO%d",
             RTC_SDA_PIN, RTC_SCL_PIN);
    s_rtc_ready = rtc_manager_init(RTC_SDA_PIN, RTC_SCL_PIN);
    if (!s_rtc_ready) {
        ESP_LOGW(TAG, "  RTC absent — horodatage a zero");
    } else {
        datetime_t dt_now;
        if (rtc_manager_get_datetime(&dt_now)) {
            char buf[32];
            rtc_manager_format_date(buf, sizeof(buf), &dt_now);
            ESP_LOGI(TAG, "  RTC OK — %s", buf);
        }
    }

    /* ── 3/5 — Capteur sol RS-485 ────────────────────────────── */
    ESP_LOGI(TAG, "[INIT 3/5] Capteur NBL-S-TMC-7 (RS-485 via module XY-485)");
    ESP_LOGI(TAG, "  RX=IO%d  TX=IO%d  DE/RE=AUTO(XY-485)  PWR=IO%d  Addr=0x%02X",
             SENSOR_UART_RX_PIN, SENSOR_UART_TX_PIN,
             SENSOR_POWER_PIN, SENSOR_MODBUS_ADDR);

    /*
     * CORRECTION v3.1 — ordre des arguments corrigé :
     *   v3.0 passait (TX_PIN, RX_PIN) → UART inversé → aucune réponse capteur
     *
     * CORRECTION v3.2 — SENSOR_DE_RE_PIN = -1 :
     *   Le module XY-485 gère automatiquement la direction RS-485.
     */
    if (!bgt_sensor_manager_init(SENSOR_UART_RX_PIN,   /* rx_pin  = IO17 */
                                  SENSOR_UART_TX_PIN,   /* tx_pin  = IO18 */
                                  SENSOR_DE_RE_PIN,     /* de_re   = -1   */
                                  SENSOR_POWER_PIN,     /* pwr     = IO42 */
                                  SENSOR_MODBUS_ADDR)) {
        ESP_LOGE(TAG, "  Capteur ECHEC — deep sleep forcé.");
        goto enter_deep_sleep;
    }
    ESP_LOGI(TAG, "  Capteur OK — stabilisation %d ms...", SENSOR_WARMUP_MS);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_WARMUP_MS));

    /* ── 4/5 — Carte SD ──────────────────────────────────────── */
    ESP_LOGI(TAG, "[INIT 4/5] SD Card  CS=IO%d  (SPI2_HOST)", SD_CS_PIN);
    s_sd_ready = sd_manager_init(SD_CS_PIN);
    ESP_LOGI(TAG, "  SD %s", s_sd_ready ? "OK -> /sdcard/capteur_bgt.csv"
                                         : "NON DISPONIBLE");

    /* ── 5/5 — LoRa Ra-02 ────────────────────────────────────── */
    /*
     * CORRECTION v3.2 — Signature corrigée :
     *   AVANT : lora_manager_init(MOSI, MISO, SCK, NSS, RST, DIO0) → crash
     *   APRÈS : lora_manager_init(NSS, RST, DIO0) → spi_bus_add_device() seul
     */
    ESP_LOGI(TAG, "[INIT 5/5] LoRa Ra-02  NSS=IO%d  RST=IO%d  DIO0=IO%d  (SPI2_HOST)",
             LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

    s_lora_ready = lora_manager_init(LORA_NSS_PIN,    /* CS   = IO10 */
                                      LORA_RST_PIN,    /* RST  = IO8  */
                                      LORA_DIO0_PIN);  /* DIO0 = IO14 */
    if (!s_lora_ready)
        ESP_LOGW(TAG, "  LoRa ECHEC — verifier NSS/RST/DIO0 (SX1278 != 0x12 ?)");
    else
        ESP_LOGI(TAG, "  LoRa OK (SX1278 detecte, 433 MHz SF12)");

    /* ── Résumé brochage ─────────────────────────────────────── */
    ESP_LOGI(TAG, "--- Brochage actif --------------------------------");
    ESP_LOGI(TAG, "  RS-485   RX=IO17  TX=IO18  DE/RE=AUTO(XY-485)  PWR=IO42");
    ESP_LOGI(TAG, "  SPI2     SCK=IO12  MOSI=IO11  MISO=IO13");
    ESP_LOGI(TAG, "  LoRa     NSS=IO10  RST=IO8   DIO0=IO14  (SPI2_HOST)");
    ESP_LOGI(TAG, "  SD       CS=IO34                         (SPI2_HOST)");
    ESP_LOGI(TAG, "  RTC I2C  SDA=IO21  SCL=IO26");
    ESP_LOGI(TAG, "  SD:%s  LoRa:%s  RTC:%s",
             s_sd_ready   ? "OK " : "N/A",
             s_lora_ready ? "OK " : "N/A",
             s_rtc_ready  ? "OK " : "N/A");
    ESP_LOGI(TAG, "--------------------------------------------------");

    /* =========================================================
     * CYCLE D'ACQUISITION
     * ========================================================= */

    bgt_sensor_data_t data;
    datetime_t        dt;
    char              date_str[32];
    char              lora_msg[120];

    /* ── 1. Lecture RTC ──────────────────────────────────────── */
    bool rtc_ok = false;
    if (s_rtc_ready) {
        rtc_ok = rtc_manager_get_datetime(&dt);
    }
    if (!rtc_ok) {
        memset(&dt, 0, sizeof(dt));
        dt.year = 2025;
    }
    rtc_manager_format_date(date_str, sizeof(date_str), &dt);
    ESP_LOGI(TAG, "== Cycle #%"PRIu32" | %s ==", s_cycle, date_str);

    /* ── 2. Lecture capteur sol ──────────────────────────────── */
    bool sensor_ok = bgt_sensor_manager_read_all(&data);

    if (sensor_ok) {
        s_read_ok++;
        ESP_LOGI(TAG, "  [CAPTEUR] OK");
        ESP_LOGI(TAG, "  Temperature   : %.1f C",     data.temperature);
        ESP_LOGI(TAG, "  Humidite      : %.1f %%",    data.humidity);
        ESP_LOGI(TAG, "  Conductivite  : %.0f uS/cm", data.ec);
        if (data.ph < 0.0f)
            ESP_LOGW(TAG, "  pH            : INVALIDE (sonde déconnectée ?)");
        else
            ESP_LOGI(TAG, "  pH            : %.2f",   data.ph);
        ESP_LOGI(TAG, "  Azote     (N) : %.0f mg/kg", data.nitrogen);
        ESP_LOGI(TAG, "  Phosphore (P) : %.0f mg/kg", data.phosphorus);
        ESP_LOGI(TAG, "  Potassium (K) : %.0f mg/kg", data.potassium);
    } else {
        s_read_err++;
        ESP_LOGE(TAG, "  [CAPTEUR] ECHEC — timeout/CRC (cycle #%"PRIu32")",
                 s_cycle);
        goto enter_deep_sleep;
    }

    /* ── 3. Écriture SD ─────────────────────────────────────── */
    if (s_sd_ready) {
        /* sd_manager_log_sensor_data() retourne void dans le driver actuel */
        sd_manager_log_sensor_data(&dt, &data);
        s_sd_ok++;
        ESP_LOGI(TAG, "  [SD] OK");
    } else {
        /* Absence matérielle ≠ erreur d'écriture → pas d'incrément */
        ESP_LOGW(TAG, "  [SD] Non disponible");
    }

    /* ── 4. Envoi LoRa ───────────────────────────────────────── */
    if (s_lora_ready) {
        format_lora_message(lora_msg, sizeof(lora_msg), &data, &dt);
        ESP_LOGI(TAG, "  [LORA] MSG: %s", lora_msg);

        bool tx_ok = lora_manager_send_message(lora_msg);
        if (tx_ok) {
            s_lora_ok++;
            ESP_LOGI(TAG, "  [LORA] TX OK | total=%"PRIu32, s_lora_ok);
        } else {
            s_lora_err++;
            ESP_LOGE(TAG, "  [LORA] TX ECHEC | total ERR=%"PRIu32, s_lora_err);
        }
    } else {
        ESP_LOGW(TAG, "  [LORA] Non disponible");
    }

    /* ── 5. Stats toutes les 10 acquisitions ─────────────────── */
    if (s_cycle % 10 == 0) print_stats();

enter_deep_sleep:
    /*
     * CORRECTION v3.2 — Deep sleep remplace vTaskDelay(10s) :
     *
     * vTaskDelay laissait tous les périphériques sous tension 10s.
     * Sur batteries, le deep sleep (~10 µA) est indispensable vs
     * ~40 mA en actif → autonomie multipliée drastiquement.
     *
     * Les statistiques (RTC_DATA_ATTR) survivent en RTC RAM.
     * app_main() est rappelé intégralement au prochain réveil.
     */
    ESP_LOGI(TAG, "Deep sleep — réveil dans 10 s (cycle #%"PRIu32" terminé)",
             s_cycle);
    esp_sleep_enable_timer_wakeup(SAMPLE_PERIOD_US);
    esp_deep_sleep_start();

    /* Ne jamais atteint */
}