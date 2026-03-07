

#include "rtc_manager.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "RTC";
static bool rtc_initialized = false;

/* Adresse I2C du DS3231 */
#define DS3231_ADDR     0x68

/* Registres DS3231 */
#define DS3231_REG_SEC  0x00
#define DS3231_REG_MIN  0x01
#define DS3231_REG_HOUR 0x02
#define DS3231_REG_DAY  0x03   /* jour de semaine (1–7) — ignoré             */
#define DS3231_REG_DATE 0x04   /* jour du mois (1–31)                         */
#define DS3231_REG_MON  0x05   /* mois (1–12), bit7 = century                 */
#define DS3231_REG_YEAR 0x06   /* année sur 2 chiffres (0–99) → +2000         */

/* ─── Conversion BCD ──────────────────────────────────────────────────────── */
static uint8_t bcd2dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t dec2bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}


bool rtc_manager_init(int sda_pin, int scl_pin)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda_pin,
        .scl_io_num       = scl_pin,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur configuration I2C : %s", esp_err_to_name(ret));
        return false;
    }

    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    /* CORRECTION v1.1 — deep sleep : driver peut déjà être installé */
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "[I2C] Driver déjà installé — OK");
        /* Ne pas retourner false : le bus est fonctionnel, on continue */
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur installation I2C : %s", esp_err_to_name(ret));
        return false;
    }

    /* Vérifier présence DS3231 (ping I2C) */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS3231 non détecté sur I2C (SDA=IO%d SCL=IO%d)",
                 sda_pin, scl_pin);
        return false;
    }

    rtc_initialized = true;
    ESP_LOGI(TAG, "RTC DS3231 initialisé (SDA=IO%d SCL=IO%d)",
             sda_pin, scl_pin);
    return true;
}

/* =========================================================================
 * rtc_manager_get_datetime
 *
 * Lit les 7 registres du DS3231 en une seule transaction I2C.
 *
 * Disposition des 7 octets reçus :
 *   [0] secondes  (BCD, bits 6-0)
 *   [1] minutes   (BCD, bits 6-0)
 *   [2] heures    (BCD, bit6=12h/24h, bits 5-0)
 *   [3] jour semaine (1–7) — NON utilisé ici
 *   [4] jour mois (BCD, bits 5-0)
 *   [5] mois      (BCD, bits 4-0 ; bit7 = century, ignoré)
 *   [6] année     (BCD, 00–99) → +2000
 * ========================================================================= */
bool rtc_manager_get_datetime(datetime_t *datetime)
{
    if (!rtc_initialized || !datetime) return false;

    uint8_t data[7];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS3231_REG_SEC, true);  /* pointeur registre  */
    i2c_master_start(cmd);                              /* repeated start     */
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur lecture RTC : %s", esp_err_to_name(ret));
        return false;
    }

    datetime->second = bcd2dec(data[0] & 0x7F);        /* bit7 = oscillateur */
    datetime->minute = bcd2dec(data[1] & 0x7F);
    datetime->hour   = bcd2dec(data[2] & 0x3F);        /* mode 24h           */
    /* data[3] = jour semaine — ignoré                                        */
    datetime->day    = bcd2dec(data[4] & 0x3F);
    datetime->month  = bcd2dec(data[5] & 0x1F);        /* masquer bit century */
    datetime->year   = bcd2dec(data[6]) + 2000;

    return true;
}


void rtc_manager_format_date(char *buffer, size_t size,
                              const datetime_t *datetime)
{
    if (!buffer || size == 0 || !datetime) return;

    snprintf(buffer, size,
             "%02d/%02d/%04d;%02d:%02d:%02d",
             datetime->day,    datetime->month,  datetime->year,
             datetime->hour,   datetime->minute, datetime->second);
            
}

bool rtc_manager_set_datetime(const datetime_t *datetime)
{
    if (!rtc_initialized || !datetime) return false;

    uint8_t data[7] = {
        dec2bcd(datetime->second),
        dec2bcd(datetime->minute),
        dec2bcd(datetime->hour),
        0x01,                            /* jour semaine : 1 par défaut      */
        dec2bcd(datetime->day),
        dec2bcd(datetime->month),
        dec2bcd((uint8_t)(datetime->year - 2000)),
    };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS3231_REG_SEC, true);
    i2c_master_write(cmd, data, 7, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur écriture RTC : %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "RTC réglé : %02d/%02d/%04d %02d:%02d:%02d",
             datetime->day, datetime->month, datetime->year,
             datetime->hour, datetime->minute, datetime->second);
    return true;
}