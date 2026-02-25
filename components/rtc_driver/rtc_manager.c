#include "rtc_manager.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "RTC";
static bool rtc_initialized = false;

// Adresse I2C du DS3231
#define DS3231_ADDR 0x68

// Registres DS3231
#define DS3231_REG_SEC 0x00
#define DS3231_REG_MIN 0x01
#define DS3231_REG_HOUR 0x02
#define DS3231_REG_DAY 0x03
#define DS3231_REG_DATE 0x04
#define DS3231_REG_MONTH 0x05
#define DS3231_REG_YEAR 0x06

// Convertir BCD en décimal
static uint8_t bcd2dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Convertir décimal en BCD
static uint8_t dec2bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

bool rtc_manager_init(int sda_pin, int scl_pin)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    
    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur configuration I2C: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur installation I2C: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Vérifier présence DS3231
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS3231 non détecté");
        return false;
    }
    
    rtc_initialized = true;
    ESP_LOGI(TAG, "RTC DS3231 initialisé");
    return true;
}

bool rtc_manager_get_datetime(datetime_t *datetime)
{
    if (!rtc_initialized || !datetime) return false;
    
    uint8_t data[7];
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, DS3231_REG_SEC, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur lecture RTC: %s", esp_err_to_name(ret));
        return false;
    }
    
    datetime->second = bcd2dec(data[0]);
    datetime->minute = bcd2dec(data[1]);
    datetime->hour = bcd2dec(data[2]);
    datetime->day = bcd2dec(data[4]);
    datetime->month = bcd2dec(data[5] & 0x1F);
    datetime->year = bcd2dec(data[6]) + 2000;
    
    return true;
}

void rtc_manager_format_date(char *buffer, size_t size, const datetime_t *datetime)
{
    snprintf(buffer, size, "%02d/%02d/%04d;%02d:%02d",
             datetime->day, datetime->month, datetime->year,
             datetime->hour, datetime->minute);
}