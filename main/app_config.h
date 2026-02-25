#pragma once

/* ================================================================
 * app_config.h
 * Configuration centralisée du nœud capteur
 * ESP32-S2 | SX1278 (433MHz) | DS3231 | Capteur Humidité
 * ================================================================ */

/* ---------- Identification du nœud ---------- */
#define NODE_ID                 1
#define LORA_SYNC_WORD          0xF3
#define LORA_FREQUENCY          433E6   // 433 MHz
#define LORA_TX_POWER           18      // dBm (max SX1278)

/* ---------- Timing Deep Sleep ---------- */
#define SLEEP_DURATION_SEC      (4 * 60 * 60)   // 4 heures en secondes
#define SLEEP_DURATION_US       ((uint64_t)SLEEP_DURATION_SEC * 1000000ULL)

/* ---------- GPIO - Bus SPI Partagé (LoRa + SD) ---------- */
#define PIN_SPI_SCK             7
#define PIN_SPI_MISO            9
#define PIN_SPI_MOSI            11

/* ---------- GPIO - LoRa SX1278 ---------- */
#define PIN_LORA_CS             10
#define PIN_LORA_RST            6
#define PIN_LORA_IRQ            5

/* ---------- GPIO - Carte SD ---------- */
#define PIN_SD_CS               12

/* ---------- GPIO - Capteur Humidité ---------- */
#define PIN_SENSOR_ADC          ADC1_CHANNEL_0  // GPIO 1 sur ESP32-S2
#define ADC_RAW_DRY             4095            // Sol complètement sec
#define ADC_RAW_WET             1200            // Sol complètement humide

/* ---------- GPIO - I2C pour DS3231 ---------- */
#define PIN_I2C_SDA             3
#define PIN_I2C_SCL             4
#define I2C_PORT                I2C_NUM_0
#define I2C_FREQ_HZ             100000

/* ---------- Fichier de log SD ---------- */
#define SD_MOUNT_POINT          "/sdcard"
#define SD_LOG_FILE             "/sdcard/log.csv"
#define SD_LOG_HEADER           "Date;Heure;Zone;Humidite(%)\n"
