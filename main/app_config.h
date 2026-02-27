#pragma once

/* ================================================================
 * app_config.h - CONFIGURATION FINALE (ESP32-S2)
 * Capteur : BGT-SMPS MODBUS RS485
 * ================================================================ */

/* ---------- Identification du nœud ---------- */
#define NODE_ID                 1
#define LORA_SYNC_WORD          0xF3
#define LORA_FREQUENCY          433E6   // 433 MHz
#define LORA_TX_POWER           18      // dBm

/* ---------- Timing Deep Sleep ---------- */
#define SLEEP_DURATION_SEC      (4 * 60 * 60)   // 4 heures
#define SLEEP_DURATION_US       ((uint64_t)SLEEP_DURATION_SEC * 1000000ULL)

/* ---------- GPIO - Bus SPI Partagé (LoRa + SD) ---------- */
#define PIN_SPI_SCK             12  // OK : CLK sur schéma
#define PIN_SPI_MISO            13  // OK : MISO sur schéma
#define PIN_SPI_MOSI            11  // OK : MOSI sur schéma

/* ---------- GPIO - LoRa SX1278 ---------- */
#define PIN_LORA_CS             14  // OK : NSS sur schéma
#define PIN_LORA_RST            10  // OK : Reset selon CSV
#define PIN_LORA_IRQ            5   // OK : Interruption

/* ---------- GPIO - Carte SD ---------- */
#define PIN_SD_CS               34  // OK : CS Carte_SD sur schéma

/* ---------- GPIO - Capteur BGT-SMPS (MODBUS RS485) ---------- */
#define PIN_MODBUS_RX           17  // IO17 (RX Connecteur)
#define PIN_MODBUS_TX           18  // IO18 (TX Connecteur)
#define PIN_MODBUS_EN           19  // IO19 (Contrôle DE/RE pour RS485)
#define MODBUS_BAUD_RATE        9600
#define MODBUS_UART_PORT        UART_NUM_1

/* ---------- GPIO - I2C pour DS3231 ---------- */
#define PIN_I2C_SDA             21  // OK : SDA sur schéma
#define PIN_I2C_SCL             26  // OK : SCL sur schéma
#define I2C_PORT                I2C_NUM_0
#define I2C_FREQ_HZ             100000

/* ---------- Fichier de log SD ---------- */
#define SD_MOUNT_POINT          "/sdcard"
#define SD_LOG_FILE             "/sdcard/log.csv"
#define SD_LOG_HEADER           "Date;Heure;ID;Humidite(%);Temperature(C)\n"