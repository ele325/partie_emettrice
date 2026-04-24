#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---------- Identification du nœud ---------- */
#define NODE_ID                 1
#define LORA_SYNC_WORD          0xF3
#define LORA_FREQUENCY          433E6
#define LORA_TX_POWER           18

/* ---------- Timing Deep Sleep ---------- */
#define SLEEP_BASE_SEC          (4 * 60 * 60)
#define SLEEP_BASE_US           ((uint64_t)SLEEP_BASE_SEC * 1000000ULL)
#define LORA_SLOT_SECONDS       (NODE_ID * 5)

/* ---------- GPIO - Bus SPI Partagé (LoRa + SD) ---------- */
#define PIN_SPI_SCK             12
#define PIN_SPI_MISO            13
#define PIN_SPI_MOSI            11

/* ---------- GPIO - LoRa SX1278 ---------- */
#define PIN_LORA_CS             10
#define PIN_LORA_RST            9
#define PIN_LORA_IRQ            14

/*
 * CORRECTION : IO34 est input-only sur ESP32-S2
 * Schéma montre NSS_SD sur IO34 → on le déplace sur IO35
 * OU utiliser IO38 si disponible
 * ⚠️  À adapter selon ton routage PCB réel
 */
#define PIN_SD_CS               5    /* PCB: NSS soudé sur IO34 (input-only) → re-câbler vers IO5 */

/* ---------- GPIO - Capteur BGT-SMPS (MODBUS RS485) ---------- */
#define PIN_MODBUS_RX           18
#define PIN_MODBUS_TX           17
#define PIN_MODBUS_EN           19
#define MODBUS_BAUD_RATE        9600
#define MODBUS_UART_PORT        UART_NUM_1

/* ---------- GPIO - I2C pour DS3231 ---------- */
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             26
#define I2C_PORT                I2C_NUM_0
#define I2C_FREQ_HZ             100000

/* ---------- Fichier de log SD ---------- */
#define SD_MOUNT_POINT          "/sdcard"
#define SD_LOG_FILE             "/sdcard/log.csv"
#define SD_LOG_HEADER \
    "Date;Heure;ID;Humidite(%);Temperature(C);pH;EC(uS/cm);N(mg/kg);P(mg/kg);K(mg/kg)\n"

#endif /* APP_CONFIG_H */
