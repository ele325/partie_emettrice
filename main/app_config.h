/* ================================================================
 * app_config.h - CONFIGURATION FINALE (ESP32-S2)
 * Capteur : BGT-SMPS MODBUS RS485
 * ================================================================ */

/* ---------- Identification du nœud ---------- */
#define NODE_ID                 1       // ← changer pour chaque carte : 1, 2, 3...
#define LORA_SYNC_WORD          0xF3
#define LORA_FREQUENCY          433E6
#define LORA_TX_POWER           18

/* ---------- Timing Deep Sleep ---------- */
// MODIFIÉ : on dort exactement 4 heures, sans offset
// L'offset est géré par la synchronisation RTC au réveil
#define SLEEP_BASE_SEC          (4 * 60 * 60)
#define SLEEP_BASE_US           ((uint64_t)SLEEP_BASE_SEC * 1000000ULL)

// Slot d'émission LoRa par nœud : NODE_ID × 5 secondes
// node 1 → attend la seconde :05 de la minute
// node 2 → attend la seconde :10 de la minute
// node 3 → attend la seconde :15 de la minute
// Max 11 nœuds (node 11 → :55s), au-delà réduire à NODE_ID * 3
#define LORA_SLOT_SECONDS       (NODE_ID * 5)

/* ---------- GPIO - Bus SPI Partagé (LoRa + SD) ---------- */
#define PIN_SPI_SCK             12
#define PIN_SPI_MISO            13
#define PIN_SPI_MOSI            11

/* ---------- GPIO - LoRa SX1278 ---------- */
#define PIN_LORA_CS             10
#define PIN_LORA_RST            9
#define PIN_LORA_IRQ            14

/* ---------- GPIO - Carte SD ---------- */
#define PIN_SD_CS               34

/* ---------- GPIO - Capteur BGT-SMPS (MODBUS RS485) ---------- */
#define PIN_MODBUS_RX           17
#define PIN_MODBUS_TX           18
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
#define SD_LOG_HEADER           "Date;Heure;ID;Humidite(%);Temperature(C)\n"