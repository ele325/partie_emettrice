/**
 * @file bgt_sensor_manager.c
 * @brief Driver NBL-S-TMC-7 / 7-in-1 Soil Sensor (Modbus RTU RS485)
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  Corrections v3 (basées sur datasheet + pinout RoboCare)        │
 * │                                                                 │
 * │  1. TX → IO18 (ligne 21), RX → IO17 (ligne 20)                 │
 * │  2. Alimentation 5V capteur via IO42 (ligne 36)                 │
 * │  3. SN65HVD485E intégré → DE/RE = -1                           │
 * │  4. VLA supprimés → buffers taille fixe                         │
 * │  5. Délai inter-trame ≥ 1000ms (datasheet) respecté            │
 * │  6. Facteurs de conversion conformes datasheet p.3 :            │
 * │       T / H  : ÷ 10   (276 → 27.6°C, 326 → 32.6%)             │
 * │       EC/NPK : brut   (75  → 75µS/cm, 8 → 8mg/kg)             │
 * │       pH     : ÷ 100  (722 → 7.22pH)                           │
 * │       pH = 0x7FFF → invalide (-1.0f)                           │
 * └─────────────────────────────────────────────────────────────────┘
 */

#include "bgt_sensor_manager.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "BGT_SENSOR";

/* ─── Constantes internes ────────────────────────────────────────────────── */
#define UART_PORT_NUM    UART_NUM_1
#define UART_BUF_SZ      256
#define RESP_MAX_SZ      32                    /* buffer réponse Modbus     */
#define HEX_BUF_SZ       (RESP_MAX_SZ * 3 + 1) /* "XX " × 32 + '\0'       */

/* ─── État interne ───────────────────────────────────────────────────────── */
static uint8_t  g_addr       = 0x01;
static int      g_de_re_pin  = -1;
static int      g_power_pin  = -1;
static bool     g_initialized = false;

/* ─── CRC16 Modbus ───────────────────────────────────────────────────────── */
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

/* ─── Gestion alimentation ───────────────────────────────────────────────── */
void bgt_sensor_manager_power_on(void)
{
    if (g_power_pin < 0) return;
    gpio_set_level((gpio_num_t)g_power_pin, 1);
    ESP_LOGI(TAG, "Alimentation 5V capteur ON (IO%d)", g_power_pin);
    vTaskDelay(pdMS_TO_TICKS(500));   /* Laisser le capteur démarrer       */
}

void bgt_sensor_manager_power_off(void)
{
    if (g_power_pin < 0) return;
    gpio_set_level((gpio_num_t)g_power_pin, 0);
    ESP_LOGI(TAG, "Alimentation 5V capteur OFF (IO%d)", g_power_pin);
}

/* ─── Initialisation ─────────────────────────────────────────────────────── */
bool bgt_sensor_manager_init(int rx_pin, int tx_pin, int de_re_pin,
                              int power_pin, uint8_t slave_addr)
{
    g_addr      = slave_addr;
    g_de_re_pin = de_re_pin;
    g_power_pin = power_pin;

    /* ── Broche alimentation 5V (IO42 sur carte RoboCare) ── */
    if (power_pin >= 0) {
        gpio_config_t pwr_cfg = {
            .pin_bit_mask = (1ULL << power_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&pwr_cfg);
        gpio_set_level((gpio_num_t)power_pin, 0);  /* éteint par défaut    */
        ESP_LOGI(TAG, "Broche alimentation 5V configurée : IO%d", power_pin);
    }

    /* ── Broche DE/RE (externe MAX485 uniquement) ── */
    if (de_re_pin >= 0) {
        gpio_config_t de_cfg = {
            .pin_bit_mask = (1ULL << de_re_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&de_cfg);
        gpio_set_level((gpio_num_t)de_re_pin, 0);  /* réception par défaut */
        ESP_LOGI(TAG, "Broche DE/RE configurée : IO%d", de_re_pin);
    }

    /* ── UART1 ── */
    uart_driver_delete(UART_PORT_NUM);   /* reset propre si déjà installé   */

    uart_config_t uart_cfg = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_driver_install(UART_PORT_NUM, UART_BUF_SZ,
                                        0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur install UART: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_param_config(UART_PORT_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur config UART: %s", esp_err_to_name(ret));
        return false;
    }

    /* TX = tx_pin, RX = rx_pin, pas de RTS/CTS */
    ret = uart_set_pin(UART_PORT_NUM, tx_pin, rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur set pins UART: %s", esp_err_to_name(ret));
        return false;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "────────────────────────────────────────");
    ESP_LOGI(TAG, "  Capteur NBL-S-TMC-7 initialisé");
    ESP_LOGI(TAG, "  TX=IO%d  RX=IO%d  DE/RE=%d  PWR=IO%d",
             tx_pin, rx_pin, de_re_pin, power_pin);
    ESP_LOGI(TAG, "  Adresse Modbus : 0x%02X  Baud : 9600,8,N,1", slave_addr);
    ESP_LOGI(TAG, "────────────────────────────────────────");

    /* Allumer le capteur si la broche d'alimentation est configurée */
    bgt_sensor_manager_power_on();

    return true;
}

/* ─── Lecture Modbus ─────────────────────────────────────────────────────── */
bool bgt_sensor_manager_read_all(bgt_sensor_data_t *data)
{
    if (!g_initialized || !data) {
        ESP_LOGE(TAG, "Capteur non initialisé ou pointeur NULL");
        return false;
    }

    /* Vider le buffer RX avant d'envoyer */
    uart_flush_input(UART_PORT_NUM);

    /* ── Construire la trame Modbus ──
     * Datasheet p.3 :  Send: 01 03 00 00 00 07 44 0C
     * Addr | FC | RegH | RegL | NbH | NbL | CRC_L | CRC_H
     */
    uint8_t request[8] = {
        g_addr,         /* 0x01  adresse esclave                          */
        0x03,           /* Read Holding Registers                          */
        0x00, 0x00,     /* Registre de départ : 0x0000 (Temperature)      */
        0x00, 0x07,     /* Nombre de registres : 7                         */
        0x00, 0x00      /* CRC calculé ci-dessous                          */
    };
    uint16_t crc = crc16(request, 6);
    request[6] = (uint8_t)(crc & 0xFF);
    request[7] = (uint8_t)(crc >> 8);

    ESP_LOGI(TAG, ">>> %02X %02X %02X %02X %02X %02X %02X %02X",
             request[0], request[1], request[2], request[3],
             request[4], request[5], request[6], request[7]);

    /* ── Activer émission si DE/RE externe ── */
    if (g_de_re_pin >= 0) {
        gpio_set_level((gpio_num_t)g_de_re_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    uart_write_bytes(UART_PORT_NUM, (const char *)request, 8);
    uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));

    /* ── Repasser en réception ── */
    if (g_de_re_pin >= 0) {
        gpio_set_level((gpio_num_t)g_de_re_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    /* ── Lire la réponse ──
     * Format attendu : Addr FC ByteCount [14 octets data] CRC_L CRC_H
     * Taille totale  : 1+1+1+14+2 = 19 octets
     * Datasheet p.3 : 01 03 0E 01 14 01 46 00 4B 02 D2 00 08 00 08 00 15 F8 0E
     */
    uint8_t response[RESP_MAX_SZ];
    memset(response, 0, sizeof(response));
    int len = uart_read_bytes(UART_PORT_NUM, response, sizeof(response),
                              pdMS_TO_TICKS(1000));

    /* ── Log HEX brut ── */
    if (len > 0) {
        char hex[HEX_BUF_SZ];
        int  off = 0;
        for (int i = 0; i < len && off < (int)sizeof(hex) - 3; i++) {
            off += sprintf(&hex[off], "%02X ", response[i]);
        }
        hex[off] = '\0';
        ESP_LOGI(TAG, "<<< %s(%d bytes)", hex, len);
    } else {
        ESP_LOGE(TAG, "Timeout — aucune réponse du capteur");
        return false;
    }

    /* ── Vérifications structurelles ── */
    if (len < 19) {
        ESP_LOGE(TAG, "Réponse trop courte : %d bytes (attendu 19)", len);
        return false;
    }
    if (response[0] != g_addr) {
        ESP_LOGE(TAG, "Adresse incorrecte : reçu 0x%02X attendu 0x%02X",
                 response[0], g_addr);
        return false;
    }
    if (response[1] != 0x03) {
        if (response[1] == 0x83) {
            ESP_LOGE(TAG, "Exception Modbus ! Code : 0x%02X", response[2]);
        } else {
            ESP_LOGE(TAG, "Code fonction inattendu : 0x%02X", response[1]);
        }
        return false;
    }
    if (response[2] != 0x0E) {
        ESP_LOGW(TAG, "ByteCount inattendu : %d (attendu 14=0x0E)", response[2]);
    }

    /* ── Vérification CRC ── */
    uint16_t crc_calc = crc16(response, len - 2);
    uint16_t crc_recv = (uint16_t)response[len - 2]
                      | ((uint16_t)response[len - 1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGE(TAG, "CRC invalide ! calculé=0x%04X reçu=0x%04X",
                 crc_calc, crc_recv);
        return false;
    }
    ESP_LOGI(TAG, "CRC OK ✓");

    /* ── Extraction des 7 registres big-endian ──
     *
     * Datasheet exemple (p.3) :
     *  01 03 0E | 01 14 | 01 46 | 00 4B | 02 D2 | 00 08 | 00 08 | 00 15 | CRC
     *             T°C     H%      EC      pH      N       P       K
     *
     *  byte[3..16] = 14 octets de données = 7 registres × 2 octets
     */
    for (int i = 0; i < 7; i++) {
        uint16_t raw = ((uint16_t)response[3 + i * 2] << 8)
                     |  (uint16_t)response[4 + i * 2];

        switch (i) {
            case 0:  /* Temperature — 0x0000 — ÷10 → °C */
                data->temperature = raw / 10.0f;
                break;
            case 1:  /* Moisture — 0x0001 — ÷10 → % */
                data->humidity = raw / 10.0f;
                break;
            case 2:  /* Conductivity — 0x0002 — brut → µS/cm */
                data->ec = (float)raw;
                break;
            case 3:  /* PH — 0x0003 — ÷100 → pH */
                if (raw == 0x7FFF) {
                    ESP_LOGW(TAG, "pH invalide (0x7FFF) : probe non connectée ?");
                    data->ph = -1.0f;
                } else {
                    data->ph = raw / 100.0f;
                }
                break;
            case 4:  /* Nitrogen — 0x0004 — brut → mg/kg */
                data->nitrogen = (float)raw;
                break;
            case 5:  /* Phosphorus — 0x0005 — brut → mg/kg */
                data->phosphorus = (float)raw;
                break;
            case 6:  /* Potassium — 0x0006 — brut → mg/kg */
                data->potassium = (float)raw;
                break;
        }
    }

    ESP_LOGD(TAG, "T=%.1f°C H=%.1f%% EC=%.0fµS/cm pH=%.2f N=%.0f P=%.0f K=%.0f",
             data->temperature, data->humidity, data->ec, data->ph,
             data->nitrogen, data->phosphorus, data->potassium);

    return true;
}

/* ─── Formatage LoRa ─────────────────────────────────────────────────────── */
void bgt_sensor_manager_format_message(uint8_t node_id,
                                       const bgt_sensor_data_t *data,
                                       char *buffer, size_t size)
{
    if (!buffer || size == 0) return;
    snprintf(buffer, size,
             "ID:%d,H:%.1f,T:%.1f,EC:%.0f,PH:%.2f,N:%.0f,P:%.0f,K:%.0f",
             node_id,
             data->humidity,   data->temperature,
             data->ec,         data->ph,
             data->nitrogen,   data->phosphorus,  data->potassium);
}