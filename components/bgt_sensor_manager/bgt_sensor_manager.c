/**
 * @file bgt_sensor_manager.c
 * @brief Driver NBL-S-TMC-7 — MAX485 (TX_EN=IO19) + auto baud 9600→4800
 *
 * Logs reproduits (identiques aux captures) :
 *   I emetteur_sensor: Fixed request test at 9600 baud
 *   I emetteur_sensor: Soil UART baud set to 9600
 *   I emetteur_sensor: Soil UART TX (8): 01 03 00 00 00 07 44 0C
 *   W emetteur_sensor: Soil UART timeout, no bytes received
 *   W emetteur_sensor: Soil UART read failed: -3
 *   I emetteur_sensor: Fixed request test at 4800 baud
 *   I emetteur_sensor: Soil UART baud set to 4800
 *   I emetteur_sensor: Soil UART TX (8): 01 03 00 00 00 07 44 0C
 *   I emetteur_sensor: Soil UART RX (13): 01 03 08 ...
 *   I emetteur_sensor: 4 OF
 *
 * MAX485 — TX_EN (RE+DE combinés) :
 *   HIGH → mode émission  (ESP32 envoie vers capteur)
 *   LOW  → mode réception (ESP32 reçoit depuis capteur)
 */

#include "bgt_sensor_manager.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "emetteur_sensor";

#define UART_PORT_NUM   UART_NUM_1
#define UART_BUF_SZ     256
#define RESP_MAX_SZ     32

/* Trame Modbus RTU — FC03, addr=0x01, reg=0x0000, count=7 */
static const uint8_t REQUEST_FRAME[8] = {
    0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x44, 0x0C
};

static int     g_rx_pin    = -1;
static int     g_tx_pin    = -1;
static int     g_txen_pin  = -1;   /* TX_EN = RE+DE combinés MAX485 */
static uint8_t g_addr      = 0x01;
static bool    g_initialized = false;
static int     g_baud      = 4800;

/* ─── CRC16 Modbus ───────────────────────────────────────────────────── */
static uint16_t crc16(const uint8_t *d, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

/* ─── TX_EN : contrôle direction MAX485 ─────────────────────────────── */
static inline void txen_tx(void)   /* passe en émission */
{
    if (g_txen_pin >= 0)
        gpio_set_level((gpio_num_t)g_txen_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static inline void txen_rx(void)   /* passe en réception */
{
    if (g_txen_pin >= 0)
        gpio_set_level((gpio_num_t)g_txen_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/* ─── (Re)configure UART à un baud donné ────────────────────────────── */
static bool uart_set_baud(int baud)
{
    uart_driver_delete(UART_PORT_NUM);

    const uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    if (uart_driver_install(UART_PORT_NUM, UART_BUF_SZ, 0, 0, NULL, 0)
            != ESP_OK) return false;
    if (uart_param_config(UART_PORT_NUM, &cfg) != ESP_OK) return false;
    if (uart_set_pin(UART_PORT_NUM, g_tx_pin, g_rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
        return false;

    ESP_LOGI(TAG, "Soil UART baud set to %d", baud);
    return true;
}

/* ─── Envoi trame + attente réponse ─────────────────────────────────── */
static int uart_transact(const uint8_t *tx, size_t tx_len,
                         uint8_t *rx,  size_t rx_max,
                         int timeout_ms)
{
    /* Log TX */
    char hex[tx_len * 3 + 2];
    int off = 0;
    for (size_t i = 0; i < tx_len; i++)
        off += sprintf(&hex[off], "%02X ", tx[i]);
    if (off > 0) hex[off - 1] = '\0';
    ESP_LOGI(TAG, "Soil UART TX (%d): %s", (int)tx_len, hex);

    /* Vider le buffer RX avant émission */
    uart_flush_input(UART_PORT_NUM);

    /* MAX485 → mode TX */
    txen_tx();

    uart_write_bytes(UART_PORT_NUM, (const char *)tx, tx_len);
    uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));

    /* MAX485 → mode RX dès que l'émission est terminée */
    txen_rx();

    /* Lecture réponse */
    int len = uart_read_bytes(UART_PORT_NUM, rx, rx_max,
                              pdMS_TO_TICKS(timeout_ms));

    if (len <= 0) {
        ESP_LOGW(TAG, "Soil UART timeout, no bytes received");
        ESP_LOGW(TAG, "Soil UART read failed: %d", len);
        return len;
    }

    /* Log RX */
    char rxhex[len * 3 + 2];
    off = 0;
    for (int i = 0; i < len; i++)
        off += sprintf(&rxhex[off], "%02X ", rx[i]);
    if (off > 0) rxhex[off - 1] = '\0';
    ESP_LOGI(TAG, "Soil UART RX (%d): %s", len, rxhex);

    return len;
}

/* ─── Auto-détection baud : 9600 d'abord, puis 4800 ────────────────── */
static bool autodetect_baud(void)
{
    const int bauds[]   = {9600, 4800};
    const int nb_bauds  = 2;

    for (int b = 0; b < nb_bauds; b++) {
        ESP_LOGI(TAG, "Fixed request test at %d baud", bauds[b]);

        if (!uart_set_baud(bauds[b]))
            continue;

        /* Repasser en RX avant d'envoyer */
        txen_rx();

        uint8_t rx[RESP_MAX_SZ];
        memset(rx, 0, sizeof(rx));

        int len = uart_transact(REQUEST_FRAME, sizeof(REQUEST_FRAME),
                                rx, sizeof(rx), 1000);

        if (len > 4 && rx[0] == g_addr && rx[1] == 0x03) {
            g_baud = bauds[b];
            ESP_LOGI(TAG, "Baud rate detected: %d", bauds[b]);
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGE(TAG, "Capteur non détecté à 9600 ni 4800 baud");
    return false;
}

/* ─── Init publique ──────────────────────────────────────────────────── */
bool bgt_sensor_manager_init(int rx_pin, int tx_pin, int de_re_pin,
                              int power_pin, uint8_t slave_addr)
{
    (void)power_pin;
    g_rx_pin   = rx_pin;
    g_tx_pin   = tx_pin;
    g_txen_pin = de_re_pin;   /* IO19 = TX_EN du MAX485 */
    g_addr     = slave_addr;

    /* Configurer TX_EN en sortie, LOW par défaut (mode RX) */
    if (g_txen_pin >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << g_txen_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        gpio_set_level((gpio_num_t)g_txen_pin, 0);  /* RX par défaut */
        ESP_LOGI(TAG, "MAX485 TX_EN = IO%d (LOW=RX)", g_txen_pin);
    }

    if (!autodetect_baud())
        return false;

    g_initialized = true;
    return true;
}

/* ─── Lecture capteur ────────────────────────────────────────────────── */
bool bgt_sensor_manager_read_all(bgt_sensor_data_t *data)
{
    if (!g_initialized || !data) return false;

    uint8_t rx[RESP_MAX_SZ];
    memset(rx, 0, sizeof(rx));

    int len = uart_transact(REQUEST_FRAME, sizeof(REQUEST_FRAME),
                            rx, sizeof(rx), 2000);

    if (len <= 0) return false;

    /* Vérification trame minimale */
    if (len < 7) {
        ESP_LOGE(TAG, "Frame too short: %d bytes", len);
        return false;
    }
    if (rx[0] != g_addr) {
        ESP_LOGE(TAG, "Bad address: 0x%02X", rx[0]);
        return false;
    }
    if (rx[1] == 0x83) {
        ESP_LOGE(TAG, "Modbus exception: 0x%02X", rx[2]);
        return false;
    }
    if (rx[1] != 0x03) {
        ESP_LOGE(TAG, "Bad FC: 0x%02X", rx[1]);
        return false;
    }

    /* Vérification CRC */
    uint16_t crc_calc = crc16(rx, len - 2);
    uint16_t crc_recv = (uint16_t)rx[len-2] | ((uint16_t)rx[len-1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGE(TAG, "CRC error: calc=0x%04X recv=0x%04X",
                 crc_calc, crc_recv);
        return false;
    }

    /* Nombre de registres reçus */
    int nb_regs = rx[2] / 2;
    ESP_LOGI(TAG, "%d OF", nb_regs);   /* reproduit "4 OF" de la capture */

    uint16_t regs[7] = {0};
    for (int i = 0; i < nb_regs && i < 7; i++)
        regs[i] = ((uint16_t)rx[3 + i*2] << 8) | rx[4 + i*2];

    /*
     * Ordre registres — datasheet NBL-S-TMC-7 page 3 :
     *   reg[0] = Temperature  ÷10  → °C  (signé int16_t)
     *   reg[1] = Moisture     ÷10  → %
     *   reg[2] = Conductivity brut → µS/cm
     *   reg[3] = PH           ÷100 → pH  (0x7FFF = invalide)
     *   reg[4] = N            brut → mg/kg
     *   reg[5] = P            brut → mg/kg
     *   reg[6] = K            brut → mg/kg
     */
    data->temperature = (float)(int16_t)regs[0] / 10.0f;
    data->humidity    = (float)regs[1] / 10.0f;
    data->ec          = (float)regs[2];
    data->ph          = (regs[3] == 0x7FFF) ? -1.0f
                                             : (float)regs[3] / 100.0f;
    data->nitrogen    = (nb_regs > 4) ? (float)regs[4] : 0.0f;
    data->phosphorus  = (nb_regs > 5) ? (float)regs[5] : 0.0f;
    data->potassium   = (nb_regs > 6) ? (float)regs[6] : 0.0f;

    ESP_LOGI(TAG, "T=%.1f°C H=%.1f%% EC=%.0f pH=%.2f N=%.0f P=%.0f K=%.0f",
             data->temperature, data->humidity, data->ec, data->ph,
             data->nitrogen, data->phosphorus, data->potassium);

    return true;
}

/* ─── Format message LoRa ────────────────────────────────────────────── */
void bgt_sensor_manager_format_message(uint8_t node_id,
                                       const bgt_sensor_data_t *data,
                                       char *buf, size_t size)
{
    if (!buf || size == 0) return;
    snprintf(buf, size,
             "ID:%d,H:%.1f,T:%.1f,EC:%.0f,PH:%.2f,N:%.0f,P:%.0f,K:%.0f",
             node_id,
             data->humidity,   data->temperature,
             data->ec,         data->ph,
             data->nitrogen,   data->phosphorus, data->potassium);
}