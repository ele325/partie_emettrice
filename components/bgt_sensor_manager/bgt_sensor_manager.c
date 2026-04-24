/**
 * @file bgt_sensor_manager.c
 * @brief Driver NBL-S-TMC-7 — MAX485 + debug printf exhaustif
 *
 * CORRECTIONS v2 :
 *  1. g_baud initialisé à 9600 (valeur datasheet)
 *  2. txen_tx/rx délai 1ms → 5ms
 *  3. Timeout autodetect 1000ms → 1500ms
 *  4. Délai inter-essais 200ms → 1200ms (datasheet: min 1000ms)
 *  5. Printf debug à chaque étape critique
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
#define UART_BUF_SZ     512          /* augmenté de 256→512 */
#define RESP_MAX_SZ     32

/* Requête Modbus RTU — FC03, reg=0x0000, count=7 (CRC calculé dynamiquement).
 * NOTE: le CRC dépend des 6 premiers octets (dont l'adresse esclave). */
static const uint8_t REQUEST_BASE[6] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x07 };

static int     g_rx_pin      = -1;
static int     g_tx_pin      = -1;
static int     g_txen_pin    = -1;
static uint8_t g_addr        = 0x01;
static bool    g_initialized = false;
static int     g_baud        = 9600;  /* CORRECTION : était 4800 */

/* ─── CRC16 Modbus ───────────────────────────────────────────────── */
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

static void build_request_frame(uint8_t out[8])
{
    memcpy(out, REQUEST_BASE, sizeof(REQUEST_BASE));
    out[0] = g_addr;

    uint16_t crc = crc16(out, 6);
    out[6] = (uint8_t)(crc & 0xFF);        /* CRC low */
    out[7] = (uint8_t)((crc >> 8) & 0xFF); /* CRC high */
}

/* Vérifie le CRC de la trame de requête (debug) */
static void debug_print_request_crc(void)
{
    uint8_t frame[8];
    build_request_frame(frame);
    uint16_t crc = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);

    printf("[DBG][CRC] Trame requête CRC calculé : 0x%04X "
           "(low=0x%02X high=0x%02X)\n", crc, frame[6], frame[7]);
    printf("[DBG][CRC] Trame complète : "
           "%02X %02X %02X %02X %02X %02X %02X %02X\n",
           frame[0], frame[1], frame[2], frame[3],
           frame[4], frame[5], frame[6], frame[7]);
}

/* ─── TX_EN : contrôle direction MAX485 ─────────────────────────── */
static inline void txen_tx(void)
{
    if (g_txen_pin >= 0) {
        gpio_set_level((gpio_num_t)g_txen_pin, 1);
        printf("[DBG][MAX485] IO%d → HIGH (mode TX/émission)\n", g_txen_pin);
    } else {
        printf("[DBG][MAX485] WARN: txen_pin non configuré (-1) !\n");
    }
    vTaskDelay(pdMS_TO_TICKS(5));   /* CORRECTION : 1ms → 5ms */
}

static inline void txen_rx(void)
{
    if (g_txen_pin >= 0) {
        gpio_set_level((gpio_num_t)g_txen_pin, 0);
        printf("[DBG][MAX485] IO%d → LOW (mode RX/réception)\n", g_txen_pin);
    } else {
        printf("[DBG][MAX485] WARN: txen_pin non configuré (-1) !\n");
    }
    vTaskDelay(pdMS_TO_TICKS(5));   /* CORRECTION : 1ms → 5ms */
}

/* ─── (Re)configure UART à un baud donné ────────────────────────── */
static bool uart_set_baud(int baud)
{
    printf("[DBG][UART] Suppression driver UART%d...\n", UART_PORT_NUM);
    esp_err_t del_ret = uart_driver_delete(UART_PORT_NUM);
    printf("[DBG][UART] uart_driver_delete → %s (0x%X)\n",
           esp_err_to_name(del_ret), del_ret);

    const uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    printf("[DBG][UART] uart_driver_install(port=%d buf=%d)...\n",
           UART_PORT_NUM, UART_BUF_SZ);
    esp_err_t ret = uart_driver_install(UART_PORT_NUM,
                                        UART_BUF_SZ, 0, 0, NULL, 0);
    printf("[DBG][UART] uart_driver_install → %s (0x%X)\n",
           esp_err_to_name(ret), ret);
    if (ret != ESP_OK) return false;

    printf("[DBG][UART] uart_param_config baud=%d...\n", baud);
    ret = uart_param_config(UART_PORT_NUM, &cfg);
    printf("[DBG][UART] uart_param_config → %s (0x%X)\n",
           esp_err_to_name(ret), ret);
    if (ret != ESP_OK) return false;

    printf("[DBG][UART] uart_set_pin TX=IO%d RX=IO%d...\n",
           g_tx_pin, g_rx_pin);
    ret = uart_set_pin(UART_PORT_NUM, g_tx_pin, g_rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    printf("[DBG][UART] uart_set_pin → %s (0x%X)\n",
           esp_err_to_name(ret), ret);
    if (ret != ESP_OK) return false;

    ESP_LOGI(TAG, "Soil UART baud set to %d", baud);
    return true;
}

/* ─── Envoi trame + attente réponse ─────────────────────────────── */
static int uart_transact(const uint8_t *tx, size_t tx_len,
                         uint8_t *rx,  size_t rx_max,
                         int timeout_ms)
{
    /* Log TX hex */
    char hex[tx_len * 3 + 2];
    int off = 0;
    for (size_t i = 0; i < tx_len; i++)
        off += sprintf(&hex[off], "%02X ", tx[i]);
    if (off > 0) hex[off - 1] = '\0';
    ESP_LOGI(TAG, "Soil UART TX (%d): %s", (int)tx_len, hex);

    /* Vider buffer RX */
    printf("[DBG][TRANS] uart_flush_input...\n");
    uart_flush_input(UART_PORT_NUM);

    /* MAX485 → TX */
    txen_tx();

    /* Envoi */
    printf("[DBG][TRANS] uart_write_bytes (%d octets)...\n", (int)tx_len);
    int written = uart_write_bytes(UART_PORT_NUM, (const char *)tx, tx_len);
    printf("[DBG][TRANS] uart_write_bytes → %d octets écrits\n", written);
    if (written != (int)tx_len) {
        printf("[DBG][TRANS] ERREUR : seulement %d/%d octets envoyés !\n",
               written, (int)tx_len);
    }

    /* Attente fin TX */
    printf("[DBG][TRANS] uart_wait_tx_done (100ms)...\n");
    esp_err_t tx_ret = uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));
    printf("[DBG][TRANS] uart_wait_tx_done → %s\n", esp_err_to_name(tx_ret));

    /* MAX485 → RX */
    txen_rx();

    /* Lecture */
    printf("[DBG][TRANS] uart_read_bytes (max=%d timeout=%dms)...\n",
           (int)rx_max, timeout_ms);
    int len = uart_read_bytes(UART_PORT_NUM, rx, rx_max,
                              pdMS_TO_TICKS(timeout_ms));
    printf("[DBG][TRANS] uart_read_bytes → %d octets reçus\n", len);

    if (len <= 0) {
        ESP_LOGW(TAG, "Soil UART timeout, no bytes received");
        ESP_LOGW(TAG, "Soil UART read failed: %d", len);
        printf("[DBG][TRANS] DIAGNOSTIC TIMEOUT :\n");
        printf("  1. Vérifier alimentation capteur (5-24V sur fil rouge)\n");
        printf("  2. Vérifier A+ (jaune) / B- (bleu) non inversés\n");
        printf("  3. Vérifier IO%d=TX → DI du MAX485\n", g_tx_pin);
        printf("  4. Vérifier IO%d=RX ← RO du MAX485\n", g_rx_pin);
        printf("  5. Vérifier IO%d=TX_EN → DE+RE du MAX485\n", g_txen_pin);
        printf("  6. Mesurer tension sur A+/B- pendant l'envoi (~2-3V diff)\n");
        return len;
    }

    /* Log RX hex */
    char rxhex[len * 3 + 2];
    off = 0;
    for (int i = 0; i < len; i++)
        off += sprintf(&rxhex[off], "%02X ", rx[i]);
    if (off > 0) rxhex[off - 1] = '\0';
    ESP_LOGI(TAG, "Soil UART RX (%d): %s", len, rxhex);

    return len;
}

/* ─── Auto-détection baud ────────────────────────────────────────── */
static bool autodetect_baud(void)
{
    const int bauds[]  = {9600, 4800};
    const int nb_bauds = 2;

    printf("[DBG][AUTO] Début autodetect baud rate\n");
    printf("[DBG][AUTO] Adresse Modbus cible : 0x%02X\n", g_addr);
    debug_print_request_crc();

    for (int b = 0; b < nb_bauds; b++) {
        printf("[DBG][AUTO] ===== Essai baud %d =====\n", bauds[b]);
        ESP_LOGI(TAG, "Fixed request test at %d baud", bauds[b]);

        if (!uart_set_baud(bauds[b])) {
            printf("[DBG][AUTO] uart_set_baud(%d) ÉCHOUÉ\n", bauds[b]);
            continue;
        }

        /* Forcer RX avant envoi */
        printf("[DBG][AUTO] Forçage MAX485 en RX avant envoi\n");
        txen_rx();

        /* Délai stabilisation */
        printf("[DBG][AUTO] Délai stabilisation 50ms...\n");
        vTaskDelay(pdMS_TO_TICKS(50));

        uint8_t tx[8];
        build_request_frame(tx);

        uint8_t rx[RESP_MAX_SZ];
        memset(rx, 0, sizeof(rx));

        printf("[DBG][AUTO] Envoi trame Modbus (timeout=1500ms)...\n");
        int len = uart_transact(tx, sizeof(tx), rx, sizeof(rx), 1500);  /* CORRECTION : 1000→1500ms */

        printf("[DBG][AUTO] Réponse : %d octets\n", len);

        if (len > 4) {
            printf("[DBG][AUTO] rx[0]=0x%02X (attendu=0x%02X addr)\n",
                   rx[0], g_addr);
            printf("[DBG][AUTO] rx[1]=0x%02X (attendu=0x03 FC)\n", rx[1]);
        }

        if (len > 4 && rx[0] == g_addr && rx[1] == 0x03) {
            g_baud = bauds[b];
            printf("[DBG][AUTO] ✓ Capteur détecté à %d baud !\n", bauds[b]);
            ESP_LOGI(TAG, "Baud rate detected: %d", bauds[b]);
            return true;
        } else {
            printf("[DBG][AUTO] ✗ Pas de réponse valide à %d baud\n", bauds[b]);
            if (len > 0 && len <= 4) {
                printf("[DBG][AUTO]   Trame trop courte (%d<5) → "
                       "bruit ou mauvais baud\n", len);
            }
        }

        printf("[DBG][AUTO] Attente 1200ms avant prochain essai "
               "(datasheet: min 1000ms)...\n");
        vTaskDelay(pdMS_TO_TICKS(1200));  /* CORRECTION : 200→1200ms */
    }

    ESP_LOGE(TAG, "Capteur non détecté à 9600 ni 4800 baud");
    printf("[DBG][AUTO] ÉCHEC TOTAL — vérifications physiques requises\n");
    printf("[DBG][AUTO]   • Alimentation : 5V-24V entre fil rouge et noir\n");
    printf("[DBG][AUTO]   • RS485 A+ (jaune) → MAX485 broche A\n");
    printf("[DBG][AUTO]   • RS485 B- (bleu)  → MAX485 broche B\n");
    printf("[DBG][AUTO]   • ESP32 IO%d (TX)  → MAX485 DI\n", g_tx_pin);
    printf("[DBG][AUTO]   • ESP32 IO%d (RX)  ← MAX485 RO\n", g_rx_pin);
    printf("[DBG][AUTO]   • ESP32 IO%d (EN)  → MAX485 DE+RE\n", g_txen_pin);
    return false;
}

/* ─── Init publique ──────────────────────────────────────────────── */
bool bgt_sensor_manager_init(int rx_pin, int tx_pin, int de_re_pin,
                              int power_pin, uint8_t slave_addr)
{
    (void)power_pin;
    printf("[DBG][INIT] bgt_sensor_manager_init("
           "rx=%d tx=%d de_re=%d addr=0x%02X)\n",
           rx_pin, tx_pin, de_re_pin, slave_addr);

    g_rx_pin   = rx_pin;
    g_tx_pin   = tx_pin;
    g_txen_pin = de_re_pin;
    g_addr     = slave_addr;

    /* Vérifications de base */
    if (rx_pin < 0 || rx_pin > 46) {
        printf("[DBG][INIT] ERREUR : rx_pin=%d invalide !\n", rx_pin);
        return false;
    }
    if (tx_pin < 0 || tx_pin > 46) {
        printf("[DBG][INIT] ERREUR : tx_pin=%d invalide !\n", tx_pin);
        return false;
    }

    /* Configurer TX_EN */
    if (g_txen_pin >= 0) {
        printf("[DBG][INIT] Config GPIO TX_EN = IO%d en sortie...\n",
               g_txen_pin);
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << g_txen_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t gc_ret = gpio_config(&io);
        printf("[DBG][INIT] gpio_config TX_EN → %s\n",
               esp_err_to_name(gc_ret));

        gpio_set_level((gpio_num_t)g_txen_pin, 0);
        printf("[DBG][INIT] TX_EN = LOW (mode RX par défaut)\n");
        ESP_LOGI(TAG, "MAX485 TX_EN = IO%d (LOW=RX)", g_txen_pin);
    } else {
        printf("[DBG][INIT] ERREUR : de_re_pin=-1 → MAX485 non contrôlable !\n");
        printf("[DBG][INIT]   Sans contrôle DE/RE, le bus RS485 est bloqué\n");
    }

    /* Autodetection */
    printf("[DBG][INIT] Lancement autodetect_baud...\n");
    if (!autodetect_baud()) {
        printf("[DBG][INIT] autodetect_baud ÉCHOUÉ → init abandonnée\n");
        return false;
    }

    g_initialized = true;
    printf("[DBG][INIT] ✓ Init réussie — baud=%d\n", g_baud);
    return true;
}

/* ─── Lecture capteur ────────────────────────────────────────────── */
bool bgt_sensor_manager_read_all(bgt_sensor_data_t *data)
{
    printf("[DBG][READ] bgt_sensor_manager_read_all()\n");

    if (!g_initialized) {
        printf("[DBG][READ] ERREUR : driver non initialisé !\n");
        return false;
    }
    if (!data) {
        printf("[DBG][READ] ERREUR : pointeur data NULL !\n");
        return false;
    }

    /* Délai inter-trame obligatoire (datasheet: 1000ms min) */
    printf("[DBG][READ] Délai inter-trame 1100ms...\n");
    vTaskDelay(pdMS_TO_TICKS(1100));

    uint8_t rx[RESP_MAX_SZ];
    memset(rx, 0, sizeof(rx));

    uint8_t tx[8];
    build_request_frame(tx);

    printf("[DBG][READ] Envoi requête (timeout=2000ms)...\n");
    int len = uart_transact(tx, sizeof(tx), rx, sizeof(rx), 2000);

    printf("[DBG][READ] Réponse : %d octets\n", len);
    if (len <= 0) {
        printf("[DBG][READ] ÉCHEC lecture — pas de données\n");
        return false;
    }

    /* Vérifications trame */
    if (len < 7) {
        printf("[DBG][READ] ERREUR trame trop courte : %d < 7\n", len);
        ESP_LOGE(TAG, "Frame too short: %d bytes", len);
        return false;
    }

    printf("[DBG][READ] rx[0]=0x%02X (addr, attendu=0x%02X)\n",
           rx[0], g_addr);
    if (rx[0] != g_addr) {
        printf("[DBG][READ] ERREUR mauvaise adresse : 0x%02X != 0x%02X\n",
               rx[0], g_addr);
        ESP_LOGE(TAG, "Bad address: 0x%02X", rx[0]);
        return false;
    }

    printf("[DBG][READ] rx[1]=0x%02X (FC)\n", rx[1]);
    if (rx[1] == 0x83) {
        printf("[DBG][READ] ERREUR exception Modbus : code=0x%02X\n", rx[2]);
        printf("[DBG][READ]   0x01=Illegal function\n");
        printf("[DBG][READ]   0x02=Illegal data address\n");
        printf("[DBG][READ]   0x03=Illegal data value\n");
        ESP_LOGE(TAG, "Modbus exception: 0x%02X", rx[2]);
        return false;
    }
    if (rx[1] != 0x03) {
        printf("[DBG][READ] ERREUR mauvais FC : 0x%02X != 0x03\n", rx[1]);
        ESP_LOGE(TAG, "Bad FC: 0x%02X", rx[1]);
        return false;
    }

    /* Vérification CRC */
    uint16_t crc_calc = crc16(rx, len - 2);
    uint16_t crc_recv = (uint16_t)rx[len-2] | ((uint16_t)rx[len-1] << 8);
    printf("[DBG][READ] CRC calculé=0x%04X reçu=0x%04X %s\n",
           crc_calc, crc_recv,
           (crc_calc == crc_recv) ? "✓ OK" : "✗ ERREUR");
    if (crc_calc != crc_recv) {
        ESP_LOGE(TAG, "CRC error: calc=0x%04X recv=0x%04X",
                 crc_calc, crc_recv);
        return false;
    }

    /* Décodage registres */
    int byte_count = rx[2];
    int nb_regs    = byte_count / 2;
    printf("[DBG][READ] byte_count=%d → %d registres\n",
           byte_count, nb_regs);
    ESP_LOGI(TAG, "%d OF", nb_regs);

    if (nb_regs < 7) {
        printf("[DBG][READ] WARN : seulement %d/7 registres reçus\n",
               nb_regs);
    }

    uint16_t regs[7] = {0};
    for (int i = 0; i < nb_regs && i < 7; i++) {
        regs[i] = ((uint16_t)rx[3 + i*2] << 8) | rx[4 + i*2];
        printf("[DBG][READ] reg[%d] = 0x%04X (%d)\n", i, regs[i], regs[i]);
    }

    /* Décodage valeurs */
    data->temperature = (float)(int16_t)regs[0] / 10.0f;
    data->humidity    = (float)regs[1]           / 10.0f;
    data->ec          = (float)regs[2];
    data->ph          = (regs[3] == 0x7FFF)
                        ? -1.0f
                        : (float)regs[3] / 100.0f;
    data->nitrogen    = (nb_regs > 4) ? (float)regs[4] : 0.0f;
    data->phosphorus  = (nb_regs > 5) ? (float)regs[5] : 0.0f;
    data->potassium   = (nb_regs > 6) ? (float)regs[6] : 0.0f;

    printf("[DBG][READ] Décodage :\n");
    printf("  Temperature : reg=0x%04X → %.1f°C\n", regs[0], data->temperature);
    printf("  Humidity    : reg=0x%04X → %.1f%%\n",  regs[1], data->humidity);
    printf("  EC          : reg=0x%04X → %.0f µS/cm\n", regs[2], data->ec);
    printf("  pH          : reg=0x%04X → %.2f%s\n",  regs[3], data->ph,
           (regs[3] == 0x7FFF) ? " (invalide)" : "");
    printf("  N           : reg=0x%04X → %.0f mg/kg\n", regs[4], data->nitrogen);
    printf("  P           : reg=0x%04X → %.0f mg/kg\n", regs[5], data->phosphorus);
    printf("  K           : reg=0x%04X → %.0f mg/kg\n", regs[6], data->potassium);

    ESP_LOGI(TAG, "T=%.1f°C H=%.1f%% EC=%.0f pH=%.2f N=%.0f P=%.0f K=%.0f",
             data->temperature, data->humidity, data->ec, data->ph,
             data->nitrogen, data->phosphorus, data->potassium);

    return true;
}

/* ─── Format message LoRa ────────────────────────────────────────── */
void bgt_sensor_manager_format_message(uint8_t node_id,
                                       const bgt_sensor_data_t *data,
                                       char *buf, size_t size)
{
    if (!buf || size == 0) {
        printf("[DBG][FORMAT] ERREUR : buf=NULL ou size=0\n");
        return;
    }
    snprintf(buf, size,
             "ID:%d,H:%.1f,T:%.1f,EC:%.0f,PH:%.2f,N:%.0f,P:%.0f,K:%.0f",
             node_id,
             data->humidity,   data->temperature,
             data->ec,         data->ph,
             data->nitrogen,   data->phosphorus, data->potassium);
    printf("[DBG][FORMAT] Message formaté : %s\n", buf);
}
