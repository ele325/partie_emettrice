/**
 * @file main.c
 * @brief Test DIAGNOSTIQUE complet — Capteur Sol NBL-S-TMC-7
 *        Projet RoboCare — Carte Émettrice ESP32-S2-WROOM-I
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  PinOut confirmé sur image A.5 de la carte émettrice            │
 * │                                                                 │
 * │  Ligne 20 : RX_UART CAPTEUR   → IO17  (IN/OUT)                 │
 * │  Ligne 21 : TX_UART CAPTEUR   → IO18  (IN/OUT)  ← pas IO20 !  │
 * │  Ligne 36 : OFF POWER 5V      → IO42  (OUTPUT)                 │
 * │                                                                 │
 * │  Transceiver embarqué : SN65HVD485E                             │
 * │    → DE/RE géré en interne : passer de_re_pin = -1              │
 * │                                                                 │
 * │  Câblage capteur NBL-S-TMC-7 (datasheet p.2) :                  │
 * │    Rouge  → 5V (IO42 ou alim externe 5-24V)                    │
 * │    Noir   → GND                                                 │
 * │    Jaune  → RS485 A+  (borne A+ du transceiver)                │
 * │    Bleu   → RS485 B-  (borne B- du transceiver)                │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * ⚠️  ALIMENTATION CAPTEUR : 5V à 24V OBLIGATOIRE (jamais 3.3V !)
 *     → IO42 active la ligne "OFF POWER / ROLLING 5V POWER"
 *     → Sans cette alimentation = timeout systématique
 *
 * CE QUE CE TEST FAIT :
 *   1. Configure IO42 en sortie et active l'alim 5V capteur
 *   2. Initialise l'UART1 : TX=IO18, RX=IO17, 9600,8,N,1
 *   3. Envoie la trame Modbus : 01 03 00 00 00 07 44 0C
 *   4. Affiche la réponse brute HEX
 *   5. Vérifie CRC, décode et affiche les 7 paramètres
 *   6. Répète toutes les 2 secondes (≥ 1000ms requis par datasheet)
 *
 * COMPILATION ESP-IDF :
 *   idf.py build && idf.py -p /dev/ttyUSBx flash monitor
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "ROBOCARE_CAPTEUR_SOL";

/* ══════════════════════════════════════════════════════════════════
 *  CONFIGURATION — à ajuster si nécessaire
 * ══════════════════════════════════════════════════════════════════ */

/* Broches UART capteur — confirmées sur pinout A.5 */
#define SENSOR_RX_PIN      17    /* IO17 — Ligne 20 : RX_UART CAPTEUR    */
#define SENSOR_TX_PIN      18    /* IO18 — Ligne 21 : TX_UART CAPTEUR    */
#define SENSOR_DE_RE_PIN   (-1)  /* SN65HVD485E intégré → pas de DE/RE  */

/* Alimentation 5V capteur — IO42 Ligne 36 "OFF POWER / ROLLING 5V" */
#define SENSOR_POWER_PIN   42    /* OUTPUT — mettre -1 si alim externe   */

/* Paramètres Modbus */
#define SENSOR_ADDR        0x01  /* Adresse usine par défaut             */
#define UART_PORT          UART_NUM_1
#define UART_BUF_SIZE      256

/* Tailles buffers (pas de VLA) */
#define RESP_MAX_LEN       32
#define HEX_BUF_LEN        (RESP_MAX_LEN * 3 + 1)

/* ══════════════════════════════════════════════════════════════════
 *  CRC16 Modbus
 * ══════════════════════════════════════════════════════════════════ */
static uint16_t crc16_modbus(const uint8_t *data, size_t len)
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

/* ══════════════════════════════════════════════════════════════════
 *  Alimentation 5V capteur
 * ══════════════════════════════════════════════════════════════════ */
static void sensor_power_init(void)
{
#if SENSOR_POWER_PIN >= 0
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << SENSOR_POWER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(SENSOR_POWER_PIN, 1);   /* Activer 5V */
    ESP_LOGI(TAG, "✓ Alimentation 5V capteur activée (IO%d)", SENSOR_POWER_PIN);
    vTaskDelay(pdMS_TO_TICKS(500));        /* Délai stabilisation capteur  */
#else
    ESP_LOGI(TAG, "Alimentation capteur externe (pas de broche configurée)");
#endif
}

/* ══════════════════════════════════════════════════════════════════
 *  Initialisation UART
 * ══════════════════════════════════════════════════════════════════ */
static bool uart_sensor_init(void)
{
    uart_driver_delete(UART_PORT);   /* reset propre */

    uart_config_t cfg = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_driver_install(UART_PORT, UART_BUF_SIZE,
                                        0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur install UART: %s", esp_err_to_name(ret));
        return false;
    }
    ret = uart_param_config(UART_PORT, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur config UART: %s", esp_err_to_name(ret));
        return false;
    }
    ret = uart_set_pin(UART_PORT,
                       SENSOR_TX_PIN, SENSOR_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur set pins UART: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "✓ UART1 initialisé — TX=IO%d  RX=IO%d  9600,8,N,1",
             SENSOR_TX_PIN, SENSOR_RX_PIN);
    return true;
}

/* ══════════════════════════════════════════════════════════════════
 *  Envoi trame Modbus + lecture réponse
 * ══════════════════════════════════════════════════════════════════ */
static int modbus_request(uint8_t *response, size_t resp_size)
{
    /* Trame Modbus RTU : 01 03 00 00 00 07 44 0C
     * Conforme datasheet p.3 — lecture des 7 registres depuis 0x0000 */
    uint8_t request[8] = {
        SENSOR_ADDR,    /* 0x01  adresse esclave                          */
        0x03,           /* Read Holding Registers                          */
        0x00, 0x00,     /* Registre départ : 0x0000 (Temperature)         */
        0x00, 0x07,     /* Nombre de registres : 7                         */
        0x00, 0x00      /* CRC                                             */
    };
    uint16_t crc = crc16_modbus(request, 6);
    request[6] = (uint8_t)(crc & 0xFF);
    request[7] = (uint8_t)(crc >> 8);

    ESP_LOGI(TAG, ">>> %02X %02X %02X %02X %02X %02X %02X %02X",
             request[0], request[1], request[2], request[3],
             request[4], request[5], request[6], request[7]);

    uart_flush_input(UART_PORT);
    uart_write_bytes(UART_PORT, (const char *)request, 8);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));

    /* Timeout 1000ms — datasheet : intervalle max entre 2 communications */
    return uart_read_bytes(UART_PORT, response, resp_size,
                           pdMS_TO_TICKS(1000));
}

/* ══════════════════════════════════════════════════════════════════
 *  Décodage et affichage
 * ══════════════════════════════════════════════════════════════════ */
static void decode_and_print(const uint8_t *resp, int len)
{
    /* Affichage HEX brut */
    char hex[HEX_BUF_LEN];
    int  off = 0;
    for (int i = 0; i < len && off < (int)sizeof(hex) - 3; i++) {
        off += sprintf(&hex[off], "%02X ", resp[i]);
    }
    hex[off] = '\0';
    ESP_LOGI(TAG, "<<< Brut (%d bytes): %s", len, hex);

    /* ── Vérifications minimales ── */
    if (len < 19) {
        ESP_LOGE(TAG, "Réponse trop courte : %d bytes (attendu 19)", len);
        return;
    }
    if (resp[0] != SENSOR_ADDR) {
        ESP_LOGE(TAG, "Adresse incorrecte : reçu 0x%02X attendu 0x%02X",
                 resp[0], SENSOR_ADDR);
        return;
    }
    if (resp[1] != 0x03) {
        if (resp[1] == 0x83)
            ESP_LOGE(TAG, "Exception Modbus ! Code erreur : 0x%02X", resp[2]);
        else
            ESP_LOGE(TAG, "Code fonction inattendu : 0x%02X", resp[1]);
        return;
    }
    if (resp[2] != 0x0E) {
        ESP_LOGW(TAG, "ByteCount=%d (attendu 14=0x0E)", resp[2]);
    }

    /* ── Vérification CRC ── */
    uint16_t crc_calc = crc16_modbus(resp, len - 2);
    uint16_t crc_recv = (uint16_t)resp[len - 2]
                      | ((uint16_t)resp[len - 1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGE(TAG, "CRC invalide ! calculé=0x%04X reçu=0x%04X",
                 crc_calc, crc_recv);
        return;
    }
    ESP_LOGI(TAG, "CRC OK ✓");

    /* ── Extraction 7 registres big-endian ──
     *
     * Datasheet p.3 example :
     *  01 03 0E | 01 14 | 01 46 | 00 4B | 02 D2 | 00 08 | 00 08 | 00 15 | CRC
     *             T=276   H=326   EC=75   pH=722  N=8     P=8     K=21
     *
     * Conversions :
     *   T   = raw ÷ 10    → 276  → 27.6°C
     *   H   = raw ÷ 10    → 326  → 32.6%
     *   EC  = raw brut    → 75   → 75µS/cm
     *   pH  = raw ÷ 100   → 722  → 7.22pH   (0x7FFF = invalide)
     *   N   = raw brut    → 8    → 8mg/kg
     *   P   = raw brut    → 8    → 8mg/kg
     *   K   = raw brut    → 21   → 21mg/kg
     */
    uint16_t raw[7];
    for (int i = 0; i < 7; i++) {
        raw[i] = ((uint16_t)resp[3 + i * 2] << 8)
               |  (uint16_t)resp[4 + i * 2];
    }

    float temperature = raw[0] / 10.0f;
    float humidity    = raw[1] / 10.0f;
    float ec          = (float)raw[2];
    float ph          = (raw[3] == 0x7FFF) ? -1.0f : raw[3] / 100.0f;
    float nitrogen    = (float)raw[4];
    float phosphorus  = (float)raw[5];
    float potassium   = (float)raw[6];

    /* ── Affichage tableau ── */
    ESP_LOGI(TAG, "┌──────────────────────────────────────────┐");
    ESP_LOGI(TAG, "│     Données Capteur Sol NBL-S-TMC-7      │");
    ESP_LOGI(TAG, "├──────────────────────────────────────────┤");
    ESP_LOGI(TAG, "│  Température    : %6.1f °C              │", temperature);
    ESP_LOGI(TAG, "│  Humidité       : %6.1f %%              │", humidity);
    ESP_LOGI(TAG, "│  Conductivité   : %6.0f µS/cm          │", ec);
    if (ph < 0.0f)
        ESP_LOGI(TAG, "│  pH             :   INVALIDE (0x7FFF)  │");
    else
        ESP_LOGI(TAG, "│  pH             : %6.2f               │", ph);
    ESP_LOGI(TAG, "│  Azote     (N)  : %6.0f mg/kg          │", nitrogen);
    ESP_LOGI(TAG, "│  Phosphore (P)  : %6.0f mg/kg          │", phosphorus);
    ESP_LOGI(TAG, "│  Potassium (K)  : %6.0f mg/kg          │", potassium);
    ESP_LOGI(TAG, "└──────────────────────────────────────────┘");

    /* ── Alertes plages datasheet ── */
    if (temperature < -40.0f || temperature > 80.0f)
        ESP_LOGW(TAG, "  ⚠  Température hors plage [-40..80°C] !");
    if (humidity < 0.0f || humidity > 100.0f)
        ESP_LOGW(TAG, "  ⚠  Humidité hors plage [0..100%%] !");
    if (ec > 10000.0f)
        ESP_LOGW(TAG, "  ⚠  EC hors plage [0..10000µS/cm] !");
    if (ph >= 0.0f && ph > 10.0f)
        ESP_LOGW(TAG, "  ⚠  pH hors plage [0..10] !");
    if (nitrogen > 2000.0f || phosphorus > 2000.0f || potassium > 2000.0f)
        ESP_LOGW(TAG, "  ⚠  NPK hors plage [0..2000mg/kg] !");
}

/* ══════════════════════════════════════════════════════════════════
 *  Guide de dépannage
 * ══════════════════════════════════════════════════════════════════ */
static void print_troubleshoot(int err_count)
{
    /* Afficher au 1er échec, puis rappeler toutes les 10 erreurs */
    if (err_count != 1 && (err_count % 10) != 0) return;

    ESP_LOGE(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGE(TAG, "║  AUCUNE RÉPONSE (%2d erreur(s))           ║", err_count);
    ESP_LOGE(TAG, "╠══════════════════════════════════════════╣");
    ESP_LOGE(TAG, "║  1. ALIMENTATION CAPTEUR                 ║");
    ESP_LOGE(TAG, "║     IO42 → 5V activé ? (gpio_set_level) ║");
    ESP_LOGE(TAG, "║     Rouge = 5-24V | Noir = GND           ║");
    ESP_LOGE(TAG, "║     ❌ Jamais 3.3V — min 5V requis        ║");
    ESP_LOGE(TAG, "╠══════════════════════════════════════════╣");
    ESP_LOGE(TAG, "║  2. CÂBLAGE RS485                        ║");
    ESP_LOGE(TAG, "║     Jaune (capteur) → A+ transceiver     ║");
    ESP_LOGE(TAG, "║     Bleu  (capteur) → B- transceiver     ║");
    ESP_LOGE(TAG, "║     Si rien → inverser A+ et B-          ║");
    ESP_LOGE(TAG, "╠══════════════════════════════════════════╣");
    ESP_LOGE(TAG, "║  3. BROCHES UART (pinout A.5)            ║");
    ESP_LOGE(TAG, "║     TX ESP32 = IO%d → DI transceiver     ║", SENSOR_TX_PIN);
    ESP_LOGE(TAG, "║     RX ESP32 = IO%d → RO transceiver     ║", SENSOR_RX_PIN);
    ESP_LOGE(TAG, "╠══════════════════════════════════════════╣");
    ESP_LOGE(TAG, "║  4. ADRESSE MODBUS                       ║");
    ESP_LOGE(TAG, "║     Défaut usine = 0x01                  ║");
    ESP_LOGE(TAG, "║     Si modifiée → changer SENSOR_ADDR    ║");
    ESP_LOGE(TAG, "╠══════════════════════════════════════════╣");
    ESP_LOGE(TAG, "║  5. INTER-TRAME                          ║");
    ESP_LOGE(TAG, "║     Datasheet impose ≥ 1000ms entre      ║");
    ESP_LOGE(TAG, "║     deux communications (OK : 2000ms)    ║");
    ESP_LOGE(TAG, "╚══════════════════════════════════════════╝");
}

/* ══════════════════════════════════════════════════════════════════
 *  app_main
 * ══════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   RoboCare — Test Capteur Sol            ║");
    ESP_LOGI(TAG, "║   NBL-S-TMC-7  Modbus RTU RS485          ║");
    ESP_LOGI(TAG, "║   TX=IO%-2d  RX=IO%-2d  PWR=IO%-2d           ║",
             SENSOR_TX_PIN, SENSOR_RX_PIN, SENSOR_POWER_PIN);
    ESP_LOGI(TAG, "║   Addr=0x%02X   Baud=9600,8,N,1           ║", SENSOR_ADDR);
    ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");

    /* 1. Activer l'alimentation 5V du capteur (IO42) */
    sensor_power_init();

    /* 2. Initialiser l'UART1 */
    if (!uart_sensor_init()) {
        ESP_LOGE(TAG, "Échec initialisation UART — arrêt.");
        return;
    }

    /* 3. Délai supplémentaire pour laisser le capteur démarrer */
    ESP_LOGI(TAG, "Attente démarrage capteur (1.5s)...");
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* 4. Boucle de lecture */
    uint8_t response[RESP_MAX_LEN];
    int ok = 0, err = 0;

    ESP_LOGI(TAG, "Démarrage des lectures — intervalle 2s");
    ESP_LOGI(TAG, "──────────────────────────────────────────");

    while (1) {
        memset(response, 0, sizeof(response));

        int len   = modbus_request(response, sizeof(response));
        int total = ok + err + 1;

        if (len > 0) {
            ok++;
            ESP_LOGI(TAG, "── Lecture #%d  [OK:%d / ERR:%d] ──",
                     total, ok, err);
            decode_and_print(response, len);
        } else {
            err++;
            ESP_LOGE(TAG, "── Lecture #%d  TIMEOUT [OK:%d / ERR:%d] ──",
                     total, ok, err);
            print_troubleshoot(err);
        }

        /* Datasheet : ≥ 1000ms entre 2 communications
           On utilise 2000ms pour être à l'aise */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}