/**
 * @file lora_manager.c
 * @brief Driver LoRa Ra-02 (SX1278) pour ESP32 — mode émission 433 MHz
 *
 * Corrections appliquées (v1.x → v2.0) :
 *
 *  Héritées :
 *  1. SPI full-duplex (SPI_DEVICE_HALFDUPLEX retiré).
 *  2. lora_read_byte : transaction 16 bits, octet utile en rx_data[1].
 *  3. lora_write_byte : transaction 16 bits, pas de rx_buffer.
 *  4. Reset hardware 100 ms pour stabilisation oscillateur.
 *  5. Vérification version SX1278 = 0x12.
 *  6. REG_FIFO_TX_BASE_ADDR écrit avant remplissage FIFO.
 *  7. REG_FIFO_ADDR_PTR = 0x00 avant écriture payload.
 *  8. Timeout TX = 5 s (SF12 ≈ 2 s sur l'air).
 *
 *  Nouvelles (v2.0) :
 *  9. [CRITIQUE] Signature corrigée : lora_manager_init(cs, rst, dio0)
 *                Plus de mosi/miso/sck — le bus SPI2 est initialisé
 *                une seule fois dans main.c (spi2_bus_init).
 *                Ce fichier appelle uniquement spi_bus_add_device().
 * 10. [CRITIQUE] lora_manager.h mis à jour en conséquence.
 * 11. [ROBUSTESSE] Vérification esp_err_t sur spi_bus_add_device.
 * 12. [ROBUSTESSE] lora_manager_deinit() pour libérer le device SPI
 *                  proprement si nécessaire.
 */

#include "lora_manager.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stddef.h>

static const char *TAG = "LORA";

/* Handle SPI du périphérique SX1278 */
static spi_device_handle_t s_spi_lora = NULL;

/* =========================================================================
 * Registres SX1278
 * ========================================================================= */
#define REG_FIFO              0x00
#define REG_OP_MODE           0x01
#define REG_FR_MSB            0x06
#define REG_FR_MID            0x07
#define REG_FR_LSB            0x08
#define REG_PA_CONFIG         0x09
#define REG_PA_DAC            0x4D   /* +20 dBm mode                        */
#define REG_FIFO_ADDR_PTR     0x0D   /* pointeur courant dans le FIFO        */
#define REG_FIFO_TX_BASE_ADDR 0x0E   /* adresse de base TX dans le FIFO      */
#define REG_IRQ_FLAGS         0x12
#define REG_MODEM_CONFIG1     0x1D
#define REG_MODEM_CONFIG2     0x1E
#define REG_PAYLOAD_LENGTH    0x22
#define REG_VERSION           0x42

/* =========================================================================
 * Modes opérationnels SX1278
 * bit7=1 → mode LoRa | bits2-0 → mode
 * ========================================================================= */
#define LORA_MODE_SLEEP       0x80   /* LoRa + Sleep (requis pour config)    */
#define LORA_MODE_STDBY       0x81   /* LoRa + Standby                       */
#define LORA_MODE_TX          0x83   /* LoRa + Transmit                      */

/* Version attendue du chip */
#define SX1278_VERSION        0x12

/* Timeout transmission SF12 */
#define LORA_TX_TIMEOUT_MS    5000

/* =========================================================================
 * Helpers SPI full-duplex
 *
 * Le SX1278 utilise un protocole SPI standard :
 *   - Lecture  : envoyer [addr & 0x7F, 0x00] → recevoir [x, valeur]
 *   - Écriture : envoyer [addr | 0x80, val]
 *
 * IMPORTANT : SPI_DEVICE_HALFDUPLEX doit être absent des flags.
 * En half-duplex, MISO n'est pas échantillonné pendant TX → lecture échoue.
 * ========================================================================= */

static uint8_t lora_read_byte(uint8_t addr)
{
    uint8_t tx[2] = { addr & 0x7F, 0x00 };
    uint8_t rx[2] = { 0x00, 0x00 };

    spi_transaction_t t = {
        .length    = 16,        /* 16 bits envoyés                          */
        .rxlength  = 16,        /* 16 bits reçus                            */
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    spi_device_transmit(s_spi_lora, &t);
    return rx[1];               /* octet utile = 2ème octet reçu            */
}

static void lora_write_byte(uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { addr | 0x80, value };

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };

    spi_device_transmit(s_spi_lora, &t);
}

/* =========================================================================
 * API publique
 * ========================================================================= */

/**
 * @brief Initialise le module LoRa Ra-02 (SX1278) sur le bus SPI2.
 *
 * PRÉREQUIS : spi_bus_initialize(SPI2_HOST, ...) doit avoir été appelé
 * AVANT cette fonction (dans main.c, étape 1/5 — spi2_bus_init()).
 * Cette fonction appelle uniquement spi_bus_add_device() en interne.
 *
 * @param cs_pin   Broche Chip Select (NSS) → IO10
 * @param rst_pin  Broche Reset             → IO8
 * @param dio0_pin Broche DIO0 (TxDone IRQ) → IO14
 *
 * @return true si le SX1278 est détecté et configuré, false sinon.
 *
 * CORRECTION v2.0 :
 *   Ancienne signature : lora_manager_init(mosi, miso, sck, cs, rst, dio0)
 *   Nouvelle signature : lora_manager_init(cs, rst, dio0)
 *   Les pins MOSI/MISO/SCK sont gérés par spi2_bus_init() dans main.c.
 *   Les passer ici en double causait ESP_ERR_INVALID_STATE au démarrage.
 */
bool lora_manager_init(int cs_pin, int rst_pin, int dio0_pin)
{
    /* ── 1. Reset matériel ─────────────────────────────────────────────── */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << rst_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 0);     /* impulsion reset active-low           */
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); /* stabilisation oscillateur LoRa       */

    /* ── 2. Ajout du SX1278 sur le bus SPI2 déjà initialisé ───────────── */
    /*
     * RÈGLE : ne jamais appeler spi_bus_initialize() ici.
     * Le bus SPI2 est initialisé une seule fois dans spi2_bus_init() (main.c).
     * On ajoute simplement ce périphérique au bus existant.
     *
     * flags = 0 → full-duplex (MOSI et MISO actifs simultanément).
     * SPI_DEVICE_HALFDUPLEX interdit : bloquerait la lecture des registres.
     */
    spi_device_interface_config_t devcfg = {
        .mode           = 0,                /* CPOL=0, CPHA=0 (SX1278 std)  */
        .clock_speed_hz = 1 * 1000 * 1000, /* 1 MHz — fiable sur Ra-02     */
        .spics_io_num   = cs_pin,           /* NSS = IO10                   */
        .queue_size     = 7,
        .flags          = 0,                /* full-duplex                  */
        .pre_cb         = NULL,
        .post_cb        = NULL,
    };

    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi_lora);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device échoué : %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "SX1278 ajouté sur SPI2_HOST  CS=IO%d  RST=IO%d  DIO0=IO%d",
             cs_pin, rst_pin, dio0_pin);

    /* ── 3. Vérification version chip ──────────────────────────────────── */
    uint8_t version = lora_read_byte(REG_VERSION);
    ESP_LOGI(TAG, "Registre VERSION = 0x%02X (attendu 0x%02X)",
             version, SX1278_VERSION);

    if (version != SX1278_VERSION) {
        ESP_LOGE(TAG, "SX1278 non détecté !");
        ESP_LOGE(TAG, "  → Vérifier : câblage MOSI/MISO, tension 3.3V,");
        ESP_LOGE(TAG, "               soudures NSS/SCK, condensateur 100nF.");
        spi_bus_remove_device(s_spi_lora);
        s_spi_lora = NULL;
        return false;
    }

    /* ── 4. Configuration du SX1278 ────────────────────────────────────── */

    /* Mode Sleep LoRa — obligatoire avant de modifier les registres config */
    lora_write_byte(REG_OP_MODE, LORA_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(15));

    /* Fréquence : 433 MHz
     * Frf = 433 000 000 / (32 000 000 / 2^19) = 7 094 272 → 0x6C 0x80 0x00 */
    lora_write_byte(REG_FR_MSB, 0x6C);
    lora_write_byte(REG_FR_MID, 0x80);
    lora_write_byte(REG_FR_LSB, 0x00);

    /* PA_CONFIG : PA_BOOST activé (bit7=1), puissance maximale
     * 0x8F = 1000 1111 → PA_SELECT=1, MaxPower=000, OutputPower=1111     */
    lora_write_byte(REG_PA_CONFIG, 0x8F);

    /* PA_DAC : 0x87 active le mode +20 dBm (vs 0x84 pour +17 dBm)        */
    lora_write_byte(REG_PA_DAC, 0x87);

    /* Adresse de base TX dans le FIFO partagé (256 octets)                */
    lora_write_byte(REG_FIFO_TX_BASE_ADDR, 0x00);

    /* ModemConfig1 :
     *   BW = 125 kHz         → bits[7-4] = 0111
     *   CodingRate = 4/5     → bits[3-1] = 001
     *   ImplicitHeader = OFF → bit[0]    = 0
     *   → 0x72                                                            */
    lora_write_byte(REG_MODEM_CONFIG1, 0x72);

    /* ModemConfig2 :
     *   SpreadingFactor = 12 → bits[7-4] = 1100  (portée maximale)
     *   TxContinuousMode = 0 → bit[3]    = 0
     *   RxPayloadCrcOn   = 1 → bit[2]    = 1
     *   SymbTimeout MSB  = 0 → bits[1-0] = 00
     *   → 0xC4                                                            */
    lora_write_byte(REG_MODEM_CONFIG2, 0xC4);

    /* Repasser en Standby — prêt à émettre                                */
    lora_write_byte(REG_OP_MODE, LORA_MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "LoRa prêt : 433 MHz | SF12 | BW125 kHz | CR4/5 | +20 dBm");
    return true;
}

/**
 * @brief Envoie un message texte via LoRa.
 *
 * Séquence :
 *   1. Standby
 *   2. Effacement IRQ
 *   3. Pointeur FIFO = 0x00
 *   4. Écriture payload dans FIFO
 *   5. Longueur payload
 *   6. Mode TX
 *   7. Attente TxDone (bit3 REG_IRQ_FLAGS) avec timeout 5 s
 *   8. Effacement IRQ + retour Standby
 *
 * @param message  Chaîne de caractères à envoyer (max 255 octets)
 * @return true si envoyé avec succès, false si timeout ou message invalide.
 */
bool lora_manager_send_message(const char *message)
{
    if (!message || s_spi_lora == NULL) return false;

    size_t len = strlen(message);
    if (len == 0)   return false;
    if (len > 255)  len = 255;      /* limite hardware SX1278               */

    /* 1. Standby avant configuration FIFO */
    lora_write_byte(REG_OP_MODE, LORA_MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 2. Effacer tous les flags IRQ résiduels */
    lora_write_byte(REG_IRQ_FLAGS, 0xFF);

    /* 3. Pointer le FIFO sur l'adresse TX base */
    lora_write_byte(REG_FIFO_ADDR_PTR, 0x00);

    /* 4. Écrire le payload octet par octet dans le FIFO */
    for (size_t i = 0; i < len; i++) {
        lora_write_byte(REG_FIFO, (uint8_t)message[i]);
    }

    /* 5. Indiquer la longueur du payload au modem */
    lora_write_byte(REG_PAYLOAD_LENGTH, (uint8_t)len);

    /* 6. Démarrer la transmission */
    lora_write_byte(REG_OP_MODE, LORA_MODE_TX);

    /* 7. Attendre TxDone (bit 3 de REG_IRQ_FLAGS)
     *
     *    SF12 + BW125 + ~50 octets ≈ 2.0 s sur l'air.
     *    Timeout fixé à 5 s pour avoir de la marge en cas de retard.
     *    Polling toutes les 10 ms → résolution suffisante.                */
    int elapsed_ms = 0;

    while (elapsed_ms < LORA_TX_TIMEOUT_MS) {
        uint8_t irq = lora_read_byte(REG_IRQ_FLAGS);
        if (irq & 0x08) {           /* bit3 = TxDone                        */
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed_ms += 10;
    }

    /* 8. Effacer les IRQ + retour en Standby */
    lora_write_byte(REG_IRQ_FLAGS, 0xFF);
    lora_write_byte(REG_OP_MODE,   LORA_MODE_STDBY);

    if (elapsed_ms >= LORA_TX_TIMEOUT_MS) {
        ESP_LOGE(TAG, "Timeout TX après %d ms — message perdu : %s",
                 LORA_TX_TIMEOUT_MS, message);
        return false;
    }

    ESP_LOGI(TAG, "TX OK — %d octets en %d ms : %s",
             (int)len, elapsed_ms, message);
    return true;
}

/**
 * @brief Libère le périphérique SPI LoRa du bus SPI2.
 *
 * À appeler avant esp_deep_sleep_start() si une réinitialisation
 * propre est souhaitée au prochain réveil.
 * En pratique, le deep sleep ESP32 réinitialise de toute façon les
 * handles SPI — cette fonction est fournie pour la complétude.
 */
void lora_manager_deinit(void)
{
    if (s_spi_lora != NULL) {
        spi_bus_remove_device(s_spi_lora);
        s_spi_lora = NULL;
        ESP_LOGI(TAG, "LoRa dés-initialisé");
    }
}