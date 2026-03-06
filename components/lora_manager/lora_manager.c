/**
 * @file lora_manager.c
 * @brief Driver LoRa Ra-02 (SX1278) pour ESP32 — mode émission 433 MHz
 *
 * Corrections appliquées :
 *  1. SPI full-duplex (flag SPI_DEVICE_HALFDUPLEX retiré) → lecture/écriture
 *     simultanées sur MOSI/MISO, comme l'exige le SX1278.
 *  2. lora_read_byte : transaction 16 bits full-duplex, octet utile en rx_data[1].
 *  3. lora_write_byte : transaction 16 bits, pas de rx_buffer.
 *  4. Reset hardware allongé (100 ms) pour garantir la stabilisation du module.
 *  5. Vérification de version : le SX1278 doit renvoyer 0x12 — retour false sinon.
 *  6. Adresse FIFO TX base correctement écrite avant le remplissage du FIFO.
 *  7. REG_FIFO_ADDR_PTR mis à 0x00 avant l'écriture du payload.
 *  8. Timeout d'émission augmenté à 5 s (SF12 peut prendre ~2 s sur l'air).
 */

#include "lora_manager.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "LORA";
static spi_device_handle_t spi_lora;

/* ─── Registres SX1278 ─────────────────────────────────────────────────── */
#define REG_FIFO              0x00
#define REG_OP_MODE           0x01
#define REG_FR_MSB            0x06
#define REG_FR_MID            0x07
#define REG_FR_LSB            0x08
#define REG_PA_CONFIG         0x09
#define REG_FIFO_ADDR_PTR     0x0D   /* pointeur courant dans le FIFO        */
#define REG_FIFO_TX_BASE_ADDR 0x0E   /* adresse de base TX dans le FIFO      */
#define REG_IRQ_FLAGS         0x12
#define REG_MODEM_CONFIG1     0x1D
#define REG_MODEM_CONFIG2     0x1E
#define REG_PAYLOAD_LENGTH    0x22
#define REG_VERSION           0x42

/* ─── Modes opérationnels (bit7=1 : LoRa, bits2-0 : mode) ──────────────── */
#define LORA_MODE_SLEEP       0x80   /* LoRa + Sleep                         */
#define LORA_MODE_STDBY       0x81   /* LoRa + Standby                       */
#define LORA_MODE_TX          0x83   /* LoRa + Transmit                      */

/* Version attendue du SX1278 */
#define SX1278_VERSION        0x12

/* ─── Helpers SPI (full-duplex) ─────────────────────────────────────────── */

/**
 * Lit un registre du SX1278.
 * En full-duplex, on envoie [addr|0x00, 0x00] et on récupère la réponse
 * dans rx_data[1] (le premier octet reçu est sans signification).
 */
static uint8_t lora_read_byte(uint8_t addr)
{
    uint8_t tx_data[2] = { addr & 0x7F, 0x00 };
    uint8_t rx_data[2] = { 0x00, 0x00 };

    spi_transaction_t t = {
        .length    = 16,          /* 16 bits total TX                        */
        .rxlength  = 16,          /* 16 bits total RX                        */
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    spi_device_transmit(spi_lora, &t);
    return rx_data[1];            /* octet utile = 2e octet reçu             */
}

/**
 * Écrit un registre du SX1278.
 * MSB à 1 indique une écriture (SX1278 SPI protocol).
 */
static void lora_write_byte(uint8_t addr, uint8_t value)
{
    uint8_t tx_data[2] = { addr | 0x80, value };

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
    };

    spi_device_transmit(spi_lora, &t);
}

/* ─── API publique ──────────────────────────────────────────────────────── */

bool lora_manager_init(int mosi_pin, int miso_pin, int sck_pin,
                       int cs_pin, int rst_pin, int dio0_pin)
{
    /* 1. Reset matériel du module ---------------------------------------- */
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
    gpio_set_level(rst_pin, 0);   /* impulsion reset active-low             */
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); /* attente stabilisation oscillateur     */

    /* 2. Ajout du périphérique SX1278 sur le bus SPI ---------------------- */
    /*    IMPORTANT : SPI_DEVICE_HALFDUPLEX retiré → full-duplex obligatoire */
    spi_device_interface_config_t devcfg = {
        .mode            = 0,                  /* CPOL=0, CPHA=0            */
        .clock_speed_hz  = 1 * 1000 * 1000,   /* 1 MHz (fiable au démarrage)*/
        .spics_io_num    = cs_pin,
        .queue_size      = 7,
        .flags           = 0,                  /* full-duplex               */
        .pre_cb          = NULL,
        .post_cb         = NULL,
    };

    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_lora);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur ajout périphérique SPI : %s", esp_err_to_name(ret));
        return false;
    }

    /* 3. Vérification de la version --------------------------------------- */
    uint8_t version = lora_read_byte(REG_VERSION);
    ESP_LOGI(TAG, "Version LoRa lue : 0x%02X (attendu 0x%02X)", version, SX1278_VERSION);

    if (version != SX1278_VERSION) {
        ESP_LOGE(TAG, "Module non détecté ! Vérifiez : câblage MOSI/MISO, "
                      "tension 3.3V, soudures NSS/SCK.");
        return false;
    }

    /* 4. Configuration LoRa ----------------------------------------------- */

    /* Passer en mode Sleep LoRa avant toute configuration des registres */
    lora_write_byte(REG_OP_MODE, LORA_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(15));

    /* Fréquence 433 MHz
     * Frf = 433e6 / (32e6 / 2^19) = 433e6 / 61.035 = 7094272 → 0x6C8000  */
    lora_write_byte(REG_FR_MSB, 0x6C);
    lora_write_byte(REG_FR_MID, 0x80);
    lora_write_byte(REG_FR_LSB, 0x00);

    /* PA_CONFIG : PA_BOOST activé, puissance max (+20 dBm)
     * 0x8F = PA_SELECT(bit7=1) + MaxPower(bits6-4=000) + OutputPower(bits3-0=1111)
     * Pour +20 dBm réels, PA_DAC doit aussi être configuré (0x87 au reg 0x4D) */
    lora_write_byte(REG_PA_CONFIG, 0x8F);
    lora_write_byte(0x4D, 0x87);   /* REG_PA_DAC : active le +20 dBm        */

    /* Adresse de base TX dans le FIFO (256 octets partagés) */
    lora_write_byte(REG_FIFO_TX_BASE_ADDR, 0x00);

    /* ModemConfig1 : BW=125kHz (0111), CR=4/5 (001), explicit header (0)
     * bits[7-4]=0111, bits[3-1]=001, bit[0]=0 → 0x72                       */
    lora_write_byte(REG_MODEM_CONFIG1, 0x72);

    /* ModemConfig2 : SF=12 (1100), TX mode normal (0), RxPayloadCrcOn (1)
     * bits[7-4]=1100, bit[3]=0, bit[2]=1, bits[1-0]=00 → 0xC4
     * Note : 0x74 active SF=7, on utilise SF=12 pour portée maximale       */
    lora_write_byte(REG_MODEM_CONFIG2, 0xC4);

    /* Repasser en Standby */
    lora_write_byte(REG_OP_MODE, LORA_MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "LoRa initialisé sur 433 MHz — SF12, BW125, CR4/5");
    return true;
}

bool lora_manager_send_message(const char *message)
{
    if (!message) return false;

    size_t len = strlen(message);
    if (len == 0) return false;
    if (len > 255) len = 255;

    /* 1. Standby avant configuration -------------------------------------- */
    lora_write_byte(REG_OP_MODE, LORA_MODE_STDBY);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 2. Effacer tous les flags IRQ restants */
    lora_write_byte(REG_IRQ_FLAGS, 0xFF);

    /* 3. Placer le pointeur FIFO au début de la zone TX ------------------- */
    lora_write_byte(REG_FIFO_ADDR_PTR, 0x00);

    /* 4. Écrire le payload dans le FIFO ----------------------------------- */
    for (size_t i = 0; i < len; i++) {
        lora_write_byte(REG_FIFO, (uint8_t)message[i]);
    }

    /* 5. Indiquer la longueur du payload ---------------------------------- */
    lora_write_byte(REG_PAYLOAD_LENGTH, (uint8_t)len);

    /* 6. Lancer la transmission ------------------------------------------- */
    lora_write_byte(REG_OP_MODE, LORA_MODE_TX);

    /* 7. Attendre TxDone (bit 3 de REG_IRQ_FLAGS)
     *    SF12 + BW125 + payload ~20 octets ≈ 2 secondes sur l'air
     *    Timeout fixé à 5 secondes pour avoir de la marge.                  */
    int timeout_ms = 0;
    const int TIMEOUT_MAX_MS = 5000;

    while (timeout_ms < TIMEOUT_MAX_MS) {
        uint8_t irq = lora_read_byte(REG_IRQ_FLAGS);
        if (irq & 0x08) {           /* TxDone flag                          */
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout_ms += 10;
    }

    /* 8. Effacer les IRQ -------------------------------------------------- */
    lora_write_byte(REG_IRQ_FLAGS, 0xFF);

    /* 9. Retour en Standby ------------------------------------------------ */
    lora_write_byte(REG_OP_MODE, LORA_MODE_STDBY);

    if (timeout_ms >= TIMEOUT_MAX_MS) {
        ESP_LOGE(TAG, "Timeout émission LoRa (message: %s)", message);
        return false;
    }

    ESP_LOGI(TAG, "Message envoyé (%d octets) : %s", (int)len, message);
    return true;
}
