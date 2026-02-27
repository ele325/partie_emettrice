#include "lora_manager.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "LORA";
static spi_device_handle_t spi_lora;

// Registres LoRa (SX1276/SX1278)
#define REG_FIFO         0x00
#define REG_OP_MODE      0x01
#define REG_FR_MSB       0x06
#define REG_FR_MID       0x07
#define REG_FR_LSB       0x08
#define REG_PA_CONFIG    0x09
#define REG_FIFO_TX_BASE 0x0E
#define REG_IRQ_FLAGS    0x12
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_PAYLOAD_LENGTH 0x22
#define REG_VERSION      0x42

#define LORA_MODE_SLEEP  0x80
#define LORA_MODE_STDBY  0x81
#define LORA_MODE_TX     0x83

// Lecture/écriture SPI
static uint8_t lora_read_byte(uint8_t addr)
{
    spi_transaction_t t = {
        .flags = 0,
        .length = 16,
        .rxlength = 8,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    uint8_t tx_data[2] = { addr & 0x7F, 0x00 };
    uint8_t rx_data[2];
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    spi_device_transmit(spi_lora, &t);
    return rx_data[1];
}

static void lora_write_byte(uint8_t addr, uint8_t value)
{
    spi_transaction_t t = {
        .flags = 0,
        .length = 16,
        .tx_buffer = NULL,
    };
    uint8_t tx_data[2] = { addr | 0x80, value };
    t.tx_buffer = tx_data;
    spi_device_transmit(spi_lora, &t);
}

bool lora_manager_init(int mosi_pin, int miso_pin, int sck_pin, 
                      int cs_pin, int rst_pin, int dio0_pin)
{
    esp_err_t ret;
    
    // Reset du module LoRa
    gpio_set_direction(rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Configuration du périphérique LoRa sur le bus SPI existant
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 5 * 1000 * 1000,
        .spics_io_num = cs_pin,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_lora);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur ajout périphérique SPI");
        return false;
    }
    
    // Vérifier la version du module
    uint8_t version = lora_read_byte(REG_VERSION);
    ESP_LOGI(TAG, "Version LoRa: 0x%02X", version);
    
    // Configuration LoRa
    lora_write_byte(REG_OP_MODE, LORA_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Fréquence 433 MHz
    lora_write_byte(REG_FR_MSB, 0x6C);
    lora_write_byte(REG_FR_MID, 0x80);
    lora_write_byte(REG_FR_LSB, 0x00);
    
    lora_write_byte(REG_PA_CONFIG, 0x8F);  // Puissance max
    
    // Configuration modulation (BW=125kHz, CR=4/5, SF=12)
    lora_write_byte(REG_MODEM_CONFIG1, 0x72);
    lora_write_byte(REG_MODEM_CONFIG2, 0x74);
    
    // Mode veille
    lora_write_byte(REG_OP_MODE, LORA_MODE_STDBY);
    
    ESP_LOGI(TAG, "LoRa initialisé sur 433MHz (mode émission)");
    return true;
}

bool lora_manager_send_message(const char *message)
{
    size_t len = strlen(message);
    if (len > 255) len = 255;
    
    // Passer en mode veille pour configuration
    lora_write_byte(REG_OP_MODE, LORA_MODE_STDBY);
    
    // Configurer la longueur du payload
    lora_write_byte(REG_PAYLOAD_LENGTH, len);
    
    // Écrire le message dans FIFO
    lora_write_byte(REG_FIFO_TX_BASE, 0x00);
    for (size_t i = 0; i < len; i++) {
        lora_write_byte(REG_FIFO, message[i]);
    }
    
    // Passer en mode émission
    lora_write_byte(REG_OP_MODE, LORA_MODE_TX);
    
    // Attendre fin d'émission (max 1s)
    int timeout = 0;
    while (!(lora_read_byte(REG_IRQ_FLAGS) & 0x08)) {  // TxDone flag
        vTaskDelay(pdMS_TO_TICKS(1));
        if (++timeout > 1000) {
            ESP_LOGE(TAG, "Timeout émission LoRa");
            return false;
        }
    }
    
    // Effacer IRQ
    lora_write_byte(REG_IRQ_FLAGS, 0xFF);
    
    // Retour en mode veille
    lora_write_byte(REG_OP_MODE, LORA_MODE_STDBY);
    
    ESP_LOGI(TAG, "Message envoyé: %s", message);
    return true;
}