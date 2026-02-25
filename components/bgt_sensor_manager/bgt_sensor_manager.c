#include "bgt_sensor_manager.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "BGT_SENSOR";
static uint8_t sensor_addr = 1;
static int rs485_en_pin = -1;
static bool sensor_initialized = false;

// Calcul CRC16 Modbus
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

bool bgt_sensor_manager_init(int rx_pin, int tx_pin, int de_re_pin, uint8_t slave_addr)
{
    sensor_addr = slave_addr;
    rs485_en_pin = de_re_pin;
    
    // Configuration UART (utiliser UART_NUM_1 car ESP32-S2 n'a que UART0 et UART1)
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    esp_err_t ret = uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur installation UART: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = uart_param_config(UART_NUM_1, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur configuration UART: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = uart_set_pin(UART_NUM_1, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erreur réglage pins UART: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configuration broche de contrôle RS485
    if (de_re_pin >= 0) {
        gpio_set_direction(de_re_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(de_re_pin, 0);  // Mode réception par défaut
    }
    
    sensor_initialized = true;
    ESP_LOGI(TAG, "Capteur BGT-SMPS initialisé (adresse %d)", slave_addr);
    return true;
}

bool bgt_sensor_manager_read_all(bgt_sensor_data_t *data)
{
    if (!sensor_initialized || !data) return false;
    
    // Activer émission pour la requête
    if (rs485_en_pin >= 0) {
        gpio_set_level(rs485_en_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Trame Modbus: Lecture de 14 registres (7 paramètres * 2 registres)
    uint8_t request[8] = {
        sensor_addr,        // Adresse esclave
        0x03,               // Fonction: Read Holding Registers
        0x00, 0x00,         // Adresse début: 0x0000
        0x00, 7,            // Nombre de registres à lire (7 paramètres)
        0x00, 0x00          // CRC (calculé après)
    };
    
    uint16_t crc = crc16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;
    
    uart_write_bytes(UART_NUM_1, (const char*)request, 8);
    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(100));
    
    // Passer en réception pour lire la réponse
    if (rs485_en_pin >= 0) {
        gpio_set_level(rs485_en_pin, 0);
    }
    
    // Lire la réponse (minimum 3 + 2*7 + 2 = 19 bytes)
    uint8_t response[64];
    int len = uart_read_bytes(UART_NUM_1, response, sizeof(response), pdMS_TO_TICKS(500));
    
    if (len < 19) {
        ESP_LOGE(TAG, "Réponse trop courte: %d bytes", len);
        return false;
    }
    
    // Vérifier CRC
    crc = crc16(response, len - 2);
    if (response[len-2] != (crc & 0xFF) || response[len-1] != ((crc >> 8) & 0xFF)) {
        ESP_LOGE(TAG, "CRC invalide");
        return false;
    }
    
    // Extraire les valeurs (format: chaque paramètre sur 2 registres en big-endian)
    // Format typique: int16_t * 100
    for (int i = 0; i < 7; i++) {
        uint16_t reg_high = response[3 + i*2];
        uint16_t reg_low = response[4 + i*2];
        uint16_t value = (reg_high << 8) | reg_low;
        
        float float_value = value / 100.0;
        
        switch (i) {
            case 0: data->humidity = float_value; break;
            case 1: data->temperature = float_value; break;
            case 2: data->ec = float_value; break;
            case 3: data->ph = float_value; break;
            case 4: data->nitrogen = float_value; break;
            case 5: data->phosphorus = float_value; break;
            case 6: data->potassium = float_value; break;
        }
    }
    
    ESP_LOGD(TAG, "Lecture réussie: H=%.1f T=%.1f EC=%.0f pH=%.1f N=%.0f P=%.0f K=%.0f",
             data->humidity, data->temperature, data->ec, data->ph,
             data->nitrogen, data->phosphorus, data->potassium);
    
    return true;
}

void bgt_sensor_manager_format_message(uint8_t node_id, const bgt_sensor_data_t *data, 
                                       char *buffer, size_t size)
{
    snprintf(buffer, size, 
             "ID:%d,H:%.1f,T:%.1f,EC:%.0f,PH:%.1f,N:%.0f,P:%.0f,K:%.0f",
             node_id, data->humidity, data->temperature, data->ec, data->ph,
             data->nitrogen, data->phosphorus, data->potassium);
}