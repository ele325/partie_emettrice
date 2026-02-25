#ifndef BGT_SENSOR_MANAGER_H
#define BGT_SENSOR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

// Structure pour stocker toutes les données du capteur
typedef struct {
    float humidity;      // 0-100%
    float temperature;   // °C
    float ec;            // µS/cm (conductivité)
    float ph;            // pH
    float nitrogen;      // N (mg/kg)
    float phosphorus;    // P (mg/kg)
    float potassium;     // K (mg/kg)
} bgt_sensor_data_t;

/**
 * @brief Initialise le capteur BGT-SMPS (Modbus RS485)
 * @param rx_pin Broche RX (RO du module RS485)
 * @param tx_pin Broche TX (DI du module RS485)
 * @param de_re_pin Broche de contrôle DE/RE
 * @param slave_addr Adresse Modbus du capteur (défaut: 1)
 * @return true si succès
 */
bool bgt_sensor_manager_init(int rx_pin, int tx_pin, int de_re_pin, uint8_t slave_addr);

/**
 * @brief Lit toutes les données du capteur
 * @param data Pointeur vers structure à remplir
 * @return true si lecture réussie
 */
bool bgt_sensor_manager_read_all(bgt_sensor_data_t *data);

/**
 * @brief Formate un message LoRa avec toutes les données
 * @param node_id ID du nœud capteur
 * @param data Structure contenant les données
 * @param buffer Buffer de sortie
 * @param size Taille du buffer
 */
void bgt_sensor_manager_format_message(uint8_t node_id, const bgt_sensor_data_t *data, 
                                       char *buffer, size_t size);

#endif