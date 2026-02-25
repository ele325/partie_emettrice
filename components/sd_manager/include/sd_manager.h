#ifndef SD_MANAGER_H
#define SD_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "rtc_manager.h"

/**
 * @brief Initialise la carte SD
 * @param cs_pin Broche Chip Select
 * @return true si succès
 */
bool sd_manager_init(int cs_pin);

/**
 * @brief Enregistre les données du capteur avec timestamp
 * @param datetime Date et heure actuelles
 * @param data Pointeur vers les données du capteur
 */
void sd_manager_log_sensor_data(const datetime_t *datetime, const bgt_sensor_data_t *data);

#endif