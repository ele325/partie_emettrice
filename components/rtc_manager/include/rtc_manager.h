#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// Structure pour stocker date et heure
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} datetime_t;

/**
 * @brief Initialise le RTC DS3231
 * @param sda_pin Broche I2C SDA
 * @param scl_pin Broche I2C SCL
 * @return true si succès
 */
bool rtc_manager_init(int sda_pin, int scl_pin);

/**
 * @brief Récupère la date et l'heure actuelles
 * @param datetime Pointeur vers structure à remplir
 * @return true si succès
 */
bool rtc_manager_get_datetime(datetime_t *datetime);

/**
 * @brief Formate la date pour affichage/fichier
 * @param buffer Buffer de sortie
 * @param size Taille du buffer
 * @param datetime Structure date/heure
 */
void rtc_manager_format_date(char *buffer, size_t size, const datetime_t *datetime);

#endif