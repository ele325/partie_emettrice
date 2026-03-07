/**
 * @file lora_manager.h
 * @brief Interface publique du driver LoRa Ra-02 (SX1278) — v2.0
 *
 * CORRECTION v2.0 :
 *   lora_manager_init() ne prend plus mosi/miso/sck en paramètres.
 *   Le bus SPI2 est initialisé une seule fois dans main.c (spi2_bus_init).
 *   Ce driver appelle uniquement spi_bus_add_device(SPI2_HOST, ...).
 */

#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <stdbool.h>

/**
 * @brief Initialise le module LoRa Ra-02 (SX1278) sur SPI2_HOST.
 *
 * PRÉREQUIS : spi_bus_initialize(SPI2_HOST, ...) doit avoir été appelé
 * avant cette fonction (dans spi2_bus_init() de main.c).
 *
 * Configure :
 *   - Fréquence  : 433 MHz
 *   - SF         : 12 (portée maximale)
 *   - BW         : 125 kHz
 *   - CR         : 4/5
 *   - Puissance  : +20 dBm (PA_BOOST + PA_DAC)
 *
 * @param cs_pin   Broche NSS/CS du SX1278  → IO10
 * @param rst_pin  Broche RST du SX1278     → IO8
 * @param dio0_pin Broche DIO0 du SX1278    → IO14
 *
 * @return true si SX1278 détecté (version 0x12) et configuré.
 */
bool lora_manager_init(int cs_pin, int rst_pin, int dio0_pin);

/**
 * @brief Envoie un message texte via LoRa (max 255 octets).
 *
 * @param message  Chaîne à envoyer (null-terminée)
 * @return true si TxDone reçu avant timeout (5 s), false sinon.
 */
bool lora_manager_send_message(const char *message);

/**
 * @brief Libère le périphérique SPI LoRa du bus SPI2.
 *
 * Optionnel avant deep sleep. Le deep sleep ESP32 réinitialise
 * de toute façon les handles SPI au réveil.
 */
void lora_manager_deinit(void);

#endif /* LORA_MANAGER_H */