#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialise le module LoRa en mode émission
 * @param mosi_pin Broche MOSI
 * @param miso_pin Broche MISO
 * @param sck_pin Broche SCK
 * @param cs_pin Broche Chip Select
 * @param rst_pin Broche Reset
 * @param dio0_pin Broche DIO0 (IRQ)
 * @return true si succès
 */
bool lora_manager_init(int mosi_pin, int miso_pin, int sck_pin, 
                      int cs_pin, int rst_pin, int dio0_pin);

/**
 * @brief Envoie un message LoRa
 * @param message Chaîne de caractères à envoyer
 * @return true si envoi réussi
 */
bool lora_manager_send_message(const char *message);

#endif