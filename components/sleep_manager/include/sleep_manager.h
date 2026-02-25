#ifndef SLEEP_MANAGER_H
#define SLEEP_MANAGER_H

#include <stdint.h>

/**
 * @brief Configure le réveil par timer
 * @param sleep_us Temps de sommeil en microsecondes
 */
void sleep_manager_configure_timer(uint64_t sleep_us);

/**
 * @brief Passe en deep sleep
 */
void sleep_manager_enter_deep_sleep(void) __attribute__((noreturn));

#endif