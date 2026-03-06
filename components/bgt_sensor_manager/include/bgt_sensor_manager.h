#ifndef BGT_SENSOR_MANAGER_H
#define BGT_SENSOR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/**
 * @file bgt_sensor_manager.h
 * @brief Driver NBL-S-TMC-7 / 7-in-1 Soil Sensor (Modbus RTU RS485)
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  PinOut Carte Émettrice RoboCare (confirmé image A.5)           │
 * │                                                                 │
 * │  Ligne 20 : RX_UART CAPTEUR → IO17                             │
 * │  Ligne 21 : TX_UART CAPTEUR → IO18   ← (pas IO20 !)            │
 * │  Ligne 36 : OFF POWER 5V    → IO42  (alimentation capteur)     │
 * │                                                                 │
 * │  Transceiver embarqué SN65HVD485E → DE/RE géré en interne      │
 * │  → passer de_re_pin = -1                                        │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * Registres Modbus (FC=03, addr=0x01) — datasheet p.3 :
 *   Reg 0x0000 = Temperature   ÷10  → °C       ex: 276 → 27.6°C
 *   Reg 0x0001 = Moisture      ÷10  → %        ex: 326 → 32.6%
 *   Reg 0x0002 = Conductivity  brut → µS/cm    ex: 75  → 75µS/cm
 *   Reg 0x0003 = PH            ÷100 → pH       ex: 722 → 7.22pH
 *                               0x7FFF = invalide / non connecté
 *   Reg 0x0004 = N             brut → mg/kg    ex: 8   → 8mg/kg
 *   Reg 0x0005 = P             brut → mg/kg
 *   Reg 0x0006 = K             brut → mg/kg
 *
 * Trame de référence (datasheet p.3) :
 *   Send  : 01 03 00 00 00 07 44 0C
 *   Return: 01 03 0E 01 14 01 46 00 4B 02 D2 00 08 00 08 00 15 F8 0E
 */

/* ─── Structure de données ───────────────────────────────────────────────── */
typedef struct {
    float temperature;   /* °C         plage : -40..80   */
    float humidity;      /* %          plage :   0..100  */
    float ec;            /* µS/cm      plage :   0..10000*/
    float ph;            /* pH         plage :   0..10   */
                         /* -1.0 si invalide (0x7FFF)    */
    float nitrogen;      /* mg/kg      plage :   0..2000 */
    float phosphorus;    /* mg/kg      plage :   0..2000 */
    float potassium;     /* mg/kg      plage :   0..2000 */
} bgt_sensor_data_t;

/* ─── API publique ───────────────────────────────────────────────────────── */

/**
 * @brief  Initialise l'UART1, la broche d'alimentation 5V et le DE/RE
 *
 * @param  rx_pin      Broche RX  → IO17 (ligne 20)
 * @param  tx_pin      Broche TX  → IO18 (ligne 21)
 * @param  de_re_pin   Broche DE/RE, ou -1 si transceiver intégré
 * @param  power_pin   Broche alimentation 5V capteur → IO42 (ligne 36)
 *                     Mettre -1 si alimentation externe permanente
 * @param  slave_addr  Adresse Modbus capteur (défaut usine : 0x01)
 * @return true si OK
 */
bool bgt_sensor_manager_init(int rx_pin, int tx_pin, int de_re_pin,
                              int power_pin, uint8_t slave_addr);

/**
 * @brief  Lit les 7 registres du capteur en une seule trame Modbus
 * @param  data  Pointeur vers la structure à remplir
 * @return true si lecture + CRC OK
 */
bool bgt_sensor_manager_read_all(bgt_sensor_data_t *data);

/**
 * @brief  Formate un message LoRa compact
 *         Format : "ID:1,H:32.6,T:27.6,EC:75,PH:7.22,N:8,P:8,K:21"
 * @param  node_id  ID du nœud (0-255)
 * @param  data     Données capteur
 * @param  buffer   Buffer de sortie (≥ 80 octets recommandé)
 * @param  size     Taille du buffer
 */
void bgt_sensor_manager_format_message(uint8_t node_id,
                                       const bgt_sensor_data_t *data,
                                       char *buffer, size_t size);

/** @brief  Coupe l'alimentation 5V du capteur (économie batterie) */
void bgt_sensor_manager_power_off(void);

/** @brief  Rallume l'alimentation 5V et attend 500ms (stabilisation) */
void bgt_sensor_manager_power_on(void);

#endif /* BGT_SENSOR_MANAGER_H */