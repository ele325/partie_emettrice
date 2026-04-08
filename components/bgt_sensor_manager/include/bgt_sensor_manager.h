/**
 * @file bgt_sensor_manager.h
 */
#ifndef BGT_SENSOR_MANAGER_H
#define BGT_SENSOR_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    float temperature;   /* °C         */
    float humidity;      /* %          */
    float ec;            /* µS/cm      */
    float ph;            /* pH, -1 si invalide */
    float nitrogen;      /* mg/kg      */
    float phosphorus;    /* mg/kg      */
    float potassium;     /* mg/kg      */
} bgt_sensor_data_t;

bool bgt_sensor_manager_init(int rx_pin, int tx_pin, int de_re_pin,
                              int power_pin, uint8_t slave_addr);

bool bgt_sensor_manager_read_all(bgt_sensor_data_t *data);

void bgt_sensor_manager_format_message(uint8_t node_id,
                                       const bgt_sensor_data_t *data,
                                       char *buffer, size_t size);
#endif /* BGT_SENSOR_MANAGER_H */