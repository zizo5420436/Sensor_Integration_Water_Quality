/*
 * Do_sensor.c
 *
 *  Created on: Dec 31, 2025
 *      Author: LENOVO
 */
#include "sensor_common.h"
#include "Do_sensor.h"

#define DO_SLAVE_ID      1
#define REG_TEMP_C       0x000A   // °C * 100
#define REG_DO_CONC_2pt  0x0018   // mg/L * 100

float DO_ReadTemperature(void)
{
    uint16_t raw;
    int err = Modbus_ReadRegisters_IT(&huart1, DO_SLAVE_ID,
                                     REG_TEMP_C, 1, &raw, true);
    if (err != 0) return -1000.0f;
    return raw / 100.0f;
}

float DO_ReadConcentration_2pt(void)
{
    uint16_t raw;
    int err = Modbus_ReadRegisters_IT(&huart1, DO_SLAVE_ID,
                                     REG_DO_CONC_2pt, 1, &raw, true);
    if (err != 0) return -1000.0f;
    return raw / 100.0f;
}
