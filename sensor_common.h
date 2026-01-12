/*
 * sensor_common.h
 *
 *  Created on: Dec 31, 2025
 *      Author: LENOVO
 */

#ifndef INC_SENSOR_COMMON_H_
#define INC_SENSOR_COMMON_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tla20xx.h"

/* ===== External peripherals ===== */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef  hi2c2;
extern TLA20XX_Handle_t  tla;

/* ===== Modbus ===== */
int Modbus_ReadRegisters_IT(UART_HandleTypeDef *huart,
                            uint8_t slave_id,
                            uint16_t reg_addr,
                            uint16_t num_regs,
                            uint16_t *out,
                            bool out_is_u16);


#endif /* INC_SENSOR_COMMON_H_ */
