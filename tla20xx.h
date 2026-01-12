/*
 * tla20xx.h
 *
 *  Created on: Dec 30, 2025
 *      Author: LENOVO
 */

#ifndef __TLA20XX_H
#define __TLA20XX_H

#include "stm32f3xx_hal.h"

/* Registers */
#define TLA20XX_CONV_REG  0x00
#define TLA20XX_CONF_REG  0x01

/* Default I2C address (HAL = left shifted) */
#define TLA20XX_I2C_ADDR  (0x48 << 1)

/* Data Rate */
typedef enum {
    DR_128SPS  = 0x00,
    DR_250SPS  = 0x01,
    DR_490SPS  = 0x02,
    DR_920SPS  = 0x03,
    DR_1600SPS = 0x04,
    DR_2400SPS = 0x05,
    DR_3300SPS = 0x06
} TLA20XX_DR;

/* Full Scale Range */
typedef enum {
    FSR_6_144V = 0x00,
    FSR_4_096V = 0x01,
    FSR_2_048V = 0x02,
    FSR_1_024V = 0x03,
    FSR_0_512V = 0x04,
    FSR_0_256V = 0x05
} TLA20XX_FSR;

/* MUX */
typedef enum {
    MUX_AIN0_AIN1 = 0x00,
    MUX_AIN0_AIN3 = 0x01,
    MUX_AIN1_AIN3 = 0x02,
    MUX_AIN2_AIN3 = 0x03,
    MUX_AIN0_GND  = 0x04,
    MUX_AIN1_GND  = 0x05,
    MUX_AIN2_GND  = 0x06,
    MUX_AIN3_GND  = 0x07
} TLA20XX_MUX;

/* Mode */
typedef enum {
    OP_CONTINUOUS = 0,
    OP_SINGLE     = 1
} TLA20XX_MODE;

/* Handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
} TLA20XX_Handle_t;

/* API */
void     TLA20XX_Init(TLA20XX_Handle_t *dev,
                      I2C_HandleTypeDef *hi2c,
                      uint8_t addr);

void     TLA20XX_Begin(TLA20XX_Handle_t *dev);

void     TLA20XX_SetFSR (TLA20XX_Handle_t *dev, TLA20XX_FSR fsr);
void     TLA20XX_SetMode(TLA20XX_Handle_t *dev, TLA20XX_MODE mode);
void     TLA20XX_SetDR  (TLA20XX_Handle_t *dev, TLA20XX_DR rate);
void     TLA20XX_SetMux (TLA20XX_Handle_t *dev, TLA20XX_MUX mux);

int16_t  TLA20XX_ReadADC(TLA20XX_Handle_t *dev);

void     TLA20XX_WriteReg(TLA20XX_Handle_t *dev, uint8_t reg, uint16_t data);
uint16_t TLA20XX_ReadReg (TLA20XX_Handle_t *dev, uint8_t reg);

#endif
