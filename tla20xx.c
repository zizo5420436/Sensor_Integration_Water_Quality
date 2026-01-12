/*
 * tla20xx.c
 *
 *  Created on: Dec 30, 2025
 *      Author: LENOVO
 */

#include "tla20xx.h"

void TLA20XX_Init(TLA20XX_Handle_t *dev,
                  I2C_HandleTypeDef *hi2c,
                  uint8_t addr)
{
    dev->hi2c     = hi2c;
    dev->i2c_addr = addr;
}
void TLA20XX_Begin(TLA20XX_Handle_t *dev)
{
    /* Same init value as Arduino library */
    TLA20XX_WriteReg(dev, TLA20XX_CONF_REG, 0x8683);
}

void TLA20XX_WriteReg(TLA20XX_Handle_t *dev, uint8_t reg, uint16_t data)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (data >> 8) & 0xFF;
    buf[2] = data & 0xFF;

    HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr,
                            buf, 3, HAL_MAX_DELAY);
}

uint16_t TLA20XX_ReadReg(TLA20XX_Handle_t *dev, uint8_t reg)
{
    uint8_t rx[2];

    HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr,
                            &reg, 1, HAL_MAX_DELAY);

    HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr,
                           rx, 2, HAL_MAX_DELAY);

    return (rx[0] << 8) | rx[1];
}

void TLA20XX_SetFSR(TLA20XX_Handle_t *dev, TLA20XX_FSR fsr)
{
    uint16_t conf = TLA20XX_ReadReg(dev, TLA20XX_CONF_REG);
    conf &= ~0x0E00;
    conf |= (fsr << 9);
    TLA20XX_WriteReg(dev, TLA20XX_CONF_REG, conf);
}

void TLA20XX_SetMode(TLA20XX_Handle_t *dev, TLA20XX_MODE mode)
{
    uint16_t conf = TLA20XX_ReadReg(dev, TLA20XX_CONF_REG);
    conf &= ~(1 << 8);
    if (mode == OP_SINGLE)
        conf |= (1 << 8);
    TLA20XX_WriteReg(dev, TLA20XX_CONF_REG, conf);
}

void TLA20XX_SetDR(TLA20XX_Handle_t *dev, TLA20XX_DR rate)
{
    uint16_t conf = TLA20XX_ReadReg(dev, TLA20XX_CONF_REG);
    conf &= ~(0x07 << 5);
    conf |= (rate << 5);
    TLA20XX_WriteReg(dev, TLA20XX_CONF_REG, conf);
}

void TLA20XX_SetMux(TLA20XX_Handle_t *dev, TLA20XX_MUX mux)
{
    uint16_t conf = TLA20XX_ReadReg(dev, TLA20XX_CONF_REG);
    conf &= ~0x7000;
    conf |= (mux << 12);
    TLA20XX_WriteReg(dev, TLA20XX_CONF_REG, conf);
}

int16_t TLA20XX_ReadADC(TLA20XX_Handle_t *dev)
{
    int16_t raw = (int16_t)TLA20XX_ReadReg(dev, TLA20XX_CONV_REG);
    return raw >> 4;
}



