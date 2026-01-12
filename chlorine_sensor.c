/*
 * chlorine_sensor.c
 *
 *  Created on: Dec 31, 2025
 *      Author: LENOVO
 */

#include "sensor_common.h"
#include "chlorine_sensor.h"

/* ===== Calibration ===== */
static float chlorine_zero_voltage = 0.394f;
static float chlorine_slope = 12.5f;

char msg_cl[50];

static float Calculate_Voltage(int raw)
{
    return raw * 0.001f;
}

static float Calculate_chlorine(float volt)
{
    float Vsignal = volt - chlorine_zero_voltage;
    if (Vsignal < 0) Vsignal = 0;
    return Vsignal * chlorine_slope;
}

void chlorine_calculation(void)
{
    TLA20XX_SetMux(&tla, MUX_AIN3_GND);
    int16_t adc = TLA20XX_ReadADC(&tla);

    float voltage  = Calculate_Voltage(adc);
    float chlorine = Calculate_chlorine(voltage);

    snprintf(msg_cl, sizeof(msg_cl),
             "ADC=%d | V=%.3f | Cl=%.2f ppm\r\n",
             adc, voltage, chlorine);

    HAL_UART_Transmit(&huart2,
                      (uint8_t*)msg_cl,
                      strlen(msg_cl), 100);
}
void chlorine_setup(void)
{
    TLA20XX_SetFSR(&tla, FSR_2_048V);
    TLA20XX_SetMode(&tla, OP_CONTINUOUS);
}


