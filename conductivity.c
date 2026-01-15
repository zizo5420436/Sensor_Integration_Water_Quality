/*
 * conductivity.c
 *
 *  Created on: Jan 14, 2026
 *      Author: LENOVO
 */


#include "conductivity.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern char msg[64];
float Voltage_to_Conductance(float voltage_A0, float voltage_A1)
{
    float I;   // current in Amps
    float R;   // resistance in Ohms

    // Current calculation
    I = voltage_A1 / R_SHUNT;   // Ohm's law

    if (I <= 0.0f)
        return 0.0f;            // avoid divide by zero

    // Resistance calculation
    R = (voltage_A0 - voltage_A1) / I;

    if (R <= 0.0f)
        return 0.0f;

    //Conductance(Siemens)
    return (1.0f / R);
}
float ADC_ToVoltage(uint16_t adc)
{
    return (adc * ADC_VREF) / ADC_MAX_COUNTS;
}
float ADC_ToTemperature(uint16_t adc)
{
    return (adc - zero_degree) / step_size;
}

void Calculation_Conductivity(void)
{
	if (adc_done) {
		adc_done = 0;

		uint32_t sum_A0 = 0;
		uint32_t sum_A1 = 0;
		uint32_t sum_TEMP = 0;

		for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
			sum_A0 += adc_buffer_A0[i];
			sum_A1 += adc_buffer_A1[i];
			sum_TEMP += adc_buffer_TEMP[i];

		}
//		 /* ---- A0 samples ---- */
//		snprintf(msg, sizeof(msg), "A0:\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//		for (uint8_t i = 0; i < NUM_SAMPLES; i++)
//		{
//			snprintf(msg, sizeof(msg), "%u ", adc_buffer_A0[i]);
//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//		}
//
//		snprintf(msg, sizeof(msg), "\r\nA1:\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//		for (uint8_t i = 0; i < NUM_SAMPLES; i++)
//		{
//			snprintf(msg, sizeof(msg), "%u ", adc_buffer_A1[i]);
//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//		}
//		snprintf(msg, sizeof(msg), "TEMP:\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//		for (uint8_t i = 0; i < NUM_SAMPLES; i++)
//		{
//			snprintf(msg, sizeof(msg), "%u ", adc_buffer_TEMP[i]);
//			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//		}
//		snprintf(msg, sizeof(msg), "\r\nA1:\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		uint16_t avg_A0 = sum_A0 / NUM_SAMPLES;
		uint16_t avg_A1 = sum_A1 / NUM_SAMPLES;
		uint32_t avg_TEMP = sum_TEMP / NUM_SAMPLES;


		float voltage_A0 = ADC_ToVoltage(avg_A0);
		float voltage_A1 = ADC_ToVoltage(avg_A1);
		float temp = ADC_ToTemperature(avg_TEMP);
//		temp = roundf(temp);

		float G = Voltage_to_Conductance(voltage_A0, voltage_A1);

		float conductance = G * Conductivity_Constant *1e6f;

		// Optional: UART debug
		sprintf(msg,
		        "EC=%.1f uS/cm  TEMP=%.1f\r\n",
		        conductance,
				temp);

//		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
//		HAL_MAX_DELAY);
	}
}

