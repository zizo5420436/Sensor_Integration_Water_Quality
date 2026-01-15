/*
 * conductivity.h
 *
 *  Created on: Jan 14, 2026
 *      Author: LENOVO
 */

#ifndef INC_CONDUCTIVITY_H_
#define INC_CONDUCTIVITY_H_

#define NUM_SAMPLES 15
#define ADC_MAX_COUNTS 4095.0f
#define ADC_VREF       3.3f
#define R_SHUNT 1000.0f
#define Conductivity_Constant 1
#define zero_degree 695.0f
#define step_size 2.863039f

#include "stm32f3xx_hal.h"
#include <stdint.h>


extern volatile uint8_t adc_done;

extern volatile uint16_t adc_buffer_A0[NUM_SAMPLES];
extern volatile uint16_t adc_buffer_A1[NUM_SAMPLES];
extern  uint16_t adc_buffer_TEMP[NUM_SAMPLES];

extern UART_HandleTypeDef huart2;

void Calculation_Conductivity(void);


//float Voltage_to_Conductance(float voltage_A0, float voltage_A1);
//float ADC_ToVoltage(uint16_t adc);
//float ADC_ToTemperature(uint16_t adc);
//void Calculation_Conductivity(void);

#endif /* INC_CONDUCTIVITY_H_ */
