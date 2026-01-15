/*
 * ph_sensor_Japan.c
 *
 *  Created on: Jan 15, 2026
 *      Author: LENOVO
 */


#include "sensor_common.h"
#include "Do_sensor.h"
#include "ph_calculation.h"

#define zero_degree_Ph_temp_Japan 1464
#define step_size_ph_temp_Japan 5.53846f


uint16_t adc_val_Japan = 0;
uint32_t adc_sum_Japan = 0;
uint16_t adc_avg_Japan = 0;
uint16_t sample_count_Japan = 0;
uint32_t adc_val_Japan_temp = 0;
//TLA20XX_Handle_t tla;

float voltage_val_Japan = 0;
float pH_value_2p_Japan = 0;
float pH_value_3p_Japan = 0;
char msg_ph_Japan[40];
int temp_int_Japan;
char uart_buf_Japan[32];
int temp_frac_Japan;


float ph_sensor_Japan_temp(uint32_t adc)
{
	 return (adc - zero_degree_Ph_temp_Japan) / step_size_ph_temp_Japan;
}

void ph_Japan_Runnnig_temp(void)
{
	TLA20XX_SetMux(&tla, MUX_AIN3_GND);
	 int adc_val_japan_temp = TLA20XX_ReadADC(&tla);
	float value = ph_sensor_Japan_temp(adc_val_japan_temp);

	 temp_int_Japan  = (int)value;
	 temp_frac_Japan = (int)((value - temp_int_Japan) * 100); // 2 decimal places
	if (temp_frac_Japan < 0) temp_frac_Japan = -temp_frac_Japan;
//	char uart_buf[32];
	sprintf(uart_buf_Japan,
	        "ADC: %d  Temp: %d.%02d C\r\n",
			adc_val_japan_temp,
			temp_int_Japan,
			temp_frac_Japan);

//	HAL_UART_Transmit(&huart2,
//	                  (uint8_t*)uart_buf,
//	                  strlen(uart_buf),
//	                  HAL_MAX_DELAY);

}


void ph_Japan_running(void)
{
    TLA20XX_SetMux(&tla, MUX_AIN2_GND);
    adc_val_Japan = TLA20XX_ReadADC(&tla);


    adc_sum_Japan += adc_val_Japan;
    sample_count_Japan++;

    if (sample_count_Japan >= 100)
    {

    	adc_avg_Japan = adc_sum_Japan / sample_count_Japan;
    	adc_sum_Japan = 0;
    	sample_count_Japan = 0;

    	voltage_val_Japan= voltage_for_ph(adc_avg_Japan);

    	pH_value_2p_Japan = Calculate_pH_2Point(
    			voltage_val_Japan, V1_PH4, PH1_VALUE,
                        V3_PH918, PH3_VALUE);

    	pH_value_3p_Japan = Calculate_pH_3Point(
    			voltage_val_Japan,
                        V1_PH4, PH1_VALUE,
                        V2_PH686, PH2_VALUE,
                        V3_PH918, PH3_VALUE);

        sprintf(msg_ph_Japan, "pH: %.2f | Temp: %d.%02d C\r\n",
        		pH_value_3p_Japan, temp_int_Japan,temp_frac_Japan);

        HAL_UART_Transmit(&huart2,(uint8_t*)msg_ph_Japan,strlen(msg_ph_Japan),HAL_MAX_DELAY);
    }
}


void ph_sensor_Japan_setup(void)
{
	TLA20XX_SetFSR(&tla, FSR_2_048V);
	TLA20XX_SetMode(&tla, OP_CONTINUOUS);
}

