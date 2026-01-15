/*
 * ph_France_running.c
 *
 *  Created on: Jan 13, 2026
 *      Author: LENOVO
 */
#include "sensor_common.h"
#include "Do_sensor.h"
#include "ph_calculation.h"

#define PH1_VALUE   4.00f
#define PH2_VALUE   6.86f
#define PH3_VALUE   9.18f

#define V1_PH4      1.770f
#define V2_PH686    1.639f
#define V3_PH918    1.496f

#define zero_degree_Ph_temp_France 1464
#define step_size_ph_temp_france 5.53846f


uint16_t adc_val_france = 0;
uint32_t adc_sum_france = 0;
uint16_t adc_avg_france = 0;
uint16_t sample_count_france = 0;
uint32_t adc_val_france_temp = 0;

float voltage_val_france = 0;
float pH_value_2p_france = 0;
float pH_value_3p_france = 0;
char msg_ph_france[40];
char uart_buf[32];
int temp_int;
int temp_frac;
float ph_sensor_france_temp(uint32_t adc)
{
	 return (adc - zero_degree_Ph_temp_France) / step_size_ph_temp_france;
}
void ph_France_Runnnig_temp(void)
{
	TLA20XX_SetMux(&tla, MUX_AIN3_GND);
	adc_val_france_temp = TLA20XX_ReadADC(&tla);
	float value = ph_sensor_france_temp(adc_val_france_temp);

	 temp_int  = (int)value;
	 temp_frac = (int)((value - temp_int) * 100); // 2 decimal places
	if (temp_frac < 0) temp_frac = -temp_frac;

	sprintf(uart_buf,
	        "ADC: %ld  Temp: %d.%02d C\r\n",
			adc_val_france_temp,
	        temp_int,
	        temp_frac);



}

void ph_france_running(void)
{
    TLA20XX_SetMux(&tla, MUX_AIN2_GND);
    adc_val_france = TLA20XX_ReadADC(&tla);


    adc_sum_france += adc_val_france;
    sample_count_france++;

    if (sample_count_france >= 100)
    {

    	adc_avg_france = adc_sum_france / sample_count_france;
    	adc_sum_france = 0;
    	sample_count_france = 0;

    	voltage_val_france = voltage_for_ph(adc_avg_france);

    	pH_value_2p_france = Calculate_pH_2Point(
    			voltage_val_france, V1_PH4, PH1_VALUE,
                        V3_PH918, PH3_VALUE);

    	pH_value_3p_france = Calculate_pH_3Point(
    			voltage_val_france,
                        V1_PH4, PH1_VALUE,
                        V2_PH686, PH2_VALUE,
                        V3_PH918, PH3_VALUE);

        sprintf(msg_ph_france, "pH: %.2f | Temp: %d.%02d C\r\n",
        		pH_value_3p_france, temp_int,temp_frac);

        HAL_UART_Transmit(&huart2,(uint8_t*)msg_ph_france,strlen(msg_ph_france),HAL_MAX_DELAY);
    }
}

void ph_sensor_France_setup(void)
{
	TLA20XX_SetFSR(&tla, FSR_2_048V);
	TLA20XX_SetMode(&tla, OP_CONTINUOUS);
}

