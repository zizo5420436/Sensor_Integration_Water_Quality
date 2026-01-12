/*
 * ph_sensor.c
 *
 *  Created on: Dec 31, 2025
 *      Author: LENOVO
 */
#include "sensor_common.h"
#include "Do_sensor.h"

#define PH1_VALUE   4.00f
#define PH2_VALUE   6.86f
#define PH3_VALUE   9.18f

#define V1_PH4      1.770f
#define V2_PH686    1.639f
#define V3_PH918    1.496f

/* ===== Globals ===== */
uint16_t adc_val = 0;
uint32_t adc_sum = 0;
uint16_t adc_avg = 0;
uint16_t sample_count = 0;

float voltage_val = 0;
float pH_value_2p = 0;
float pH_value_3p = 0;

char msg_ph[50];

static float Calculate_pH_2Point(float V, float V1, float pH1,
                                 float V2, float pH2)
{
    float slope = (pH2 - pH1) / (V2 - V1);
    return slope * V + (pH1 - slope * V1);
}

static float Calculate_pH_3Point(float V,
                                 float V1, float pH1,
                                 float V2, float pH2,
                                 float V3, float pH3)
{
    float A = (pH1*(V2 - V3) + pH2*(V3 - V1) + pH3*(V1 - V2)) /
              ((V1 - V2)*(V1 - V3)*(V2 - V3));

    float B = (pH1*(V3*V3 - V2*V2) +
               pH2*(V1*V1 - V3*V3) +
               pH3*(V2*V2 - V1*V1)) /
              ((V1 - V2)*(V1 - V3)*(V2 - V3));

    float C = (pH1*V2*V3*(V2 - V3) +
               pH2*V1*V3*(V3 - V1) +
               pH3*V1*V2*(V1 - V2)) /
              ((V1 - V2)*(V1 - V3)*(V2 - V3));

    return (A*V*V + B*V + C);
}

float voltage_for_ph(uint16_t val)
{
    return val * 0.001f;
}

void ph_running(void)
{
    TLA20XX_SetMux(&tla, MUX_AIN2_GND);
    adc_val = TLA20XX_ReadADC(&tla);

    adc_sum += adc_val;
    sample_count++;

    if (sample_count >= 1500)
    {
        adc_avg = adc_sum / sample_count;
        adc_sum = 0;
        sample_count = 0;

        voltage_val = voltage_for_ph(adc_avg);

        pH_value_2p = Calculate_pH_2Point(
                        voltage_val, V1_PH4, PH1_VALUE,
                        V3_PH918, PH3_VALUE);

        pH_value_3p = Calculate_pH_3Point(
                        voltage_val,
                        V1_PH4, PH1_VALUE,
                        V2_PH686, PH2_VALUE,
                        V3_PH918, PH3_VALUE);

        sprintf(msg_ph, "pH: %.2f | V=%.3f\r\n",
                pH_value_3p, voltage_val);
    }
}

void ph_setup(void)
{
    TLA20XX_SetFSR(&tla, FSR_4_096V);
    TLA20XX_SetMode(&tla, OP_CONTINUOUS);
}
