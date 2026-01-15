/*
 * ph_calculation.h
 *
 *  Created on: Jan 13, 2026
 *      Author: LENOVO
 */

#ifndef INC_PH_CALCULATION_H_
#define INC_PH_CALCULATION_H_

#include <stdint.h>
#define PH1_VALUE   4.00f
#define PH2_VALUE   6.86f
#define PH3_VALUE   9.18f

#define V1_PH4      1.770f
#define V2_PH686    1.639f
#define V3_PH918    1.496f
float Calculate_pH_2Point(float V, float V1, float pH1,
                          float V2, float pH2);

float Calculate_pH_3Point(float V,
                          float V1, float pH1,
                          float V2, float pH2,
                          float V3, float pH3);

float voltage_for_ph(uint16_t val);
void ph_France_Runnnig_temp(void);
#endif /* INC_PH_CALCULATION_H_ */

