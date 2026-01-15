/*
 * ph_calculation.h
 *
 *  Created on: Jan 13, 2026
 *      Author: LENOVO
 */

#ifndef INC_PH_CALCULATION_H_
#define INC_PH_CALCULATION_H_

#include <stdint.h>

float Calculate_pH_2Point(float V, float V1, float pH1,
                          float V2, float pH2);

float Calculate_pH_3Point(float V,
                          float V1, float pH1,
                          float V2, float pH2,
                          float V3, float pH3);

float voltage_for_ph(uint16_t val);
void ph_France_Runnnig_temp(void);
#endif /* INC_PH_CALCULATION_H_ */
