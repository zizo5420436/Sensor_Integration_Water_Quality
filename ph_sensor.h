/*
 * ph_sensor.h
 *
 *  Created on: Dec 31, 2025
 *      Author: LENOVO
 */

#ifndef INC_PH_SENSOR_H_
#define INC_PH_SENSOR_H_
void  ph_setup(void);
void  ph_running(void);
float voltage_for_ph(uint16_t val);

extern float pH_value_2p;
extern float pH_value_3p;
extern char msg_ph[64];

#endif /* INC_PH_SENSOR_H_ */
