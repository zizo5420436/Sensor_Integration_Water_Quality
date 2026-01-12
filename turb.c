/*
 * turb.c
 *
 *  Created on: Dec 31, 2025
 *      Author: LENOVO
 */

#include "sensor_common.h"
#include "turb.h"

float TURB_ReadNTU(void)
{
    uint8_t buf[4];

    if (Modbus_ReadRegisters_IT(&huart1, 10, 0, 2, buf, false) != 0)
        return -1000;

    uint8_t fbytes[4] = { buf[1], buf[0], buf[3], buf[2] };
    float val;
    memcpy(&val, fbytes, 4);
    return val;
}

