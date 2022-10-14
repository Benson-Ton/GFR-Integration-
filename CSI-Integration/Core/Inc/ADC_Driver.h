/*
 * ADC_Driver.h
 *
 *  Created on: Sep 24, 2022
 *      Author: Benson
 */

#ifndef INC_ADC_DRIVER_H_
#define INC_ADC_DRIVER_H_

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "global.h"
#include "sensor.h"

int converted_ADC_values(uint32_t raw_ADC_val);

sint32 VoltageToPhysValue(struct_Sensor *sensor);


#endif /* INC_ADC_DRIVER_H_ */
