/*
 * sensor.h
 *
 *  Created on: Sep 15, 2022
 *      Author: Benson
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
#include "global.h"
#include <string.h>
#include <stdio.h>


// MACROS

#define S08_EEPROM_START_ADDRESS_X 0x00
#define S08_EEPROM_START_ADDRESS_Y 0x180



typedef struct{
	uint16_t 	x_values[32];
	sint32 		y_values[32];
	uint8_t 	prev_index;
	uint16_t 	voltage;
	uint32_t 	physValue;
	uint32_t 	start_address;
	uint32_t 	end_address;

}struct_Sensor;


void init_sensor(struct_Sensor *sensor);
void serial_print_sensor_values(struct_Sensor sensor);

#endif /* INC_SENSOR_H_ */
