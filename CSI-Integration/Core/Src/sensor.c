/*
 * sensor.c
 *
 *  Created on: Sep 15, 2022
 *      Author: Benson
 */
#include "sensor.h"



//need to divide by 100 for mv
// currently in integers
void init_sensor(struct_Sensor *sensor){
	uint32_t count = 0;

	sensor->start_address = S08_EEPROM_START_ADDRESS_X;
	sensor->end_address = 0;
	sensor->voltage = 0;
	sensor->physValue = 0;

	//setting the x values increments of 103mV up to 3296mV or approx 3.3v
	for(int i = 0; i < 32; i++){
		sensor->x_values[i] = count;
		count += 103;
	}

	count = 0;

	//setting the y values in increments of 3 degrees Celsius up to 96 C
	for(int i = 0; i < 32; i++){
		sensor->y_values[i] = count;
		count += 3;
	}



}
