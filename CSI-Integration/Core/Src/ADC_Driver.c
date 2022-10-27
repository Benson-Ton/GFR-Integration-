/*
 * ADC_Driver.c
 *
 *  Created on: Sep 24, 2022
 *      Author: Benson
 */
#include "ADC_Driver.h"



/*
 * converting the raw adc values to voltage values
 *
 * Note: assuming 3.3v and returns milivolts
 * */
int converted_ADC_values(uint32_t raw_ADC_val){

	return (double)(3.3/4095) * raw_ADC_val * 1000;

}

sint32 VoltageToPhysValue(void){

	sint32 PhysValue = 0;

	uint16_t voltage = sensor.voltage;


	int index = 0;

	//shift index when voltage is higher than the x position
    while((voltage > sensor.x_values[index]) && (index < 31))
    {
        index++;
    }
    //shift index when the voltage is lower than the x position
    while((voltage < sensor.x_values[index]) && (index > 0))
    {
        index--;

    }

    //interpolating between x_values[index] and x_values[index+1]
    //						y_values[index] and y_values[index+1]
	PhysValue = sensor.y_values[index] + (sensor.y_values[index+1] - sensor.y_values[index])
		        * (voltage - sensor.x_values[index]) / (sensor.x_values[index+1] - sensor.x_values[index]);



	return PhysValue;
}
