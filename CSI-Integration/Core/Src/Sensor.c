/*
 * sensor.c
 *
 *  Created on: Sep 15, 2022
 *      Author: Benson
 */
#include "sensor.h"

extern UART_HandleTypeDef huart3;

//need to divide by 100 for mv
// currently in integers
/* ONLY USE THIS FOR EEPROM RESET */
void init_sensor(struct_Sensor *sensor){
	uint32_t count = 0;

	sensor->start_address = S08_EEPROM_START_ADDRESS_X;
	sensor->end_address = S08_EEPROM_START_ADDRESS_Y;
	sensor->voltage = 0;
	sensor->physValue = 0;

	//setting the x values increments of 103mV up to 3296mV or approx 3.3v
	for(int i = 0; i < 32; i++){
		sensor->x_values[i] = count;
		count += 106;
	}

	count = 0;	//reset the count for y intervals

	//setting the y values in increments of 3 degrees Celsius up to 96 C
	for(int i = 0; i < 32; i++){
		sensor->y_values[i] = count;
		count += 3;
	}
}

// HAL_StatusTypeDef HAL_UART_Transmit (UART_HandleTypeDef * huart, uint8_t * pData, uint16_t Size, uint32_t Timeout)

void serial_print_sensor_values(struct_Sensor sensor)
{

	char uart_buf[50] = {0};
//	char values_buffer[20];
	uint16_t temp[2];

	for(int i = 0; i < 32; i++)
	{
		temp[0] = sensor.x_values[i];
		sprintf(uart_buf, "x values: %d \r\n ", temp[0]);
		HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, strlen(uart_buf), 100);
	}

	for(int i = 0; i < 32; i++)
	{
		temp[1] = sensor.y_values[i];
		sprintf(uart_buf, "y values: %d \r\n ", temp[1]);
		HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, strlen(uart_buf), 100);
	}

}
