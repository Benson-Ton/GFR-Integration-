/*
 * EEPROM_Driver_EXT.c
 *
 *  Created on: Mar 22, 2022
 *      Author: Benson
 */

#include "CAN_Driver.h"
#include "EEPROM_Driver_EXT.h"
#include "sensor.h"


//initialization
void initialize_EEPROM(S08 * device, I2C_HandleTypeDef *i2cHandle1){
	device->i2cHandle1 = i2cHandle1;
	device->dest_address = 0x00;
	device->data = 0x00;

}

/* Writing to Sensor */
void write_EEPROM(struct_Sensor  sensor, S08 *device){
	uint32_t address = sensor.start_address;



	for(int i = 0; i < 32; i++){
		if(S_Write_EEPROM(device, address, sensor.x_values[i]) != HAL_OK )
			Error_Handler();
		address += 0x10;
	}
	//maybe add the end address here to track

	address = S08_EEPROM_START_ADDRESS_Y;
	for(int i = 0; i < 32; i++){
		if(S_Write_EEPROM(device, address, sensor.y_values[i]) != HAL_OK)
			Error_Handler();
		address += 0x10;
	}

}


//create a separate function you when want to change a specific grid in the EEPROM after the if statement for the matching ID


/* low level functions */
HAL_StatusTypeDef S_Write_EEPROM(S08 *device, uint32_t dest_address, uint8_t data){

	uint8_t temp[3] = {0};
	uint8_t id = 0;
	HAL_StatusTypeDef error_check;

	id = ((dest_address >> 10) << 1) | S08_DEV_ID_ADDR_W; // A16 for 17 - bit to get into one byte

	temp[0] = dest_address >> 8; // most significant byte| A15:A8
	temp[1] = dest_address & 0xFF; // least significant byte| A7:A0
	temp[2] = data;

	while(HAL_I2C_IsDeviceReady(device->i2cHandle1, id, 10, 100) != HAL_OK);	//make sure the device is ready to communicate before transmitting

	error_check = HAL_I2C_Master_Transmit(device->i2cHandle1, id, temp, 3, HAL_MAX_DELAY);



	return error_check;
}

HAL_StatusTypeDef S_Read_EEPROM(S08 *device, uint32_t dest_address, uint8_t *data){

	HAL_StatusTypeDef bool;
	uint8_t temp[2] = {0};
	uint8_t id = 0;

	id = ((dest_address >> 10) << 1) | S08_DEV_ID_ADDR_R; // A16 for 17 - bit to get into one byte

	temp[0] = dest_address >> 8; // most significant byte| A15:A8
	temp[1] = dest_address & 0xFF; // least significant byte| A7:A0

	bool = HAL_I2C_Master_Transmit(device->i2cHandle1, id, temp, 2, HAL_MAX_DELAY);

	if(bool != HAL_OK){return HAL_ERROR;}

	else{return HAL_I2C_Master_Receive(device->i2cHandle1, id, data, 1, HAL_MAX_DELAY);}
}



