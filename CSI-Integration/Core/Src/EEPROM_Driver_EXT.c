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


/*
 * The Page Write mode allows up to 256 byte to be written in a single Write cycle, provided
 * that they are all located in the same page in the memory: that is, the most significant
 * memory address bits, b16-b8, are the same.
 *
 * Note: You can use this function to overwrite a specific address
 * */
HAL_StatusTypeDef Write_Page_EEPROM(S08 *device, uint32_t dest_address, uint16_t data){

	uint16_t upper_byte = 0, lower_byte = 0;
	uint8_t temp[4] = {0};
	uint8_t id = 0;
	HAL_StatusTypeDef error_check;

	id = ((dest_address >> 10) << 1) | S08_DEV_ID_ADDR_W; // A16 for 17 - bit to get into one byte

	//parsing the data to keep it within 1 byte
	upper_byte = (data & 0xFF00) >> 8;	//need to keep it size of 1 byte
	lower_byte = data & 0x00FF;

	//sending 4 bytes
	temp[0] = dest_address >> 8; // most significant byte| A15:A8
	temp[1] = dest_address & 0xFF; // least significant byte| A7:A0
	temp[2] = upper_byte;
	temp[3] = lower_byte;


	//make sure the device is ready to communicate before transmitting
	while(HAL_I2C_IsDeviceReady(device->i2cHandle1, id, 10, 100) != HAL_OK);

	//transmit data
	error_check = HAL_I2C_Master_Transmit(device->i2cHandle1, id, temp, 4, HAL_MAX_DELAY);

	return error_check;


}


uint8_t * Read_Page_EEPROM(S08 *device, uint32_t dest_address){


	uint8_t temp[2] = {0};
	uint8_t id = 0;
	static uint8_t data[2] = {0};

	id = ((dest_address >> 10) << 1) | S08_DEV_ID_ADDR_R; // A16 for 17 - bit to get into one byte

	temp[0] = dest_address >> 8; // most significant byte| A15:A8
	temp[1] = dest_address & 0xFF; // least significant byte| A7:A0


	//make sure device is ready before transmitting
	while(HAL_I2C_IsDeviceReady(device->i2cHandle1, id, 10, 100) != HAL_OK);

	if(HAL_I2C_Master_Transmit(device->i2cHandle1, id, temp, 2, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}

	//make sure device is ready before transmitting
	while(HAL_I2C_IsDeviceReady(device->i2cHandle1, id, 10, 100) != HAL_OK);

	if(HAL_I2C_Master_Receive(device->i2cHandle1, id, data, 2, HAL_MAX_DELAY) != HAL_OK){
		Error_Handler();
	}


	return data;
}


/*
 * Writing all of the current Sensor structs' x and y values to the EEPROM
 *
 * */
void write_ALL_EEPROM(struct_Sensor  sensor, S08 *device){

	uint32_t address = sensor.start_address;


	//writing to array of 32 points for the x values
	for(int i = 0; i < 32; i++){
		if(Write_Page_EEPROM(device, address, sensor.x_values[i]) != HAL_OK )
			Error_Handler();
		//address += 0x05;
		address += 2;
	}
	//maybe add the end address here to track

	//address = sensor.end_address; //starting at 0x180

	//increment for a different address for the y values
	address += 5;

	//writing to array of 32 points for the y values
	for(int i = 0; i < 32; i++){
		if(Write_Page_EEPROM(device, address, sensor.y_values[i]) != HAL_OK)
			Error_Handler();
		//address += 0x05;
		address += 2;
	}

}
/*
 *  Reading from EEPROM and updating the sensor struct's x and y values
 * */
void read_ALL_EEPROM(struct_Sensor * sensor, S08 *device){

	  uint32_t address = sensor->start_address;
	  uint8_t *p;
	  uint32_t temp_buffer[3];

	  //reading to array of 32 points for the x values
	  for(int i = 0; i < 32; i++)
	  {
	  	  p =  Read_Page_EEPROM(device, address);
	  	  temp_buffer[0] = *p << 8; //upper byte
	  	  temp_buffer[1] = *(p+1);	//lower byte
	  	  temp_buffer[3] = temp_buffer[0] | temp_buffer[1]; //result

	  	sensor->x_values[i] = temp_buffer[3];
	  	//address += 0x05;
	  	address += 2;
	  }

	  //address = sensor->end_address; //starting at 0x180
	  address += 5;

	  for(int i = 0; i < 32; i++)
	  {
	  	  p =  Read_Page_EEPROM(device, address);
	  	  temp_buffer[0] = *p << 8; //upper byte
	  	  temp_buffer[1] = *(p+1);	//lower byte
	  	  temp_buffer[3] = temp_buffer[0] | temp_buffer[1]; //result

	  	sensor->y_values[i] = temp_buffer[3];
	  	//address += 0x05;
	  	address += 2;
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

	//maybe add another temp[3] for 16 bits when parsing data

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

	//transmit a message to the address of the device it is trying to communicate to
	bool = HAL_I2C_Master_Transmit(device->i2cHandle1, id, temp, 2, HAL_MAX_DELAY);

	if(bool != HAL_OK){return HAL_ERROR;}

	else{return HAL_I2C_Master_Receive(device->i2cHandle1, id, data, 1, HAL_MAX_DELAY);}
}



