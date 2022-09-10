/*
 * EEPROM_Driver_EXT.c
 *
 *  Created on: Mar 22, 2022
 *      Author: Benson
 */

#include "Sensor.h"
#include "CAN_Driver.h"
#include "EEPROM_Driver_EXT.h"


//initialization
void initialize_EEPROM(S08 * device, I2C_HandleTypeDef *i2cHandle1){
	device->i2cHandle1 = i2cHandle1;
	device->dest_address = 0x00;
	device->data = 0x00;

}

/* low level functions */
HAL_StatusTypeDef S_Write_EEPROM(S08 *device, uint32_t dest_address, uint8_t data){

	uint8_t temp[3] = {0};
	uint8_t id = 0;

	id = ((dest_address >> 10) << 1) | S08_DEV_ID_ADDR_W; // A16 for 17 - bit to get into one byte

	temp[0] = dest_address >> 8; // most significant byte| A15:A8
	temp[1] = dest_address & 0xFF; // least significant byte| A7:A0
	temp[2] = data;

	return HAL_I2C_Master_Transmit(device->i2cHandle1, id, temp, 3, HAL_MAX_DELAY);
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


/*	new eeprom function with integration v1
 * void S_Write_EEPROM(SO8 *device, uint32_t dest_address, uint8_t data){
 * 		uint16_t values[16];
 *
 *
 *
 * }
 */


//void read_D_EEPROM (struct_D_Sensor *Sensor)
//{
//    uint16_t address = Sensor->Start_Address;
//    Sensor->CON_mode = read_Word(address);
//    address += 2;
//    Sensor->NumbTeeth = read_Word(address);
//    address += 2;
//    Sensor->radius = read_Word(address);
//    address += 2;
//    Sensor->SensorIDchar = read_Word(address);
//    address += 2;
//    Sensor->SensorIDchar = (((uint32_t)read_Word(address)) << 16);
//}


//OLD CSI FUNCTIONS

//need to recreate this function
/*
void read_D_EEPROM (struct_D_Sensor *Sensor)
{
    uint16_t address = Sensor->Start_Address;
    Sensor->CON_mode = read_Word(address);
    address += 2;
    Sensor->NumbTeeth = read_Word(address);
    address += 2;
    Sensor->radius = read_Word(address);
    address += 2;
    Sensor->SensorIDchar = read_Word(address);
    address += 2;
    Sensor->SensorIDchar = (((uint32_t)read_Word(address)) << 16);
}


void write_EEPROM (struct_Sensor* Sensor){

	 uint16_t values[16];
	    uint8_t i,j;
	    uint16_t address = Sensor->Start_Address;

	    // 1st 16 xValues to EEProm
	    for(i = 0; i < 16; i++)
	    {
	        values[i] = Sensor->x_Values[i];
	    }
	    write_Block(address, values); // write
	    address += 32;                // increment WriteAddress

	    // 2nd 16 xValues to EEProm
	    for(i = 0; i < 16; i++)
	    {
	        values[i] = Sensor->x_Values[i+16];
	    }
	    write_Block(address, values); // write
	    address += 32;                // increment WriteAddress

	    j = 0;
	    for(i = 0; i < 8; i++)
	    {
	        values[j++] = (Sensor->y_Values[i]);   //low bits
	        values[j++] = (Sensor->y_Values[i] >> 16); //high bits
	    }
	    write_Block(address, values);
	    address += 32;
	    j = 0;

	    for(i = 8; i < 16; i++)
	    {
	        values[j++] = (Sensor->y_Values[i]);   //low bits
	        values[j++] = (Sensor->y_Values[i] >> 16); //high bits
	    }
	    write_Block(address, values);
	    address += 32;
	    j = 0;
	    for(i = 16; i < 24; i++)
	    {
	        values[j++] = (Sensor->y_Values[i]);   //low bits
	        values[j++] = (Sensor->y_Values[i] >> 16); //high bits
	    }
	    write_Block(address, values);
	    address += 32;
	    j = 0;

	    for(i = 24; i < 32; i++)
	    {
	        values[j++] = (Sensor->y_Values[i]);   //low bits
	        values[j++] = (Sensor->y_Values[i] >> 16); //high bits
	    }
	    write_Block(address, values);
	    address += 32;

	    values[0] = (Sensor->offset);
	    values[1] = (Sensor->faktor);
	    values[2] = (Sensor->CON_mode);
	    values[3] = Sensor->SensorIDchar;
	    values[4] = (Sensor->SensorIDchar)>>16;
	    for(i = 5; i < 16; i++)
	    {
	        values[i] = 0;
	    }
	    write_Block(address, values);
	}
*/
