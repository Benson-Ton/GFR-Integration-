/*
 * EEPROM_Driver_EXT.h
 *
 *  Created on: Mar 22, 2022
 *      Author: Benson
 */
#ifndef INC_EEPROM_DRIVER_EXT_H_
#define INC_EEPROM_DRIVER_EXT_H_

#include "sensor.h"
#include "global.h"
#include "stm32f7xx_hal.h"




//STM32: M95M01-DF-M95M01-R EEPROM (SPI)

#define WRITE_EN	0x06	//ENABLE WRITING
#define WRITE_DIS	0x04	//DISABLE WRITING
#define WRITE_MEM	0x02	//WRITE TO MEMORY
#define READ_MEM	0x03	//READ FROM MEMORY
#define READ_STATUS 0x05	//READ STATUS REGISTER

//STM32 : M24M01-DFMN6TP EEPROM (I2C) - currently using this EEPROM

#define S08_DEV_ID_ADDR_W		0xAC		//E2 = E1 = 1
#define S08_DEV_ID_ADDR_R		0xAD		//E2 = E1 = 1


typedef struct{

	I2C_HandleTypeDef *i2cHandle1;

	uint8_t dest_address;

	uint8_t data;

} S08;

void write_EEPROM(struct_Sensor sensor, S08 *device);
HAL_StatusTypeDef Write_Page_EEPROM(S08 *device, uint32_t dest_address, uint16_t data);
uint8_t * Read_Page_EEPROM(S08 *device, uint32_t dest_address);

//initialization
void initialize_EEPROM(S08 * device, I2C_HandleTypeDef *i2cHandle1);

//new read and write functions
HAL_StatusTypeDef S_Read_EEPROM(S08 *device, uint32_t dest_address, uint8_t *data);
HAL_StatusTypeDef S_Write_EEPROM(S08 *device, uint32_t dest_address, uint8_t data);




//low level functions
//HAL_StatusTypeDef Write_EEPROM(S08 *device, uint32_t dest_address, uint8_t data);
//HAL_StatusTypeDef Read_EEPROM(S08 *device, uint32_t dest_address, uint8_t *data);








#endif /* INC_EEPROM_DRIVER_EXT_H_ */
