#ifndef EEPROM_DRIVER_H
#define	EEPROM_DRIVER_H

#include "global.h"
#include "Sensor.h"

#define WRITE_WORD 0x4004
#define WRITE_BLOCK 0x400A
#define ERASE_WORD 0x4044
#define ERASE_BLOCK 0x4045

#define HIGH_ADDRESS 0x7F
#define ADDRESS_OFFSET 0xFC00
#define EEPROM_STARTADRESS 0x7FF000

//STM32: M95M01-DF-M95M01-R EEPROM (SPI)

#define WRITE_EN	0x06	//ENABLE WRITING
#define WRITE_DIS	0x04	//DISABLE WRITING
#define WRITE_MEM	0x02	//WRITE TO MEMORY
#define READ_MEM	0x03	//READ FROM MEMORY
#define READ_STATUS 0x05	//READ STATUS REGISTER


/*Unlock sequence for access to eeprom write operation + Start Operation*/

//dont know what this does

// a sequence to start writing to the internal eeprom of the dspic
/*
#define UNLOCK_START_SEQUENCE() __builtin_disi(5);\
                                NVMKEY = 0x55;    \
                                NVMKEY = 0xAA;    \
                                NVMCONbits.WR = 1;
*/
//EEPROM addressing starts with 0

void write_EEPROM (struct_Sensor* Sensor); //uint16 Start_Address, uint16* x_Values, sint32* y_Values, sint32 offset, sint32 faktor, uint8 CON_mode);
void read_EEPROM (struct_Sensor* Sensor);
void write_D_EEPROM (struct_D_Sensor* Sensor);
void read_D_EEPROM (struct_D_Sensor* Sensor);
void write_config_ID_EEPROM (uint16_t* cfgID, uint32_t* cfg_str, uint16_t* cfg_version);
void read_config_ID_EEPROM (uint16_t* cfgID, uint32_t* cfg_str, uint16_t* cfg_version);

uint16_t read_Word(uint16_t address);
void write_Word(uint16_t address, uint16_t value);
void write_Block(uint16_t start_address, uint16_t *values);

#endif

