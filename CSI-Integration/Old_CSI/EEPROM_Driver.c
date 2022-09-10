//#include "EEPROM_Driver.h"
//#include "led.h"
#include "Sensor.h"
#include "CAN_Driver.h"
#include "EEPROM_Driver.h"


/*EEPROM adress from 0x7FF000 to 0x7FFFFE*/

void write_EEPROM (struct_Sensor *Sensor)// uint16 Start_Address, uint16* x_Values, sint32* y_Values, sint32 offset, sint32 faktor, uint8 CON_mode)
{
    uint16_t values[16];
    uint8_t i,j;
    uint16_t address = Sensor->Start_Address; //what address you are writing to

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

//DASH?

void write_D_EEPROM (struct_D_Sensor* Sensor)
{
    uint16_t values[16];

    values[0] = Sensor->CON_mode;
    values[1] = Sensor->NumbTeeth;
    values[2] = Sensor->radius;
    values[3] = Sensor->SensorIDchar;
    values[4] = (Sensor->SensorIDchar)>>16;
    
    uint8_t i;
    for(i = 5; i < 16; i++)
    {
        values[i] = 0;
    }

    write_Block(Sensor->Start_Address, values);
}

void write_config_ID_EEPROM (uint16_t* cfgID, uint32_t* cfg_str, uint16_t* cfg_version)
{
    write_Word(CSI_CONFIG_ID_EEPROM_ADDRESS, *cfgID);
    write_Word(CSI_CONFIG_ID_EEPROM_ADDRESS+2, (*cfg_str)&0xFFFF);
    write_Word(CSI_CONFIG_ID_EEPROM_ADDRESS+4, ((*cfg_str)>>16)&0xFFFF);
    write_Word(CSI_CONFIG_ID_EEPROM_ADDRESS+6, *cfg_version);
}

void read_config_ID_EEPROM (uint16_t* cfgID, uint32_t* cfg_str, uint16_t* cfg_version)
{
    *cfgID = read_Word(CSI_CONFIG_ID_EEPROM_ADDRESS);
    *cfg_str = (read_Word(CSI_CONFIG_ID_EEPROM_ADDRESS+2)&0xFFFF) | ((uint32_t)(read_Word(CSI_CONFIG_ID_EEPROM_ADDRESS+4)&0xFFFF) << 16);
    *cfg_version = read_Word(CSI_CONFIG_ID_EEPROM_ADDRESS+6);
}

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

void read_EEPROM (struct_Sensor* Sensor)
{
    uint16_t address = 0;
    address = Sensor->Start_Address;
    uint8_t i;

    for(i = 0; i < 32; i++)
    {
        Sensor->x_Values[i] = read_Word(address);
        address += 2;
    }
    for(i = 0; i < 32; i++)
    {
        Sensor->y_Values[i] = read_Word(address);
        address += 2;
        Sensor->y_Values[i] |= ((int32_t)read_Word(address) << 16);
        address += 2;
    }
    Sensor->offset = read_Word(address);
    address += 2;
    
    Sensor->faktor = read_Word(address);
    address += 2;

    Sensor->CON_mode = read_Word(address);
    address += 2;
        
    Sensor->SensorIDchar = read_Word(address);
    address += 2;
    Sensor->SensorIDchar = (((uint32_t)read_Word(address)) << 16);

#ifdef DEBUG
    struct_CAN_Message msg;
    msg.CAN_ID = 0x49;
    msg.CAN_Word1 = Sensor->offset;
    msg.CAN_Word2 = Sensor->faktor;
    msg.CAN_Word3 = Sensor ->CON_mode;
    msg.CAN_Word4 = Sensor->y_Values[0];
    send_CAN(msg);
#endif
}

uint16_t read_Word(uint16_t address)
{
    // Write Start Address of EEProm to requested Address
    address += ADDRESS_OFFSET;

    // Write upper 8 Bits of address (for EEProm always 0x7F) to upper Address register
    TBLPAG = HIGH_ADDRESS; // /*change */ <<<<<<<<<<<<<<<<<<<<<

    uint16_t EEProm_Value = __builtin_tblrdl(address); // /*change */ <<<<<<<<<<<<<<<<<<<<<
//    struct_CAN_Message msg;
//    msg.CAN_ID = 0x47;
//    msg.CAN_Word1 = EEProm_Value;
//    msg.CAN_Word2 = address;
//    send_CAN(msg);
//    // Read and return Value saved to Address (read one word)
    return EEProm_Value;
}

void write_Word(uint16_t address, uint16_t value)
{
     /*delete word*/
    NVMADRU = HIGH_ADDRESS;             /*address high */ // /*change */ <<<<<<<<<<<<<<<<<<<<<
    NVMADR = address + ADDRESS_OFFSET;  /*~ low + offset*/ // /*change */ <<<<<<<<<<<<<<<<<<<<<

    NVMCON = ERASE_WORD;                /*select eeprom word erase*/ // /*change */ <<<<<<<<<<<<<<<<<<<<<

    NVMCONbits.WREN = 1;                /*enable Write Operation on EEPROM*/ // /*change */ <<<<<<<<<<<<<<<<<<<<<
    UNLOCK_START_SEQUENCE();
    while(NVMCONbits.WR);               /*wait for operation to finish / 2ms */ // /*change */ <<<<<<<<<<<<<<<<<<<<<
 
    /*write to EEPROM*/
    TBLPAG = HIGH_ADDRESS;              /*set address*/ // /*change */ <<<<<<<<<<<<<<<<<<<<<
    address += ADDRESS_OFFSET;          /*~*/
    __builtin_tblwtl(address, value);   /*load data into table*/ // /*change */ <<<<<<<<<<<<<<<<<<<<<

    NVMCON = WRITE_WORD;                /*select eeprom for single write word op*/
    
    UNLOCK_START_SEQUENCE();            /*unlock + start Operation*/

    /*reset Watchdog to get some time for EEPROM writing*/
  //  CLRWDT();

    while(NVMCONbits.WR) { }            /*wait for write operation to finish / 2ms*/
    NVMCONbits.WREN = 0;                /*WREN should kept clear*/
}



// WHAT DOES THIS DO??

/* workaround for write_Block function
 * 'true' write Block doesn't work as it is atm
 * workaround my result in timelag up to 140ms
 */



