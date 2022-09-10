#include "Sensor.h"
//#include "ADC_Driver.h"
#include "EEPROM_Driver_EXT.h"
#include "CAN_Driver.h"
//#include "Runtime.h"
#include "FIFO.h"

struct_Sensor Sensors[4];
struct_D_Sensor D_Sensor;
//extern uint16_t CSI_config_ID;
uint32_t CSI_config_str;
uint16_t CSI_config_version;

uint32_t D_Sens_filter_array[5];
uint8_t D_sens_filter_act_i;

void Init_Sensors (void)
{
    uint8 counter, counter2;

/*	already declared in the main.c under 'MX_ADC1_Init(void)' function*/

//    Sensors[0].ADC_PIN = ANA_SENS1_ADC_PIN;
//    Sensors[1].ADC_PIN = ANA_SENS2_ADC_PIN;
//    Sensors[2].ADC_PIN = ANA_SENS3_ADC_PIN;
//    Sensors[3].ADC_PIN = ANA_SENS4_ADC_PIN;
//
    Sensors[0].Start_Address = ANA_SENS1_EEPROM_STARTADDRESS;	//this constant value is 0
//    Sensors[1].Start_Address = ANA_SENS2_EEPROM_STARTADDRESS;
//    Sensors[2].Start_Address = ANA_SENS3_EEPROM_STARTADDRESS;
//    Sensors[3].Start_Address = ANA_SENS4_EEPROM_STARTADDRESS;

    /**************************************************/
    for (counter = 0; counter < 4; counter++)
    {
        // Reset Sensor
        Sensors[counter].connected = TRUE;
        Sensors[counter].physValue = 0;
        Sensors[counter].voltage = 0;
        Sensors[counter].offset = 0;
        Sensors[counter].faktor = 1;
       // Sensors[counter].CON_mode = CON_MODE_ANA;
        Sensors[counter].prevIndex = 0;

        for (counter2 = 0; counter2 < 32; counter2++)
        {
            Sensors[counter].x_Values[counter2] = 0;//counter2;
            Sensors[counter].y_Values[counter2] = 0; //counter2;
        }

        // Readout EEProm
        read_EEPROM(&Sensors[counter]);
    }

 //   D_Sensor.Start_Address = D_SENSOR_EEPROM_STARTADDRESS;
    D_Sensor.NumbTeeth = 0;
    D_Sensor.radius = 0;
    D_Sensor.speed = 0;
    D_Sensor.speed_filtered = 0;



    //Readout EEPROM

//    read_D_EEPROM(&D_Sensor);
//    D_Sens_filter_array[0] = 0xFFFF;
//    D_Sens_filter_array[1] = 0xFFFF;
//    D_Sens_filter_array[2] = 0xFFFF;
//    D_Sens_filter_array[3] = 0xFFFF;
//    D_Sens_filter_array[4] = 0xFFFF;
//    D_sens_filter_act_i = 0;

}



