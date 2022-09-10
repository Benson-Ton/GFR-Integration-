#ifndef SENSOR_H

#define	SENSOR_H

#include "global.h"
#include "main.h"


typedef struct
{
    uint8_t ADC_PIN;
    uint16_t Start_Address;
    uint16_t x_Values[32];
    sint32 y_Values[32];
    uint16_t voltage;
    int32_t physValue;
    uint8_t connected; //
    uint8_t prevIndex;
    int16_t offset;
    int16_t faktor;
    uint8_t CON_mode;
    uint32_t SensorIDchar; //storage for 4 chars
    //char 
}struct_Sensor;

typedef struct
{
	uint16_t Start_Address;
	uint16_t radius;
	uint16_t NumbTeeth;
	uint16_t speed;
	uint16_t speed_filtered;
    uint8_t CON_mode;
    uint32_t SensorIDchar;
}struct_D_Sensor;


#define CON_MODE_ANA        0
#define CON_MODE_SPEED      0
#define CON_MODE_DIG        1
#define CON_MODE_DIG_INV    2
#define CON_MODE_SPEED_FILT 3
#define CON_MODE_NO_CFG     0xFF //if EEPROM has no config con mode = 0xFF

//EEPROM addressing starts with 0
#define SENSOR_ADDRESS_WIDTH 2*112
#define D_SENSOR_ADDRESS_WIDTH  2*16

#define ANA_SENS1_PIN_CONFIG TRISBbits.TRISB11
#define ANA_SENS1_PIN LATBbits.LATB11
#define ANA_SENS1_PIN_GET PORTBbits.RB11
#define ANA_SENS1_ADC_PIN 11
#define ANA_SENS1_ADC_SAMPLE ADCHSbits.CH0SA = ANA_SENS1_ADC_PIN
#define ANA_SENS1_EEPROM_STARTADDRESS 0

//output definitions
#define ANA_SENS1_PORT LATB
#define ANA_SENS1_CONFIG        Cbits.RC4
#define ANA_SENS1_ON()        ANA_SENS1_PORT |= (1<<ANA_SENS1_ADC_PIN)
#define ANA_SENS1_OFF()       ANA_SENS1_PORT &= ~(1<<ANA_SENS1_ADC_PIN)


#define ANA_SENS2_PIN_CONFIG TRISBbits.TRISB12
#define ANA_SENS2_PIN LATBbits.LATB12
#define ANA_SENS2_PIN_GET PORTBbits.RB12
#define ANA_SENS2_ADC_PIN 12
#define ANA_SENS2_ADC_SAMPLE ADCHSbits.CH0SA = ANA_SENS2_ADC_PIN
#define ANA_SENS2_EEPROM_STARTADDRESS ANA_SENS1_EEPROM_STARTADDRESS + SENSOR_ADDRESS_WIDTH

//output definitions
#define ANA_SENS2_PORT LATB
#define ANA_SENS2_ON()        ANA_SENS2_PORT |= (1<<ANA_SENS2_ADC_PIN)
#define ANA_SENS2_OFF()       ANA_SENS2_PORT &=~ (1<<ANA_SENS2_ADC_PIN)

#define ANA_SENS3_PIN_CONFIG TRISBbits.TRISB4
#define ANA_SENS3_PIN LATBbits.LATB4
#define ANA_SENS3_PIN_GET PORTBbits.RB4
#define ANA_SENS3_ADC_PIN 4
#define ANA_SENS3_ADC_SAMPLE ADCHSbits.CH0SA = ANA_SENS3_ADC_PIN
#define ANA_SENS3_EEPROM_STARTADDRESS ANA_SENS2_EEPROM_STARTADDRESS + SENSOR_ADDRESS_WIDTH

//output definitions
#define ANA_SENS3_PORT LATB
#define ANA_SENS3_ON()        ANA_SENS3_PORT |= (1<<ANA_SENS3_ADC_PIN)
#define ANA_SENS3_OFF()       ANA_SENS3_PORT &= ~(1<<ANA_SENS3_ADC_PIN)

#define ANA_SENS4_PIN_CONFIG TRISBbits.TRISB5
#define ANA_SENS4_PIN LATBbits.LATB5
#define ANA_SENS4_PIN_GET PORTBbits.RB5
#define ANA_SENS4_ADC_PIN 5
#define ANA_SENS4_ADC_SAMPLE ADCHSbits.CH0SA = ANA_SENS4_ADC_PIN
#define ANA_SENS4_EEPROM_STARTADDRESS ANA_SENS3_EEPROM_STARTADDRESS + SENSOR_ADDRESS_WIDTH

//output definitions
#define ANA_SENS4_PORT LATB
#define ANA_SENS4_ON()        ANA_SENS4_PORT |= (1<<ANA_SENS4_ADC_PIN)
#define ANA_SENS4_OFF()       ANA_SENS4_PORT &= ~(1<<ANA_SENS4_ADC_PIN)

#define OFFSET_PIN_CONFIG TRISBbits.TRISB2
#define OFFSET_PIN LATBbits.LATB2
#define OFFSET_PIN_GET PORTBbits.RB2
#define OFFSET_ADC_PIN 2
#define OFFSET_ADC_SAMPLE ADCHSbits.CH0SA = OFFSET_ADC_PIN

#define D_SENSOR_PIN_CONFIG     TRISDbits.TRISD8
#define D_SENSOR_PIN            LATDbits.LATD8
#define D_SENSOR_PIN_GET()      PORTDbits.RD8
#define D_SENSOR_EEPROM_STARTADDRESS ANA_SENS4_EEPROM_STARTADDRESS + SENSOR_ADDRESS_WIDTH

#define THRESHOLD_DIGITAL_LOGIC 1250 //LSB: 2mV
#define THRESHOLD_OPEN_WIRE     5   //LSB: 2mV

#define CSI_CONFIG_ID_EEPROM_ADDRESS D_SENSOR_EEPROM_STARTADDRESS + D_SENSOR_ADDRESS_WIDTH


#endif	/* SENSOR_H */
