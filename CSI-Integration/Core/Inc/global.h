#ifndef GLOBAL_H
#define	GLOBAL_H

//#define DEBUG
//#define IDLE_TIMER
#define IDLE_COUNTER_VAL 86890 //5ms  used as basis for percentage of free cycles

/*******************************************************
 **                 Oscillator		                  **
 *******************************************************/
/* Microcontroller MIPs (FCY) */
//#define SYSCLOCK			80000000UL
//#define FCY                 SYSCLOCK/4

/*******************************************************
 **                 Data Types   	                  **
 *******************************************************/

typedef signed char sint8;
typedef unsigned char uint8;
typedef signed int sint16;
typedef unsigned int uint16;
typedef signed long int sint32;
typedef unsigned long int uint32;
typedef signed long long int sint64;
typedef unsigned long long int uint64;


#define TRUE    1
#define FALSE   0

/*Defining ON and OFF Status for LEDs*/
#define ON      1
#define OFF     0

#define INPUT   1
#define OUTPUT  0

/*******************************************************
 **                   Include Section                  **
 *******************************************************/

//#include <libpic30.h>

//#include <p30F4013.h>
//#include <stdint.h> /*Datentypen Definition hier entnehmen*/
//#include <CAN.h>
//#include <spi.h>
//#include "timer.h"
//#include <adc12.h>
//#include "delay.h"

/*******************************************************
 **                 Useful Macros                      **
 *******************************************************/

//#define CLI()		INTCON2bits.DISI = TRUE;
//#define SEI()   	INTCON2bits.DISI = FALSE;
//#define INTSTATE	INTCON2bits.DISI

#define NOP() 		{__asm__ volatile ("nop");}
#define CLRWDT() 	{__asm__ volatile ("clrwdt");}
#define SLEEP()		{__asm__ volatile ("pwrsav #0");}
#define IDLE() 		{__asm__ volatile ("pwrsav #1");}

#define SBIT(port,pin) ((*(volatile struct BitsOfByte*)&port).b##pin)

/* useful macros*/
#define MIN(a,b) ((a)<(b))?(a):(b)
#define MAX(a,b) ((a)>(b))?(a):(b)
#define ABS(a)   ((a)<0)?(-a):(a)

//#define CONCATx(a,b) a##b
//#define CONCAT(a,b) CONCATx(a,b)

extern uint16 CSI_config_ID; //need this for CAN ID defines

/* Define CSI_ID 0x0-0xF*/
#define CSI_ID 1 //hardware ID

// CSI CAN IDs TX
#define ID_STATUS       ((0x600) | (0x0F0&(CSI_config_ID<<4)))
#define ID_ANA_VAL12    ((0x601) | (0x0F0&(CSI_config_ID<<4)))     //send Analog Value 1 and 2
#define ID_ANA_VAL34    ((0x602) | (0x0F0&(CSI_config_ID<<4)))   //send Analog Value 3 and 4
#define ID_DIG_VAL1     ((0x603) | (0x0F0&(CSI_config_ID<<4)))    //send Digital Value
#define ID_ERROR        ((0x606) | (0x0F0&(CSI_config_ID<<4)))    //error message
#define ID_CSI_LOAD     ((0x607) | (0x0F0&(CSI_config_ID<<4)))      //send  the acutal CSI Load

#define ID_REQ_VOLTS    (0x608 | (CSI_ID<<4))   // actual Voltage of a channel to the dash manager
#define ID_CONFIG_CSI   (0x609 | (CSI_ID<<4))      //send the config from the CSI to the dash manager

// CSI CAN IDs RX
#define ID_GET_config   0x700    //Dash Manager sends a new config to the CSI
#define ID_SWITCH_5V_SW  ((0x604) | (CSI_config_ID<<4))

#define LED_TIMEOUT_MS 3000 //LED timeout in ms. LED light up if no msg received within LED timout time

#endif
