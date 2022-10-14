/*******************************************************
 **                   Include Section                  **
 *******************************************************/
#include "CAN_Driver.h"
#include "main.h"
#include "stm32f7xx_hal_can.h"



/*******************************************************
 **                 Functions                         **
 *******************************************************/

// NOTE:

// byte 1 = index
// byte 3, 4 = x value (uint16_t)
// byte 5, 6 = y value (uint16_t)



/*Sends the Message with the given ID and Bytes*/
void send_CAN(struct_CAN_Message Msg)
{
/* Filters and interrupts are activated in the main.c file under MX_CAN1_Init */

/* Setting up the CAN Frame */



    //!!!! new changes for ID-register STM32 format
	TxHeader.DLC = 8; // byte in length, 8 bytes stated in line 94(CAN_Driver.c)
    TxHeader.ExtId = 0; // Specifies the extended identifier. This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF.
    TxHeader.IDE = CAN_ID_STD; // set the identifier to standard | 0x00
    TxHeader.RTR = CAN_RTR_DATA; // set to allow for remote transmission requests | it is set for data frame

    TxHeader.StdId = Msg.CAN_ID; //set the standard identifier to 256 (Field Nodes cannot access anything higher than 255 for their addresses)

    TxData[0] = Msg.CAN_Byte1;
    TxData[1] = Msg.CAN_Byte2;
    TxData[2] = Msg.CAN_Byte3;
    TxData[3] = Msg.CAN_Byte4;
    TxData[4] = Msg.CAN_Byte5;
    TxData[5] = Msg.CAN_Byte6;
    TxData[6] = Msg.CAN_Byte7;
    TxData[7] = Msg.CAN_Byte8;

    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK ){
    	Error_Handler();
    };
}
//while( HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox));




