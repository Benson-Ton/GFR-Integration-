#ifndef CAN_DRIVER_H
#define	CAN_DRIVER_H

#include "global.h"
#include "main.h"

#define CAN_TIMEOUT 2000 //in us
#define START_CAN_ID 0x600
#define CAN_FIFOSIZE 10

extern uint8_t TxData[8];
extern CAN_TxHeaderTypeDef TxHeader;
extern	uint32_t TxMailbox;
extern CAN_HandleTypeDef hcan1;

/*******************************************************
**                 System Functions                   **
*******************************************************/
//Setting up for CAN message by holding CAN IDs, and Data
typedef struct
{
    uint16_t CAN_ID;
    uint8_t CAN_Byte1;
    uint8_t CAN_Byte2;
    uint8_t CAN_Byte3;
    uint8_t CAN_Byte4;
    uint8_t CAN_Byte5;
    uint8_t CAN_Byte6;
    uint8_t CAN_Byte7;
    uint8_t CAN_Byte8;
}struct_CAN_Message;

//CAN_FilterTypeDef canfilterconfig;



void send_CAN(struct_CAN_Message Msg); //used to prep to send CAN data and IDd
void can_fifo_add_msg(struct_CAN_Message msg);
struct_CAN_Message can_fifo_read_msg();

#endif

