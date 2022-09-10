#ifndef FIFO_H
#define	FIFO_H

#include "global.h"
#include "CAN_Driver.h"
//#include "ADC_Driver.h"

#define FIFO_SIZE 20

typedef struct
{
    uint8_t read_counter;
    uint8_t write_counter;
    uint8_t FIFO_Array[FIFO_SIZE];
    uint8_t act_Value;

}struct_FIFO;

typedef struct
{
	uint8_t read_counter;
	uint8_t write_counter;
    struct_CAN_Message FIFO_Array[CAN_FIFOSIZE];
    struct_CAN_Message act_Value;
}struct_CAN_FIFO;

void add_FIFO (struct_FIFO* FIFO, uint8 value);
void read_FIFO (struct_FIFO* FIFO);

void add_FIFO_CAN (struct_CAN_FIFO* FIFO, struct_CAN_Message value);
void read_FIFO_CAN (struct_CAN_FIFO* FIFO);

#endif	/* FIFO_H */
