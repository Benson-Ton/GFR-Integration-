#include "FIFO.h"
#include "CAN_Driver.h"
//#include "ADC_Driver.h"

void add_FIFO (struct_FIFO* FIFO, uint8 value)
{
    FIFO->FIFO_Array[FIFO->write_counter] = value;

    FIFO->write_counter++;

    if (FIFO->write_counter == FIFO_SIZE)
    {
        FIFO->write_counter = 0;
    }
}

void read_FIFO (struct_FIFO* FIFO)
{
    if (FIFO->read_counter != FIFO->write_counter)
    {
        FIFO->act_Value = FIFO->FIFO_Array[FIFO->read_counter];

        FIFO->read_counter++;
        
        if (FIFO->read_counter == FIFO_SIZE)
        {
            FIFO->read_counter = 0;
        }

    }
    else
    {
        FIFO->act_Value = 0;
    }
}

void add_FIFO_CAN (struct_CAN_FIFO* FIFO, struct_CAN_Message value)
{
    FIFO->FIFO_Array[FIFO->write_counter] = value;

    FIFO->write_counter++;

    if (FIFO->write_counter == CAN_FIFOSIZE)
    {
        FIFO->write_counter = 0;
    }
}

void read_FIFO_CAN (struct_CAN_FIFO* FIFO)
{
    if (FIFO->read_counter != FIFO->write_counter)
    {
        FIFO->act_Value = FIFO->FIFO_Array[FIFO->read_counter];

        FIFO->read_counter++;

        if (FIFO->read_counter == CAN_FIFOSIZE)
        {
            FIFO->read_counter = 0;
        }
    }
    else
    {
        FIFO->act_Value.CAN_ID = 0;
        FIFO->act_Value.CAN_Word1 = 0;
        FIFO->act_Value.CAN_Word2 = 0;
        FIFO->act_Value.CAN_Word3 = 0;
        FIFO->act_Value.CAN_Word4 = 0;
    }
}
