/*******************************************************
 **                   Include Section                  **
 *******************************************************/
#include "CAN_Driver.h"
#include "FIFO.h"

struct_CAN_FIFO CAN_Receive_FIFO;

/*******************************************************
 **                 Functions                         **
 *******************************************************/

/*Initializes the CAN-Channels and enables Interrupts*/
void CAN_Init(void)
{
    /*Set CAN1 in cofigurationmode*/
    C1CTRL = CAN_REQ_OPERMODE_CONFIG;

    /*Wait till it is in configurationmode*/
    while ((C1CTRL | 0xFF0F) != 0xFF8F);

    /*set Baudrate to 500kb/s*/
    //C1CFG1 = 0x0000;
    //C1CFG2 = 0x07BA;  //SEG2PH = 8, SAM = 0, SEG1PH = 8, PRSEG = 2

    /*set Baudrate to 1Mbit */
    C1CFG1 = 0x0000;
    C1CFG2bits.PRSEG = 1;   //2 Tq
    C1CFG2bits.SEG1PH = 3;  //4 Tq
    C1CFG2bits.SAM = 1;     //sample three times
    C1CFG2bits.SEG2PHTS = 1;//SEG2PH freely programmable
    C1CFG2bits.SEG2PH = 2;  //3 Tq
    C1CFG2bits.WAKFIL = 0;  //no wake up on CAN
    C1CFG1bits.SJW = 2;     //3 Tq

    /*EvalBoard*/
//    C1CFG1 = 0x3;
//    C1CFG2 = 0x04B0;

    /*set Filter for reception interrupts*/
    setFilter();

    /*Enable all Interrupts*/
    C1INTE = 0x0003;

    /*Enable Interrupts CAN1*/
    EnableIntCAN1;

    /*Set CAN1 Receive opten to receive*/
    C1RX1CONbits.RXFUL = 0;

    /*Set CAN1 in normalmode*/
    C1CTRL = CAN_REQ_OPERMODE_NOR;

     /*Disable CAN1 capturemode, because otherwise RD9 is not usable (Family References p.685)*/
    C1CTRLbits.CANCAP = 0;

    /*Wait till it is in normalmode*/
    while ((C1CTRL | 0xFF0F) != 0xFF0F);

     C1INTFbits.RX0IF = 0;
}


/*Sends the Message with the given ID and Bytes*/
void send_CAN(struct_CAN_Message Msg)
{
    int timeoutcounter = 0;

    /*Waiting for CAN-Buffer to be ready again*/
    while (C1TX0CONbits.TXREQ == 1 && (timeoutcounter < CAN_TIMEOUT * 4))
    {
        timeoutcounter++;
    }

    /*Disable Interrupts*/
    CLI();

    /*Reset ID so no old ID remains*/
    C1TX0SID = 0x000;

    /*Reset Dataregisters so no old Data remains*/
    C1TX0B1 = 0x0000;
    C1TX0B2 = 0x0000;
    C1TX0B3 = 0x0000;
    C1TX0B4 = 0x0000;

    /*Writing the data register*/
    C1TX0B1 = Msg.CAN_Word1;
    C1TX0B2 = Msg.CAN_Word2;
    C1TX0B3 = Msg.CAN_Word3;
    C1TX0B4 = Msg.CAN_Word4;

    /*Setting DataLength to 8 Bytes*/
    C1TX0DLCbits.DLC = 8;

    /*Writing the ID-register*/
    C1TX0SIDbits.SID5_0 = Msg.CAN_ID & 0x03F; // writing the 6 bits 5:0
    C1TX0SIDbits.SID10_6 = (Msg.CAN_ID & 0x7C0) >> 6; // writing the 5 upper bits 10:6
    C1TX0SIDbits.TXIDE = 0; //extended identifier set to standard
    C1TX0SIDbits.SRR = 0; // Substitute Remote Request Control bit set to normal message

    /*Request Message Transmission from CAN-Module*/
    C1TX0CONbits.TXREQ = 1;

    /*Enable Interrupts again*/
    SEI();
}


/*Defines the Masks and Filters*/
void setFilter()
{
     /*CSI listens to ids 0x700 */
    C1RXM0SIDbits.MIDE = 0x0;           //MIDE bit
    C1RXM0SIDbits.SID = 0x7FF;          //mask
    C1RXF0SIDbits.SID = 0x700; //ID     //filter
    C1RXF1SIDbits.SID = 0x700; //ID       //filter

    C1RXM1SIDbits.MIDE = 0;             //MIDE bit
    C1RXM1SIDbits.SID = 0x70F;          //mask 5
    C1RXF2SIDbits.SID = 0x604;          //filter
    C1RXF3SIDbits.SID = 0x604;            //filter
    C1RXF4SIDbits.SID = 0x604;            //filter
    C1RXF5SIDbits.SID = 0x604;            //filter
}

/*adds the data of incoming messages to the buffer array (uint can1_fifo_msg_array)*/
void can_fifo_add_msg(struct_CAN_Message msg){
    add_FIFO_CAN(&CAN_Receive_FIFO, msg);
}

/*returns the latest data and message ID of incoming messages of the buffer array (uint can1_fifo_msg_array)*/
struct_CAN_Message can_fifo_read_msg(){
    read_FIFO_CAN(&CAN_Receive_FIFO);
    return CAN_Receive_FIFO.act_Value;
}
