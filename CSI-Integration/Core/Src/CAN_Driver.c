/*******************************************************
 **                   Include Section                  **
 *******************************************************/
#include "CAN_Driver.h"
#include "FIFO.h"
#include "main.h"
//#include "stm32f7xx_hal_can.h"
//extern uint8_t TxData[8];
//extern CAN_TxHeaderTypeDef TxHeader;
//extern	uint32_t TxMailbox;
//extern CAN_HandleTypeDef hcan1;
struct_CAN_FIFO CAN_Receive_FIFO;

/*******************************************************
 **                 Functions                         **
 *******************************************************/

/*Initializes the CAN-Channels and enables Interrupts*/
void CAN_Filter_Config_Init(void)
{

//	  CAN_FilterTypeDef canfilterconfig;

	  //configuring CAN Filter
//	    canfilterconfig.FilterActivation = ENABLE;
	//    canfilterconfig.FilterBank = 0;  // anything between 0 to SlaveStartFilterBank
	    									/*!< Specifies the filter bank which will be initialized.
	                                           For single CAN instance(14 dedicated filter banks),
	                                           this parameter must be a number between Min_Data = 0 and Max_Data = 13.
	                                           For dual CAN instances(28 filter banks shared),
	                                           this parameter must be a number between Min_Data = 0 and Max_Data = 27. */
	 //   canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	 //   canfilterconfig.FilterIdHigh = 0x255; // highest ID allowed through the filter
	 //   canfilterconfig.FilterIdLow = 0x0000; // lowest ID allowed through the filter
	//    canfilterconfig.FilterMaskIdHigh = 0x00; // mask bits that apply to address
	//    canfilterconfig.FilterMaskIdLow = 0x00;
	//    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	 //   canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	//    canfilterconfig.SlaveStartFilterBank = 10;
														/*  Select the start filter bank for the slave CAN instance.  For single CAN instances, this parameter is meaningless.
	    												For dual CAN instances, all filter banks with lower index are assigned to master
	    												CAN instance, whereas all filter banks with greater index are assigned to slave CAN instance.
	    												This parameter must be a number between Min_Data = 0 and Max_Data = 27.
	     	 	 	 	 	 	 	 	 	 	 	 */

	  //  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);





	/*
    //Set CAN1 in cofigurationmode
    C1CTRL = CAN_REQ_OPERMODE_CONFIG;

    //Wait till it is in configurationmode
    while ((C1CTRL | 0xFF0F) != 0xFF8F);

    //set Baudrate to 500kb/s
    //C1CFG1 = 0x0000;
    //C1CFG2 = 0x07BA;  //SEG2PH = 8, SAM = 0, SEG1PH = 8, PRSEG = 2

    //set Baudrate to 1Mbit
  //  C1CFG1 = 0x0000;
  //  C1CFG2bits.PRSEG = 1;   //2 Tq
  //  C1CFG2bits.SEG1PH = 3;  //4 Tq
  //  C1CFG2bits.SAM = 0;     //sample once at sample point
  //  C1CFG2bits.SEG2PHTS = 1;//SEG2PH freely programmable
   // C1CFG2bits.SEG2PH = 2;  //3 Tq
  //  C1CFG2bits.WAKFIL = 0;  //no wake up on CAN

    //EvalBoard
//    C1CFG1 = 0x3;
//    C1CFG2 = 0x04B0;

    //set Filter for reception interrupts
    setFilter();

    //Enable all Interrupts
    C1INTE = 0x0003;

    //Enable Interrupts CAN1
    EnableIntCAN1;

    //Set CAN1 Receive opten to receive
    C1RX1CONbits.RXFUL = 0;

    //Set CAN1 in normalmode
    C1CTRL = CAN_REQ_OPERMODE_NOR;
    
     //Disable CAN1 capturemode, because otherwise RD9 is not usable (Family References p.685)
    C1CTRLbits.CANCAP = 0;

    //Wait till it is in normalmode
    while ((C1CTRL | 0xFF0F) != 0xFF0F);

     C1INTFbits.RX0IF = 0;
     */
}


/*Sends the Message with the given ID and Bytes*/
void send_CAN(struct_CAN_Message Msg)
{
    //int timeoutcounter = 0;

    /*Waiting for CAN-Buffer to be ready again*/
  //  while (C1TX0CONbits.TXREQ == 1 && (timeoutcounter < CAN_TIMEOUT * 4))
   // {
  //      timeoutcounter++;
   // } // <<< needs to be changed for STM32 parameters

    /*Disable Interrupts*/
   // CLI(); // use HAL_CAN_DeactivateNotification instead

    /*Reset ID so no old ID remains*/
  //  C1TX0SID = 0x000; // <<< needs to be changed for STM32 parameters

    /*Reset Dataregisters so no old Data remains*/ // <<< needs to be changed for STM32 parameters
  //  C1TX0B1 = 0x0000;
    //C1TX0B2 = 0x0000;
    //C1TX0B3 = 0x0000;
    //C1TX0B4 = 0x0000;

    /*Writing the data register*/
    //can be used directly instead of assigning them to a register

   // C1TX0B1 = Msg.CAN_Word1;
  //  C1TX0B2 = Msg.CAN_Word2;
  //  C1TX0B3 = Msg.CAN_Word3;
  //  C1TX0B4 = Msg.CAN_Word4;


    //!!!! new changes for data-register STM32 format




    /*Setting DataLength to 8 Bytes*/
   // C1TX0DLCbits.DLC = 8; // dsPIC


    //STM32
    TxHeader.DLC = 8; // byte in length, 8 bytes stated in line 94(CAN_Driver.c)

    /*Writing the ID-register*/ // these are transmit buffers
    //C1TX0SIDbits.SID5_0 = Msg.CAN_ID & 0x03F; 			// standard identifier bits <6:0>
    //C1TX0SIDbits.SID10_6 = (Msg.CAN_ID & 0x7C0) >> 6;	// standard identifier bits<10:6>
    //C1TX0SIDbits.TXIDE = 0; 							// extended identifier bit | 0 = standard identifier
   // C1TX0SIDbits.SRR = 0;								// substitute remote request control bit | 0 = normal message

    //!!!! new changes for ID-register STM32 format

    TxHeader.ExtId = 0; // Specifies the extended identifier. This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF.
    TxHeader.IDE = CAN_ID_STD; // set the identifier to standard | 0x00
    TxHeader.RTR = CAN_RTR_DATA; // set to allow for remote transmission requests | it is set for data frame
    TxHeader.StdId = Msg.CAN_ID; //set the standard identifier to 256 (Field Nodes cannot access anything higher than 255 for their addresses)

    TxData[0] = Msg.CAN_Word1;
    TxData[1] = Msg.CAN_Word2;
    TxData[2] = Msg.CAN_Word3;
    TxData[3] = Msg.CAN_Word4;
    TxData[4] = Msg.CAN_Word5;
    TxData[5] = Msg.CAN_Word6;
    TxData[6] = Msg.CAN_Word7;
    TxData[7] = Msg.CAN_Word8;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);


    /*Request Message Transmission from CAN-Module*/
    //C1TX0CONbits.TXREQ = 1;

    /*Enable Interrupts again*/
    //SEI(); // use HAL_CAN_Activate Notification instead
}


/*Defines the Masks and Filters*/
/*
void setFilter()
{
     //CSI listens to ids 0x700
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
*/

/*adds the data of incoming messages to the buffer array (uint can1_fifo_msg_array)*/
void can_fifo_add_msg(struct_CAN_Message msg){
    add_FIFO_CAN(&CAN_Receive_FIFO, msg);
}

/*returns the latest data and message ID of incoming messages of the buffer array (uint can1_fifo_msg_array)*/
struct_CAN_Message can_fifo_read_msg(){
    read_FIFO_CAN(&CAN_Receive_FIFO);
    return CAN_Receive_FIFO.act_Value;
}
