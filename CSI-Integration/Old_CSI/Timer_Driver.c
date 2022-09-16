#include "TIMER_Driver.h"

/*******************************************************
 **                   Global Interface                 **
 *******************************************************/

//TCKPS (bit 5-4) Timer Input Clock Prescale Select Bits
// 11 = 1:256 prescale value

/* max 8892 us */
void Timer1_Init() {
    T1CONbits.TCKPS = 0b11; // 1:256 prescale value

    /* Period Register (PR1): Used to match with the timer then resets to '0' abd continues to count  */
    /* From Timer_Driver.h:
     *	F_TIMER1 = 200
     *	SYSCLK = 80 MHz
     *	FCY =  SYSCLK / 4 = 20 MHz
     *
     * ((FCY/F_TIMER1/256)-1) = (20MHz/200/256) - 1 = //Calculate timer1Period with prescaler 256
     *
     *	Thus: TIMER1PR1 = 389.625
     */
    PR1 = TIMER1PR1; /*(SYSCLOCK / 4 / f);*/

    EnableIntT1; // enabling interrupts?
}

// ON and OFF is defined as 1 and 0 in global.h
//TON: Timer On Control Bit
void Timer1_Start() {
    TMR1 = 0;	//clear timer1 to 0
    T2CONbits.TON = OFF;
    T1CONbits.TON = ON;
}

void Timer1_Stop() {
    T1CONbits.TON = OFF;
}

void Timer2_Init() {
    T2CONbits.TCKPS = 0b11; // 1:256 prescale value
    PR2 = TIMER2PR1; /*(SYSCLOCK / 4 / f);*/
    EnableIntT2;
}

void Timer2_Start() {
    TMR2 = 0;
    T1CONbits.TON = OFF;
    T2CONbits.TON = ON;
}

void Timer2_Stop() {
    T2CONbits.TON = OFF;
}

void Timer3_Init() {
    T3CONbits.TCKPS = 0b10; //prescaler 64
//    PR2 = TIMER3PR1; /*(SYSCLOCK / 4 / f);*/
    PR3 = 0xFFFE;
    EnableIntT3;
}

void Timer3_Start() {
    TMR3 = 0;
    //T3CONbits.TCKPS = 0b10;
    T3CONbits.TON = OFF;
    T3CONbits.TON = ON;
}

void Timer3_Stop() {
    T3CONbits.TON = OFF;
}

void Timer3_Reset() {
//    TMR3 = 0;
    Timer3_Stop();
    Timer3_Start();
}
