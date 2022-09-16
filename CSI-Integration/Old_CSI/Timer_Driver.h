#ifndef TIMER_H
#define	TIMER_H

#include "global.h"

/*******************************************************
**                   Define Section                  **
*******************************************************/

#define F_TIMER1 200                            //Frequency of Timer1Interrupt in 1/s
#define TIMER1PR1 ((FCY/F_TIMER1/256)-1) //Calculate timer1Period with prescaler 256
#define F_TIMER2 200                            //Frequency of Timer3Interrupt in 1/s
#define TIMER2PR1 ((FCY/F_TIMER2/256)-1) //Calculate timer2Period with prescaler 256
//used for wheelspeed
#define F_TIMER3 5                         //Frequency of Timer3Interrupt in 1/s
#define TIMER3PR1 ((FCY/F_TIMER3/64)-1)   //Calculate timer3Period with prescaler 256

/*******************************************************
**                   Global Interface                 **
*******************************************************/

void Timer1_Init();
void Timer1_Start();
void Timer1_Stop();

void Timer2_Init();
void Timer2_Start();
void Timer2_Stop();

void Timer3_Init();
void Timer3_Start();
void Timer3_Stop();
void Timer3_Reset();
#endif
