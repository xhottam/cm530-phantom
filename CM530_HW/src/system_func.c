/*
 * system_func.c
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#include "stm32f10x_lib.h"
#include "system_func.h"
#include "system_init.h"
#include "led.h"





//##############################################################################
//##############################################################################
// CM-530 Utility functions
//##############################################################################
//##############################################################################


  volatile u32 glDelayCounter;
  volatile u32 glCountdownCounter;
  volatile u32 glDxlTimeoutCounter;
  volatile u32 glPcuTimeoutCounter;
  volatile u32 glBuzzerCounter;
  volatile u8 gbCounterCount;
  volatile u32 Millis;
  volatile u32 msMillis;

  volatile u32 Millis_TIM2;
  volatile u32 msMillis_TIM2;

  vu32                            capture = 0;
  volatile vu32                   gw1msCounter;



#ifdef USING_BREAK_TO_BOOTLOADER
void BreakToBootLoader(void);
#endif

//##############################################################################
void mDelay(u32 nTime) {
	uDelay(nTime * 1000);
	//uDelay(nTime);
}

/*
void mDelay(u32 nTime)
{

	   Enable the SysTick Counter
	  SysTick_CounterCmd(SysTick_Counter_Enable);

	  gwTimingDelay = nTime * 900;

	  while(gwTimingDelay != 0);

	   Disable SysTick Counter
	  SysTick_CounterCmd(SysTick_Counter_Disable);
	   Clear SysTick Counter
	  SysTick_CounterCmd(SysTick_Counter_Clear);

}
*/

//##############################################################################
void uDelay(u32 nTime) {
	if (glDelayCounter == 0)
		gbCounterCount++;

#ifdef USING_SYSTICK_1000US
	if (nTime>=1000)
	glDelayCounter = (nTime/1000);
	else
	glDelayCounter = 1;
#elif USING_SYSTICK_100US
	if (nTime>=100)
	glDelayCounter = (nTime/100);
	else
	glDelayCounter = 1;
#elif defined USING_SYSTICK_10US
	if (nTime >= 10)
		glDelayCounter = (nTime / 10);
	else
		glDelayCounter = 1;
#elif defined USING_SYSTICK_1US
	glDelayCounter = (nTime);
#endif

	if (gbCounterCount == 1) {
		// Enable the SysTick Counter
		SysTick_CounterCmd(SysTick_Counter_Enable);
	}

	while (glDelayCounter != 0)
		;
}

//##############################################################################
void StartCountdown(u32 StartTime) {
	if (glCountdownCounter == 0)
		gbCounterCount++;

	// Want Timer counting in 1 [ms] intervals
#ifdef USING_SYSTICK_1000US
	glCountdownCounter = (StartTime*1);
#elif USING_SYSTICK_100US
	glCountdownCounter = (StartTime*10);
#elif defined USING_SYSTICK_10US
	glCountdownCounter = (StartTime * 100);
#elif defined USING_SYSTICK_1US
	glCountdownCounter = (StartTime*1000);
#endif

	if (gbCounterCount == 1) {
		// Enable the SysTick Counter
		SysTick_CounterCmd(SysTick_Counter_Enable);
	}

	//SetLED(AUX, 1);
}

//##############################################################################
void start_countdown_buzzer(u32 nTime) {
	if (glBuzzerCounter == 0)
		gbCounterCount++;

	// Want Timer counting in 1 [ms] intervals
#ifdef USING_SYSTICK_1000US
	glBuzzerCounter = (nTime*1);
#elif USING_SYSTICK_100US
	glBuzzerCounter = (nTime*10);
#elif defined USING_SYSTICK_10US
	glBuzzerCounter = (nTime * 100);
#elif defined USING_SYSTICK_1US
	glBuzzerCounter = (nTime*1000);
#endif

	if (gbCounterCount == 1) {
		// Enable the SysTick Counter
		SysTick_CounterCmd(SysTick_Counter_Enable);
	}
}

//##############################################################################
void start_timeout_dxl(u32 nTime) {
	if (glDxlTimeoutCounter == 0)
		gbCounterCount++;

	// Due to SysTick() interrupt, default is using 10 [us] intervals
#ifdef USING_SYSTICK_1000US
	if (nTime>=1000)
	glDxlTimeoutCounter = (nTime/1000);
	else
	glDxlTimeoutCounter = 1;
#elif USING_SYSTICK_100US
	if (nTime>=100)
	glDxlTimeoutCounter = (nTime/100);
	else
	glDxlTimeoutCounter = 1;
#elif defined USING_SYSTICK_10US
	if (nTime >= 10)
		glDxlTimeoutCounter = (nTime / 10);
	else
		glDxlTimeoutCounter = 1;
#elif defined USING_SYSTICK_1US
	glDxlTimeoutCounter = (nTime);
#endif

	if (gbCounterCount == 1) {
		// Enable the SysTick Counter
		SysTick_CounterCmd(SysTick_Counter_Enable);
	}
}

void StartDiscount(s32 StartTime)
{
	gw1msCounter = StartTime;
}

u8 CheckTimeOut(void)
{
	// Check timeout
	// Return: 0 is false, 1 is true(timeout occurred)

	if(gw1msCounter == 0)
		return 1;
	else
		return 0;
}

//##############################################################################
void start_timeout_pcu(u32 nTime) {
	if (glPcuTimeoutCounter == 0)
		gbCounterCount++;

	// Due to SysTick() interrupt, default is using 10 [us] intervals
#ifdef USING_SYSTICK_1000US
	if (nTime>=1000)
	glPcuTimeoutCounter = (nTime/1000);
	else
	glPcuTimeoutCounter = 1;
#elif USING_SYSTICK_100US
	if (nTime>=100)
	glPcuTimeoutCounter = (nTime/100);
	else
	glPcuTimeoutCounter = 1;
#elif defined USING_SYSTICK_10US
	if (nTime >= 10)
		glPcuTimeoutCounter = (nTime / 10);
	else
		glPcuTimeoutCounter = 1;
#elif defined USING_SYSTICK_1US
	glPcuTimeoutCounter = (nTime);
#endif

	if (gbCounterCount == 1) {
		// Enable the SysTick Counter
		SysTick_CounterCmd(SysTick_Counter_Enable);
	}
}

//##############################################################################
u32 getMillis(){
	/**msMillis = (Millis * (u32)(0.001)); //convert us to ms
	return msMillis;*/
	return (Millis/100) ;
	//return Millis;

}
u32 getMillis_TIM2(){
	/**msMillis = (Millis * (u32)(0.001)); //convert us to ms
	return msMillis;*/
	return Millis_TIM2;
	//return Millis;

}
//}
//##############################################################################
void ISR_Delay_Base(void) {

	//Millis++;



	// User accessible delay counter
	if (glDelayCounter > 1)
		glDelayCounter--;
	else if (glDelayCounter > 0) {
		glDelayCounter--;
		gbCounterCount--;
	}

	// User accessible timeout/countdown counter
	if (glCountdownCounter > 1) {
		glCountdownCounter--;
#ifdef USING_SYSTICK_1000US
		if ( (glCountdownCounter&0x00000300) )
#elif USING_SYSTICK_100US
		if ( (glCountdownCounter&0x00000200) )
#elif defined USING_SYSTICK_10US
		//if ((glCountdownCounter & 0x00001000))
#elif defined USING_SYSTICK_1US
		if ( (glCountdownCounter&0x00010000) )
#endif
			//SetLED(AUX, 1);
		//else
			//SetLED(AUX, 0);
	} else if (glCountdownCounter > 0) {
		//SetLED(AUX, 0);
		glCountdownCounter--;
		gbCounterCount--;
	}
	// Buzzer countdown counter
	if (glBuzzerCounter > 1)
		glBuzzerCounter--;
	else if (glBuzzerCounter > 0) {
		glBuzzerCounter--;
		gbCounterCount--;
	}

	// Dynamixel timeout counter
	if (glDxlTimeoutCounter > 1)
		glDxlTimeoutCounter--;
	else if (glDxlTimeoutCounter > 0) {
		glDxlTimeoutCounter--;
		gbCounterCount--;
	}

	// PC UART timeout counter
	if (glPcuTimeoutCounter > 1)
		glPcuTimeoutCounter--;
	else if (glPcuTimeoutCounter > 0) {
		glPcuTimeoutCounter--;
		gbCounterCount--;
	}

	// Battery Monitor timeout counter
//    if (glBatTimeoutCounter>1)
//        glBatTimeoutCounter--;
//    else
//    {
//        Battery_Monitor_Alarm();
//        if (glBatTimeoutSet>100000)
//            glBatTimeoutCounter = glBatTimeoutSet;
//        else
//            glBatTimeoutCounter = 100000;
//    }

	// If no active counters, disable interrupt
	if (gbCounterCount == 0) {
		// Disable SysTick Counter
		SysTick_CounterCmd(SysTick_Counter_Disable);
		// Clear SysTick Counter
		SysTick_CounterCmd(SysTick_Counter_Clear);
	}
}

void TimerInterrupt_1ms(void) //OLLO CONTROL
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) // 1ms//
	{
		Millis_TIM2++;

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		capture = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + (vu16)100);

		if(gw1msCounter > 0)
			gw1msCounter--;
	}
}

