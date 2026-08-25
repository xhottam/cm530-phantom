/*
 * system_func.h
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_HW_INC_SYSTEM_FUNC_H_
#define CM530_HW_INC_SYSTEM_FUNC_H_

#include "stm32f10x_type.h"
#include "stm32f10x_systick.h"

// Select which time interval to call SysTick() interrupt
//   Using "USING_SYSTICK_1US" calls interrupt every 72 cycles, so increased
//   accuracy/resolution for Dxl/Pcu timeouts might not be worth the lost cycles
//   due to handling the interrupt.  i.e. 10 [us] interval works fine and will
//   round to nearest 10 [us].
//#define USING_SYSTICK_1000US
//#define USING_SYSTICK_100US
#define USING_SYSTICK_10US
//#define USING_SYSTICK_1US

 extern volatile u32 glDelayCounter;
 extern volatile u32 glCountdownCounter;
 extern volatile u32 glDxlTimeoutCounter;
 extern volatile u32 glPcuTimeoutCounter;
 extern volatile u32 glBuzzerCounter;
//volatile u32 Millis;
//volatile u32 glBatTimeoutCounter;
//volatile u32 glBatTimeoutSet;
 extern volatile u8 gbCounterCount;

 extern volatile u32 Millis;
 extern volatile u32 msMillis;

 extern volatile u32 Millis_TIM2;
 extern volatile u32 msMillis_TIM2;

 void start_timeout_dxl(u32);
 void start_timeout_pcu(u32);
 void start_countdown_buzzer(u32 nTime);

 void StartDiscount(s32 StartTime);
 u8 CheckTimeOut(void);


/**
 * Function to initialize an independent countdown timer
 * @param nTime Time in milliseconds for the timer to countdown.
 */
void StartCountdown(u32);

u32 getMillis(void);

u32 getMillis_TIM2(void);

/**
 * Microsecond Delay function (default resolution is 10 [us])
 * @param nTime Time in microseconds to delay (unsigned 32-bit integer).
 * @see mDelay()
 */
void uDelay(u32);    // Actually uses a 10 [us] delay interval
/**
 * Millisecond Delay function
 * @param nTime Time in milliseconds to delay (unsigned 32-bit integer).
 * @see uDelay()
 */
void mDelay(u32);    // Correctly uses 1 [ms] delay interval


void ISR_Delay_Base(void);

void TimerInterrupt_1ms(void);

#endif /* CM530_HW_INC_SYSTEM_FUNC_H_ */
