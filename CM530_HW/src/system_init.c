/*
 * system_init.c
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#include "stm32f10x_lib.h"
#include "system_init.h"
#include "usart.h"
#include "adc.h"
#include "system_func.h"
#include "serial.h"
#include "dynamixel.h"
#include "zigbee.h"



//##############################################################################
void SysInit(){
	// Clear the WatchDog Early Wakeup interrupt flag
//    WWDG_ClearFlag();
	ReBootToBootLoader = 0;

	// System Clocks Configuration
	RCC_Configuration();

	// NVIC configuration
	NVIC_Configuration();

	// GPIO configuration
	GPIO_Configuration();

	// System clock count configuration
	SysTick_Configuration();

	Timer_Configuration();

	// Analog to Digital Converter Configuration
	ADC_Configuration();

	// USART Configuration
	USART_Configuration();
}

//##############################################################################
void RCC_Configuration(void) {
	ErrorStatus HSEStartUpStatus;
	// RCC system reset(for debug purpose)
	RCC_DeInit();

	// Enable HSE
	RCC_HSEConfig(RCC_HSE_ON);

	// Wait till HSE is ready
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS) {
		// Enable Prefetch Buffer
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		// Flash 2 wait state
		FLASH_SetLatency(FLASH_Latency_2);

		// HCLK = SYSCLK
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		// PCLK2 = HCLK
		RCC_PCLK2Config(RCC_HCLK_Div1);

		// PCLK1 = HCLK/2
		RCC_PCLK1Config(RCC_HCLK_Div2);

		// PLLCLK = 8MHz * 9 = 72 MHz
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		// Enable PLL
		RCC_PLLCmd(ENABLE);

		// Wait till PLL is ready
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
		}

		// Select PLL as system clock source
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		// Wait till PLL is used as system clock source
		while (RCC_GetSYSCLKSource() != 0x08) {
		}
	}

	// Enable peripheral clocks

	// Enable GPIOB and GPIOC clocks
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);

	// Enable USART1 Clock (Dynamixel)
#ifdef USING_DYNAMIXEL
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
#ifdef USING_PC_UART
	// Enable USART3 Clock (PC_UART)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
#ifdef USING_ZIGBEE
	// Enable UART5 Clock (Zigbee)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
#endif

	/* Enable Timer2  clocks */
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2 , ENABLE);

	PWR_BackupAccessCmd(ENABLE);
}

//##############################################################################
void NVIC_Configuration(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
	// Set the Vector Table base location at 0x20000000
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  // VECT_TAB_FLASH
	// Set the Vector Table base location at 0x08003000
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
#endif

	// Configure the NVIC Preemption Priority Bits
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

//############################################################

	NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick,0,0);

//############################################################

	// Enable the USART1 Interrupt (Dynamixel)
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#ifdef USING_PC_UART
	// Enable the USART3 Interrupt (Serial/PC_UART)
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	// Enable the UART5 Interrupt (Zigbee)
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;


	// Enable the TIM2 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

//##############################################################################
void GPIO_Configuration(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_StructInit(&GPIO_InitStructure);

		// PORTA CONFIG
		GPIO_InitStructure.GPIO_Pin = PIN_SIG_MOT1P | PIN_SIG_MOT1M | PIN_SIG_MOT2P
				| PIN_SIG_MOT2M | PIN_SIG_MOT5P | PIN_SIG_MOT5M;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_BUZZER | PIN_ZIGBEE_RESET;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_ADC1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_SW_RIGHT | PIN_SW_LEFT;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		// PORTB CONFIG
		GPIO_InitStructure.GPIO_Pin = 	 PIN_SIG_MOT6P | PIN_SIG_MOT6M;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_LED_AUX | PIN_LED_MANAGE | PIN_LED_PROGRAM
				| PIN_LED_PLAY;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_ENABLE_TXD | PIN_ENABLE_RXD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_SW_START;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		// PORTC CONFIG
		GPIO_InitStructure.GPIO_Pin = PIN_SIG_MOT3P | PIN_SIG_MOT3M | PIN_SIG_MOT4P
				| PIN_SIG_MOT4M;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_ADC_SELECT0 | PIN_ADC_SELECT1;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_LED_POWER | PIN_LED_TXD | PIN_LED_RXD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_TXD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_ADC0 | PIN_VDD_VOLT;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_MIC;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = PIN_SW_UP | PIN_SW_DOWN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		// PORTD CONFIG
		GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_RXD;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		GPIO_PinRemapConfig( GPIO_Remap_USART1, ENABLE);
		GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE);
}
//##############################################################################
void ADC_Configuration(void) {
	/*ADC_InitTypeDef ADC_InitStructure;

		ADC_StructInit(&ADC_InitStructure);

		// ADC1 configuration
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 2;

		ADC_Init(ADC1, &ADC_InitStructure);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;

		ADC_Init(ADC2, &ADC_InitStructure);

		// ADC1 regular channels configuration
		// Set ADC1 to read SIG_ADC0 (ADC1 multiplexer output) on Channel 10
		ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1,
		ADC_SampleTime_239Cycles5);
		// Set ADC1 to read VBUS on Channel 13
	//    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1 , ADC_SampleTime_239Cycles5);    // SIG_VDD/VBUS
		//ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

		// ADC2 regular channels configuration
		// Set ADC2 to read SIG_ADC1 (ADC2 multiplexer output) on Channel 5
		ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5); // SIG_ADC1
		//ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

		// Enable ADC1 DMA
		//ADC_DMACmd(ADC1, ENABLE);

		// Enable ADC1,2
		ADC_Cmd(ADC1, ENABLE);
		ADC_Cmd(ADC2, ENABLE);

		// Enable ADC1,2 reset calibration register
		// Check the end of ADC1,2 reset calibration register
		ADC_ResetCalibration(ADC1);
		while (ADC_GetResetCalibrationStatus(ADC1))
			;

		ADC_ResetCalibration(ADC2);
		while (ADC_GetResetCalibrationStatus(ADC2))
			;

		// Start ADC1,2 calibration
		// Check the end of ADC1,2 calibration
		ADC_StartCalibration(ADC1);
		while (ADC_GetCalibrationStatus(ADC1))
			;

		ADC_StartCalibration(ADC2);
		while (ADC_GetCalibrationStatus(ADC2))
			;

		// Start ADC2 Software Conversion
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		ADC_SoftwareStartConvCmd(ADC2, ENABLE);*/
	ADC_InitTypeDef ADC_InitStructure;

	ADC_StructInit(&ADC_InitStructure);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Init(ADC2, &ADC_InitStructure);

	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1 , ADC_SampleTime_239Cycles5);
	//ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	/* ADC2 regular channels configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5);
	//ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

	/* Enable ADC1 DMA */
	//ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1,2 */
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	/* Enable ADC1,2 reset calibration register */
	/* Check the end of ADC1,2 reset calibration register */
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2));

	/* Start ADC1,2 calibration */
	/* Check the end of ADC1,2 calibration */
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC2);
	while(ADC_GetCalibrationStatus(ADC2));


	/* Start ADC2 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}
//##############################################################################
void SysTick_Configuration(void) {
#ifdef USING_SYSTICK_1000US
	// SysTick end of count event each 100us with input clock equal to 9MHz (HCLK/8, default)
	SysTick_SetReload(9000);
#elif USING_SYSTICK_100US
	// SysTick end of count event each 100us with input clock equal to 9MHz (HCLK/8, default)
	SysTick_SetReload(900);
#elif defined USING_SYSTICK_10US
	// SysTick end of count event each 10us with input clock equal to 9MHz (HCLK/8, default)
	SysTick_SetReload(90);
#elif defined USING_SYSTICK_1US
	// SysTick end of count event each 1us with input clock equal to 9MHz (HCLK/8, default)
	SysTick_SetReload(9);
#endif
	// Enable SysTick interrupt
	SysTick_ITConfig(ENABLE);

	// Enable the SysTick Counter
    SysTick_CounterCmd(SysTick_Counter_Enable);

	// Reset Active Counter count
	gbCounterCount = 0;
}


void Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_DeInit(TIM2);

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// Prescaler configuration
	TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

	// Output Compare Timing Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = (vu16)100 ;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	// TIM IT enable
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	// TIM2 enable counter
	TIM_Cmd(TIM2, ENABLE);
}
//##############################################################################
void USART_Configuration(){
	u16 error = 0;

#ifdef USING_PC_UART
	mDelay(100);
	if (!pcu_initialize(Baudrate_PCU))
		error |= (1 << 0);
#endif
#ifdef USING_ZIGBEE
	mDelay(100);
	if (!zgb_initialize(Baudrate_ZIG))
	error|=(1<<1);
#endif
#ifdef USING_DYNAMIXEL
	mDelay(100);
	if (!dxl_initialize(Baudrate_DXL))
		error |= (1 << 2);
#endif
}
