/*
 * system_init.h
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_HW_INC_SYSTEM_INIT_H_
#define CM530_HW_INC_SYSTEM_INIT_H_

#include "stm32f10x_map.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_nvic.h"
#include "stm32f10x_pwr.h"

// ADC select PORT and PIN definitions
#define PORT_ADC_SELECT0                GPIOC
#define PORT_ADC_SELECT1                GPIOC
#define PIN_ADC_SELECT0                 GPIO_Pin_1
#define PIN_ADC_SELECT1                 GPIO_Pin_2
#define PIN_ADC0                        GPIO_Pin_0
#define PIN_ADC1                        GPIO_Pin_5
#define PIN_VDD_VOLT                    GPIO_Pin_3

// Button PORT and PIN definitions
#define PORT_SW_UP                      GPIOC
#define PORT_SW_DOWN                    GPIOC
#define PORT_SW_RIGHT                   GPIOA
#define PORT_SW_LEFT                    GPIOA
#define PORT_SW_START                   GPIOB
#define PORT_MIC                        GPIOC
#define PIN_SW_UP                       GPIO_Pin_11
#define PIN_SW_DOWN                     GPIO_Pin_10
#define PIN_SW_RIGHT                    GPIO_Pin_14
#define PIN_SW_LEFT                     GPIO_Pin_15
#define PIN_SW_START                    GPIO_Pin_3

// LED PORT and PIN definitions
#define PORT_LED_POWER                  GPIOC
#define PORT_LED_MANAGE                 GPIOB
#define PORT_LED_PROGRAM                GPIOB
#define PORT_LED_PLAY                   GPIOB
#define PORT_LED_TXD                    GPIOC
#define PORT_LED_RXD                    GPIOC
#define PORT_LED_AUX                    GPIOB
#define PIN_LED_POWER                   GPIO_Pin_13
#define PIN_LED_MANAGE                  GPIO_Pin_13
#define PIN_LED_PROGRAM                 GPIO_Pin_14
#define PIN_LED_PLAY                    GPIO_Pin_15
#define PIN_LED_TXD                     GPIO_Pin_14
#define PIN_LED_RXD                     GPIO_Pin_15
#define PIN_LED_AUX                     GPIO_Pin_12


// Buzzer PORT and PIN definitions
#define PORT_BUZZER                     GPIOA
#define PIN_BUZZER                      GPIO_Pin_6
#define PIN_MIC                         GPIO_Pin_4

// Ollo port PORT and PIN definitions
#define PORT_SIG_MOT1P                  GPIOA
#define PORT_SIG_MOT1M                  GPIOA
#define PORT_SIG_MOT2P                  GPIOA
#define PORT_SIG_MOT2M                  GPIOA
#define PORT_SIG_MOT3P                  GPIOC
#define PORT_SIG_MOT3M                  GPIOC
#define PORT_SIG_MOT4P                  GPIOC
#define PORT_SIG_MOT4M                  GPIOC
#define PORT_SIG_MOT5P                  GPIOA
#define PORT_SIG_MOT5M                  GPIOA
#define PORT_SIG_MOT6P                  GPIOB
#define PORT_SIG_MOT6M                  GPIOB
#define PIN_SIG_MOT1P                   GPIO_Pin_0
#define PIN_SIG_MOT1M                   GPIO_Pin_1
#define PIN_SIG_MOT2P                   GPIO_Pin_2
#define PIN_SIG_MOT2M                   GPIO_Pin_3
#define PIN_SIG_MOT3P                   GPIO_Pin_6
#define PIN_SIG_MOT3M                   GPIO_Pin_7
#define PIN_SIG_MOT4P                   GPIO_Pin_8
#define PIN_SIG_MOT4M                   GPIO_Pin_9
#define PIN_SIG_MOT5P                   GPIO_Pin_8
#define PIN_SIG_MOT5M                   GPIO_Pin_11
#define PIN_SIG_MOT6P                   GPIO_Pin_8
#define PIN_SIG_MOT6M                   GPIO_Pin_9

// Dynamixel
#define PORT_ENABLE_TXD                 GPIOB
#define PORT_ENABLE_RXD                 GPIOB
#define PORT_DXL_TXD                    GPIOB
#define PORT_DXL_RXD                    GPIOB
#define PIN_ENABLE_TXD                  GPIO_Pin_4
#define PIN_ENABLE_RXD                  GPIO_Pin_5
#define PIN_DXL_TXD                     GPIO_Pin_6
#define PIN_DXL_RXD                     GPIO_Pin_7

// Zigbee
#define PORT_ZIGBEE_TXD                 GPIOC
#define PORT_ZIGBEE_RXD                 GPIOD
#define PORT_ZIGBEE_RESET               GPIOA
#define PIN_ZIGBEE_TXD                  GPIO_Pin_12
#define PIN_ZIGBEE_RXD                  GPIO_Pin_2
#define PIN_ZIGBEE_RESET                GPIO_Pin_12

// Serial/PC_UART
#define PORT_PC_TXD                     GPIOB
#define PORT_PC_TXD                     GPIOB
#define PIN_PC_TXD                      GPIO_Pin_10
#define PIN_PC_RXD                      GPIO_Pin_11



typedef struct EasyPort_s {
    GPIO_TypeDef* port;
    u16 pin;
} EasyPort_t;



	// System Clocks Configuration
void RCC_Configuration();

	// NVIC configuration
void NVIC_Configuration();

	// GPIO configuration
void GPIO_Configuration();

	// System clock count configuration
void SysTick_Configuration();

// System clock count configuration
void Timer_Configuration();
	// Analog to Digital Converter Configuration
void ADC_Configuration();

	// USART Configuration
void USART_Configuration();

void SysInit();

#endif /* CM530_HW_INC_SYSTEM_INIT_H_ */
