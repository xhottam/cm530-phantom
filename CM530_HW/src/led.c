/*
 * led.c
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#include "led.h"
#include "system_init.h"


EasyPort_t EasyLED[7] = { { PORT_LED_POWER,
PIN_LED_POWER }, { PORT_LED_MANAGE,
PIN_LED_MANAGE }, { PORT_LED_PROGRAM, PIN_LED_PROGRAM }, { PORT_LED_PLAY,
PIN_LED_PLAY }, { PORT_LED_TXD, PIN_LED_TXD }, { PORT_LED_RXD,
PIN_LED_RXD }, { PORT_LED_AUX, PIN_LED_AUX } };

//##############################################################################
void SetLED(LED_t led, u8 state) {
	if (state)
		GPIO_ResetBits(EasyLED[led].port, EasyLED[led].pin);
	else
		GPIO_SetBits(EasyLED[led].port, EasyLED[led].pin);
}
