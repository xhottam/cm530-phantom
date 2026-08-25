/*
 * button.c
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#include "button.h"
#include "system_init.h"


EasyPort_t EasyButton[6] = { { PORT_SW_UP, PIN_SW_UP }, { PORT_SW_DOWN,
PIN_SW_DOWN }, { PORT_SW_LEFT, PIN_SW_LEFT }, {
PORT_SW_RIGHT, PIN_SW_RIGHT }, { PORT_SW_START,
PIN_SW_START }, { PORT_MIC, PIN_MIC } };
//##############################################################################
u8 ReadButton(Button_t button) {
	if (GPIO_ReadInputDataBit(EasyButton[button].port, EasyButton[button].pin)
			!= SET)
		return 1;
	return 0;
}
