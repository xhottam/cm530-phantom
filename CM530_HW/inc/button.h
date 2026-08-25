/*
 * button.h
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_HW_INC_BUTTON_H_
#define CM530_HW_INC_BUTTON_H_

#include "stm32f10x_type.h"

typedef enum Button_e {
    UP                                = 0,
    DOWN                              = 1,
    LEFT                              = 2,
    RIGHT                             = 3,
    START                             = 4,
    MIC                               = 5
} Button_t;



/**
 * Easy Function to read a button on the CM-530.
 * Possible buttons on the CM-530 are:
 *   START, UP, DOWN, LEFT, RIGHT, MIC
 * @param button The button to be read.
 * @return The state of the button.
 */
u8 ReadButton(Button_t button);




#endif /* CM530_HW_INC_BUTTON_H_ */
