/*
 * led.h
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_HW_INC_LED_H_
#define CM530_HW_INC_LED_H_

#include "stm32f10x_type.h"

typedef enum LED_e {
    POWER                             = 0,
    MANAGE                            = 1,
    PROGRAM                           = 2,
    PLAY                              = 3,
    TXD                               = 4,
    RXD                               = 5,
    AUX                               = 6
} LED_t;

/**
 * Easy Function to control an LED on the CM-530.
 * Possible LED's on the CM-530 are:
 *   MANAGE, PROGRAM, PLAY, TXD, RXD, AUX, POWER
 * @param led The LED to be controlled.
 * @param state The target state of the LED.
 */
void SetLED(LED_t led, u8 state);


#endif /* CM530_HW_INC_LED_H_ */
