/*
 * adc.h
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_HW_INC_ADC_H_
#define CM530_HW_INC_ADC_H_

#include "stm32f10x_type.h"
#include "stm32f10x_adc.h"


//#define DEBUG_ADC
#define VBUS_LOW_LIMIT                  115    // 11.5 Volts

typedef enum EPortD_e {
    EPORT11                           = 0,
    EPORT15                           = 1,
    EPORT21                           = 2,
    EPORT25                           = 3,
    EPORT31                           = 4,
    EPORT35                           = 5,
    EPORT41                           = 6,
    EPORT45                           = 7,
    EPORT51                           = 8,
    EPORT55                           = 9,
    EPORT61                           = 10,
    EPORT65                           = 11
} EPortD_t;
typedef enum EPortA_e {
    EPORT1A                           = 0,
    EPORT2A                           = 1,
    EPORT3A                           = 2,
    EPORT4A                           = 3,
    EPORT5A                           = 4,
    EPORT6A                           = 5,
    VBUS                              = 6
} EPortA_t;

typedef enum Motor_e {
    MOTOR1P                           = 0,
    MOTOR1M                           = 1,
    MOTOR2P                           = 2,
    MOTOR2M                           = 3,
    MOTOR3P                           = 4,
    MOTOR3M                           = 5,
    MOTOR4P                           = 6,
    MOTOR4M                           = 7,
    MOTOR5P                           = 8,
    MOTOR5M                           = 9,
    MOTOR6P                           = 10,
    MOTOR6M                           = 11
} Motor_t;

/**
 * Easy Function to control a pin of an external port of the CM-530.
 * Using CM-510/530 pin numbering convention:
 *   Pin 5 is gray (beside GND) and Pin 1 is black (beside VCC).
 *     It is very important to note that the numbering convention is reversed
 *     between the IR sensor/Servo motor pages and the CM-510/530 pages on
 *     the Robotis e-Support site.
 * @param pin The pin to be controlled.
 * @param state The target state of the pin.
 */
void SetEPort(EPortD_t pin, u8 state);
/**
 * Easy Function to read analog pin (Pin 3) of an external port of the CM-530.
 * @param port The external port to retrieve the analog value.
 * @return The 12-bit analog value from the analog pin.
 */
u16 ReadAnalog(EPortA_t port);
/**
 * Easy Function to get an analog value from an IR Module connected to CM-530.
 * Will automatically turn on the IR Module, wait a fixed time period, read
 * the ADC value, turn off the IR Module, and return the ADC value representing
 * a distance.
 * @param port The external port to retrieve the analog value.
 * @return The 12-bit analog value from the analog pin.
 */
u16 ReadIR(EPortA_t port);

//#define BATTERY_MONITOR_INTERVAL        10    // Check once every 10 seconds
//void start_timeout_bat(uint32_t);

/**
 * Function to check battery voltage and sound an alarm if below threshold.
 */
void Battery_Monitor_Alarm(void);


#endif /* CM530_HW_INC_ADC_H_ */
