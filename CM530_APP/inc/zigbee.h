/*
 * zigbee.h
 *
 *  Created on: 9 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_APP_INC_ZIGBEE_H_
#define CM530_APP_INC_ZIGBEE_H_

#include "stm32f10x_type.h"
#include "usart.h"
#include "dynamixel_address_tables.h"

//#define DEBUG_ZIGBEE

// RC-100 Button values
#define RC100_BTN_U                     0x0001
#define RC100_BTN_D                     0x0002
#define RC100_BTN_L                     0x0004
#define RC100_BTN_R                     0x0008
#define RC100_BTN_1                     0x0010
#define RC100_BTN_2                     0x0020
#define RC100_BTN_3                     0x0040
#define RC100_BTN_4                     0x0080
#define RC100_BTN_5                     0x0100
#define RC100_BTN_6                     0x0200

#define PACKET_LENGTH                   6
//#define PACKET_LENGTH_ARDUINO                   13
#define PACKET_LENGTH_ARDUINO                   9

extern u8 gbRcvPacket[PACKET_LENGTH];
extern u8 gbRcvPacketArduino[PACKET_LENGTH_ARDUINO];
extern u8 gbRcvPacketNum;
extern u16 gwRcvData;
extern volatile u8 gbRcvFlag;

extern u8 right_V;
extern u8 right_H;
extern u8 left_V;
extern u8 left_H;
extern u8 buttons_;
extern u8 extra;

/**
 * High-level function to initialize the ZigBee Library.
 * @param baudrate The baudrate [bps] to be used.
 * @see zgb_terminate()
 * @return Returns 1 if successful.
 */
u8 zgb_initialize(u32 baudrate);
/**
 * High-level function to terminate the ZigBee Library.
 * @see zgb_initialize()
 */
void zgb_terminate(void);

/**
 * High-level function to send a 16-bit data payload in ZigBee packet.
 * @param data The 16-bit value to be sent.
 * @see zgb_rx_check()
 * @see zgb_rx_data()
 * @return Returns 1 if successful.
 */
u8 zgb_tx_data(u16 data);
/**
 * High-level function to check for a valid received ZigBee packet.
 * @see zgb_rx_data()
 * @return Returns 1 if valid packet available.
 */
u8 zgb_rx_check(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
u16 zgb_rx_data(void);
/**
 * High-level function to check for a valid received ZigBee packet.
 * @see zgb_rx_data()
 * @return Returns 1 if valid packet available.
 */
u8 zgb_rx_check_arduino(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
u16 zgb_rx_data_right_V(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
u16 zgb_rx_data_right_H(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
u16 zgb_rx_data_left_V(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
u16 zgb_rx_data_left_H(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
u8 zgb_rx_data_buttons(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
u8 zgb_rx_data_extra(void);
/**
 * High-level function to retrieve a 16-bit data payload from a ZigBee packet.
 * @see zgb_rx_check()
 * @return The 16-bit data payload.
 */
u16 zgb_rx_data(void);


byte CheckNewArrive(void);

byte RxDByte_Zigbee(void);

signed char zgb_rx_data_right_V_(void);
signed char  zgb_rx_data_right_H_(void);
signed char  zgb_rx_data_left_V_(void);
signed char  zgb_rx_data_left_H_(void);
byte CheckZBNewArrive(void);

#endif /* CM530_APP_INC_ZIGBEE_H_ */
