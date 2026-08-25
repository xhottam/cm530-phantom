/*
 * dynamixel.h
 *
 *  Created on: 9 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_APP_INC_DYNAMIXEL_H_
#define CM530_APP_INC_DYNAMIXEL_H_

#include "stm32f10x_type.h"
#include "usart.h"
#include "dynamixel_address_tables.h"

//#define DEBUG_DYNAMIXEL

#define Relax(id) (dxl_write_byte(id, AXM_TORQUE_ENABLE, 0))
#define TorqueOn(id) (dxl_write_byte(id, AXM_TORQUE_ENABLE, 1))
#define TorqueOff(id) (dxl_write_byte(id, AXM_TORQUE_ENABLE, 0))
#define DXL_MAXNUM_TXPARAM                  160
#define DXL_MAXNUM_RXPARAM                  80

extern u8 gbInstructionPacket[DXL_MAXNUM_TXPARAM];
extern u8 gbStatusPacket[DXL_MAXNUM_RXPARAM];
extern u8 gbRxPacketLength;
extern u8 gbRxGetLength;
extern volatile u16 gbCommStatus;
extern volatile u8 giBusUsing;

void dxl_tx_packet(void);
void dxl_rx_packet(void);
void dxl_clear_statpkt(void);

/**
 * High-level function to initialize the Dynamixel Library.
 * @param baudrate The baudrate [bps] to be used.
 * @see dxl_terminate()
 * @return Returns 1 if successful.
 */
u8 dxl_initialize(u32 baudrate);
/**
 * High-level function to terminate the Dynamixel Library.
 * @see dxl_initialize()
 */
void dxl_terminate(void);

/**
 * Set the outgoing packet ID byte.
 * @param id The target device's ID.
 */
void dxl_set_txpacket_id(u8 id);
/**
 * Set the outgoing packet Instruction byte.
 * @param instruction The Instruction byte (read, write, etc.).
 */
void dxl_set_txpacket_instruction(u8 instruction);
/**
 * Set one parameter byte of the outgoing packet.
 * @param index The position of the parameter byte in the outgoing packet.
 * @param value The value to be stored at 'index' of the outgoing packet.
 */
void dxl_set_txpacket_parameter(u8 index, u8 value);
/**
 * Set the length of the outgoing packet.
 * @param length The number of parameter bytes + 2.
 */
void dxl_set_txpacket_length(u8 length);

/**
 * Check the incoming packet's error byte for an active flag.
 * @param errbit The bit mask of the error flag you seek to check.
 * @see dxl_get_result()
 * @return Returns 1 if the flag is set.
 */
u8 dxl_get_rxpacket_error(u8 errbit);
/**
 * Get the value stored at a location in the incoming packet.
 * @param index The position of the parameter byte in the incoming packet.
 * @see dxl_get_rxpacket_length()
 * @return Returns the value of the parameter byte.
 */
u8 dxl_get_rxpacket_parameter(u8 index);
/**
 * Get the length of the incoming packet.
 * @see dxl_get_rxpacket_parameter()
 * @return Returns the number of paramter bytes + 2.
 */
u8 dxl_get_rxpacket_length(void);

/**
 * Makes a 16-bit word from two input bytes.
 * @param lowbyte The low byte of the word.
 * @param highbyte The high byte of the word.
 * @return Returns a 16-bit word.
 */
u16 dxl_makeword(u8 lowbyte, u8 highbyte);
/**
 * Gets the low byte of a 16-bit word.
 * @param word The word of which to find the low byte.
 * @see dxl_get_highbyte()
 * @return The low byte of the word.
 */
u8 dxl_get_lowbyte(u16 word);
/**
 * Gets the low byte of a 16-bit word.
 * @param word The word of which to find the high byte.
 * @see dxl_get_lowbyte()
 * @return The high byte of the word.
 */
u8 dxl_get_highbyte(u16 word);

/**
 * Send and receive a dynamixel packet.
 * Must be called after constructing a custom packet.
 * @see dxl_get_result()
 * @see dxl_get_rxpacket_error()
 */
void dxl_txrx_packet(void);

/**
 * Retrieves the 16-bit error word produced by dxl_txrx_packet().
 * Each bit indicates a different error or success flag from
 * the attempt to send a dynamixel packet and receive the response
 * from the device (if any).
 * @see dxl_txrx_packet()
 * @see dxl_get_rxpacket_error()
 * @return Returns the error word.
 */
u16 dxl_get_result(void);

/**
 * High-level function to send a ping packet to a device.
 * @param id The ID of the device to ping.
 */
void dxl_ping(u8 id);
/**
 * High-level function to read a single byte from a device.
 * @param id The ID of the target device.
 * @param address The address in the device table of the byte to be read.
 * @see dxl_read_word()
 * @return The value of the byte read from the device.
 */
u8 dxl_read_byte(u8 id, u8 address);
/**
 * High-level function to write a single byte to a device.
 * @param id The ID of the target device.
 * @param address The address in the device table of the byte to be written.
 * @param value The value to be written to that address.
 * @see dxl_write_word()
 */
void dxl_write_byte(u8 id, u8 address, u8 value);
/**
 * High-level function to read a 16-bit word from a device.
 * @param id The ID of the target device.
 * @param address The address in the device table of the word to be read.
 * @see dxl_read_byte()
 * @return The value of the two consecutive bytes read from the device.
 */
u16 dxl_read_word(u8 id, u8 address);
/**
 * High-level function to write a 16-bit word to a device.
 * @param id The ID of the target device.
 * @param address The address in the device table of the word to be written.
 * @param value The value to be written to that address.
 * @see dxl_write_byte()
 */
void dxl_write_word(u8 id, u8 address, u16 value);

/**
 * High-level function to begin an image capture with the HaViMo2 camera module.
 * @param id HaViMo2 camera ID (fixed as 100 in HaViMo2 firmware).
 * @see dxl_recover()
 */
void dxl_capture(u8 id);
/**
 * High-level function to retrieve an image buffer from a HaViMo2 camera module.
 * @param id HaViMo2 camera ID (fixed as 100 in HaViMo2 firmware).
 * @param hvm2rb Pointer to a user region buffer data type.
 * @see dxl_capture()
 * @return The number of valid regions found in the image.
 */
byte checkSumatory(byte  data[], int length);
#endif /* CM530_APP_INC_DYNAMIXEL_H_ */
