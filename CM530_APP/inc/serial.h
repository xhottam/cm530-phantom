/*
 * serial.h
 *
 *  Created on: 9 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_APP_INC_SERIAL_H_
#define CM530_APP_INC_SERIAL_H_

#include "stm32f10x_type.h"
#include "dynamixel.h"
#include "usart.h"

#define PACKET_LENGTH_ODROID                   9

extern u8 gbRcvPacketOdroid[PACKET_LENGTH_ODROID];
extern u8 gbRcvPacketNumOdroid;
extern u16 gwRcvDataOdroid;
extern volatile u8 gbRcvFlagOdroid;

extern u8 right_V_Odroid;
extern u8 right_H_Odroid;
extern u8 left_V_Odroid;
extern u8 left_H_Odroid;
extern u8 buttons_Odroid;
extern u8 extra_Odroid;

byte CheckPCNewArrive(void);
byte RxDByte_PC(void);

u8 pc_rx_check_odroid(void);
u8 pc_rx_check_odroid_virtualCommander(void);
signed char pcu_rx_data_right_V_(void);
signed char  pcu_rx_data_right_H_(void);
signed char  pcu_rx_data_left_V_(void);
signed char  pcu_rx_data_left_H_(void);
u8 pcu_rx_data_buttons(void);
u8 pcu_rx_data_extra(void);


/**
 * High-level function to initialize the PC UART Library.
 * @param baudrate The baudrate [bps] to be used.
 * @see pcu_terminate()
 * @return Returns 1 if successful.
 */
u8 pcu_initialize(u32 baudrate);
/**
 * High-level function to terminate the PC UART Library.
 * @see pcu_initialize()
 */
void pcu_terminate(void);
/**
 * Get the number of available characters in the receive buffer/queue.
 * @return The number of available characters.
 */
u8 pcu_get_qstate(void);

// stdio.h compatibility
/**
 * Implementation of the C-Standard putchar() function.
 * @param c The char to be printed to stdout (PC UART).
 * @see std_puts()
 * @return The character printed to stdout (PC UART).
 */
int std_putchar(char);
/**
 * Implementation of the C-Standard puts() function.
 * @param str Pointer to the char array to be printed to stdout (PC UART).
 * @see std_putchar()
 * @return The number of characters printed to stdout (PC UART).
 */
int std_puts(const char*);
/**
 * Implementation of the C-Standard getchar() function.
 * @see std_gets()
 * @return The next character from stdin (PC UART receive buffer/queue).
 */
int std_getchar(void);
/**
 * Implementation of the C-Standard gets() function.
 * @param str Pointer to a char array buffer to receive from stdin (PC UART).
 * @see std_getchar()
 * @return If successful, the input pointer.  If error, a null pointer.
 */
char* std_gets(char*);

/**
 * Print message for any and all errors encountered during dxl_txrx_packet().
 * @param Status The 16-bit error variable returned by dxl_get_result().
 * @see PrintErrorCode()
 */
void PrintCommStatus(u16);
/**
 * Print message for any and all error flags of the incoming dynamixel packet.
 * @see PrintCommStatus()
 */
void PrintErrorCode(void);

/**
 * Wrapper function for putchar().
 * @param c The char to be printed to stdout (PC UART).
 * @see PrintString()
 * @return The character printed to stdout (PC UART).
 */
int PrintChar(char c);
/**
 * Wrapper function for puts().
 * @param str Pointer to the char array to be printed to stdout (PC UART).
 * @see PrintChar()
 * @return The number of characters printed to stdout (PC UART).
 */
int PrintString(const char* s);
/**
 * Wrapper function for getchar().
 * @param str Pointer to a char array buffer to receive from stdin (PC UART).
 * @see GetString()
 * @return If successful, the input pointer.  If error, a null pointer.
 */
int GetChar(void);
/**
 * Wrapper function for gets().
 * @param str Pointer to a char array buffer to receive from stdin (PC UART).
 * @see GetChar()
 * @return If successful, the input pointer.  If error, a null pointer.
 */
char* GetString(char* s);

/**
 * Print an unsigned 32-bit integer to stdout (PC UART).
 * Prints in decimal format without leading zeroes. Partly replicates printf()
 * because of problems getting WinARM to actually link to libc (stdio.h).
 * @param lNum The number to be printed to stdout.
 * @see Prints32d()
 * @see Printu16h()
 * @see Printu8h()
 */
void Printu32d(u32);
/**
 * Print a signed 32-bit integer to stdout (PC UART).
 * Prints in decimal format with a '+' or '-' but without any leading zeroes.
 * Partly replicates printf() because of problems getting WinARM to actually
 * link to libc (stdio.h).
 * @param lNumS The number to be printed to stdout.
 * @see Printu32d()
 * @see Printu16h()
 * @see Printu8h()
 */
void Prints32d(s32);
/**
 * Print an unsigned 16-bit integer to stdout (PC UART).
 * Prints in hexadecimal format with preceding '0x' but without any leading
 * zeroes. Partly replicates printf() because of problems getting WinARM to
 * actually link to libc (stdio.h).
 * @param wNum The number to be printed to stdout.
 * @see Printu8h()
 * @see Printu32d()
 * @see Prints32d()
 */
void Printu16h(u16);
/**
 * Print an unsigned 8-bit integer to stdout (PC UART).
 * Prints in hexadecimal format with preceding '0x' but without any leading
 * zeroes. Partly replicates printf() because of problems getting WinARM to
 * actually link to libc (stdio.h).
 * @param bNum The number to be printed to stdout.
 * @see Printu16h()
 * @see Printu32d()
 * @see Prints32d()
 */
void Printu8h(u8);

void TxD_Dec_U8(u8 bByte);
void TxD_Dec_U16(u16 wData);
void TxD_Dec_U32(u32 wData);
void TxD_Dec_S8(s8 wData);
void TxD_Dec_S16(s16 wData);
void TxD_Dec_S32(s32 lLong);



#endif /* CM530_APP_INC_SERIAL_H_ */
