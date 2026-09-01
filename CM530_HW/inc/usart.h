/*
 * usart.h
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_HW_INC_USART_H_
#define CM530_HW_INC_USART_H_

#include "stm32f10x_type.h"
#include "stm32f10x_usart.h"
#include "dynamixel_address_tables.h"

#define USING_DYNAMIXEL
#define USING_PC_UART
//#define VIRTUAL_COMANDER
#define USING_ZIGBEE

#ifdef USING_DYNAMIXEL
	extern u32 Baudrate_DXL;
#endif
#ifdef USING_ZIGBEE
	extern u32 Baudrate_ZIG;
#endif
#ifdef USING_PC_UART
	extern u32 Baudrate_PCU;
#endif


	static volatile u8 ReBootToBootLoader;

#ifdef USING_PC_UART
 //##############################################################################
//##############################################################################
// Serial/PC_UART platform dependent source
//##############################################################################
//##############################################################################
//#define PC_UART_BUFFER_LENGTH           32
#define PC_UART_BUFFER_LENGTH             1023

static volatile u16 gbPcuWrite, gbPcuRead;
static volatile u8 gbpPcuBuffer[PC_UART_BUFFER_LENGTH+1] = { 0 };
static volatile u8 ReBootToBootLoader;

u8 pcu_hal_open(u32);
void pcu_hal_close(void);
void pcu_hal_set_timeout(u8);
u8 pcu_hal_timeout(void);

void pcu_put_byte(u8);
u8 pcu_get_queue(void);
u8 pcu_peek_queue(void);
//u8 pcu_get_qstate(void);
void pcu_clear_queue(void);
void pcu_put_queue(void);
u8 pcu_hal_rx(u8 *pPacket, u8 numPacket);
void RxD_PCU_Interrupt(void);


#endif

 #ifdef USING_ZIGBEE
//##############################################################################
//##############################################################################
// Zigbee SDK platform dependent source
//##############################################################################
//##############################################################################
//#define ZIGBEE_BUFFER_LENGTH            64
//#define ZIGBEE_BUFFER_LENGTH            32
#define ZIGBEE_BUFFER_LENGTH            1023
static volatile u8 gbZigWrite=0, gbZigRead=0;
static volatile u8 gbpZigBuffer[ZIGBEE_BUFFER_LENGTH+1] = { 0 };

u8 zgb_hal_open(u32);
void zgb_hal_close(void);
u8 zgb_hal_tx(u8*, u8);
u8 zgb_hal_rx(u8*, u8);
void RxD_ZIG_Interrupt(void);
void zgb_clear_queue(void);

#endif

#ifdef USING_DYNAMIXEL
//##############################################################################
//##############################################################################
// Dynamixel SDK platform dependent source
//##############################################################################
//##############################################################################

#define DXL_BUFFER_LENGTH               1023

static volatile u16 gbDxlWrite = 0, gbDxlRead = 0;
static volatile u8 gbpDxlBuffer[DXL_BUFFER_LENGTH+1];

u8 dxl_hal_open(u32);
void dxl_hal_close(void);
void dxl_hal_clear(void);
u8 dxl_hal_tx(u8*, u8);
u8 dxl_hal_rx(u8*, u8);
void dxl_hal_set_timeout(u8);
u8 dxl_hal_timeout(void);
void RxD_DXL_Interrupt(void);

#endif

#endif /* CM530_HW_INC_USART_H_ */
