/*
 * usart.c
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#include "usart.h"
#include "led.h"
#include "system_init.h"
#include "system_func.h"

#ifdef USING_DYNAMIXEL
	u32 Baudrate_DXL = 1000000;
#endif
#ifdef USING_ZIGBEE
	u32 Baudrate_ZIG = 57600;
#endif
#ifdef USING_PC_UART
	u32 Baudrate_PCU = 57600;
	//u32 Baudrate_PCU = 38400;
	//u32 Baudrate_PCU = 1000000;
#endif


#ifdef USING_PC_UART
//##############################################################################
u8 pcu_hal_open(u32 baudrate) {
	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_DeInit(USART3);
	mDelay(10);
	// Configure USART3 (PC_UART)
	USART_Init(USART3, &USART_InitStructure);

	// Enable USART3 (PC_UART) Receive interrupt
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	// Enable USART3 (PC_UART)
	USART_Cmd(USART3, ENABLE);

//    PC_UART_Device = fdevopen( std_putchar, std_getchar );

	return 1;
}

//##############################################################################
void pcu_hal_close(void) {
	// Disable USART3 (PC UART)
	USART_Cmd(USART3, DISABLE);
}

//##############################################################################
void pcu_hal_set_timeout(u8 NumRcvByte) {
	// 200us; ~180 us to transmit one byte at 57600 bps
	start_timeout_pcu(NumRcvByte * 200);
}

//##############################################################################
u8 pcu_hal_timeout(void) {
	if (glPcuTimeoutCounter == 0)
		return 1;
	else
		return 0;
}

//##############################################################################
void pcu_put_byte(u8 bTxdData) {
	//SetLED(TXD, 1);

	USART_SendData(USART3, bTxdData);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		;

	//SetLED(TXD, 0);
}

//##############################################################################
u8 pcu_get_queue(void) {
	if (gbPcuWrite == gbPcuRead)
		return 0xFF;

	u8 data = gbpPcuBuffer[gbPcuRead++];

	if (gbPcuRead > (PC_UART_BUFFER_LENGTH - 1))
		gbPcuRead = 0;

	return data;
}

//##############################################################################
u8 pcu_peek_queue(void) {
	if (gbPcuWrite == gbPcuRead)
		return 0xFF;

	u8 data = gbpPcuBuffer[gbPcuRead];

	return data;
}

//##############################################################################
void pcu_put_queue(void) {
	u8 temp;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		temp = USART_ReceiveData(USART3);
		if (temp == '#')
			ReBootToBootLoader++;
		else
			ReBootToBootLoader = 0;

		if (ReBootToBootLoader > 15) {
#ifdef USING_BREAK_TO_BOOTLOADER
			//BreakToBootLoader();
#endif
		}
	} else
		return;

	//SetLED(RXD, 1);

	if (gbPcuWrite < (PC_UART_BUFFER_LENGTH - 1)) {
		gbpPcuBuffer[gbPcuWrite++] = temp;
	} else {
		gbpPcuBuffer[gbPcuWrite] = temp;
		gbPcuWrite = 0;
	}

	if (gbPcuRead == gbPcuWrite)
		gbPcuRead++;
	if (gbPcuRead > (PC_UART_BUFFER_LENGTH - 1))
		gbPcuRead = 0;

	//SetLED(RXD, 0);
}

//##############################################################################
void pcu_clear_queue(void) {
	gbPcuWrite = 0;
	gbPcuRead = 0;
}

//##############################################################################
u8 pcu_get_qstate(void) {
	if (gbPcuWrite == gbPcuRead) {
		pcu_clear_queue();
		return 0;
	} else if (gbPcuRead < gbPcuWrite)
		return (u8) (gbPcuWrite - gbPcuRead);
	else
		return (u8) (PC_UART_BUFFER_LENGTH - (gbPcuRead - gbPcuWrite));
}
//##############################################################################
u8 pcu_hal_rx(u8 *pPacket, u8 numPacket)
{
	u8 i;
	for (i=0; i<numPacket; i++)
	{
		if (gbPcuRead!=gbPcuWrite)
		{
			pPacket[i] = gbpPcuBuffer[gbPcuRead++];
			if (gbPcuRead>(PC_UART_BUFFER_LENGTH-1))
				gbPcuRead = 0;
		}
		else
		return i;
	}

	return numPacket;
}
//##############################################################################
void RxD_PCU_Interrupt(void) {
	pcu_put_queue();
}
#endif

#ifdef USING_ZIGBEE
//##############################################################################
u8 zgb_hal_open(u32 baudrate)
{
	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_DeInit(UART5);
	mDelay(10);
	// Configure UART5 (ZigBee)
	USART_Init(UART5, &USART_InitStructure);

	// Enable UART5 (ZigBee) Receive interrupt
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

	// Enable UART5 (ZigBee)
	USART_Cmd(UART5, ENABLE);

	// Disable Zigbee
	//GPIO_SetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);
	//Enable Zigbee
	GPIO_ResetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);

	return 1;
}

//##############################################################################
void zgb_hal_close(void)
{
	// Disable UART5 (ZigBee)
	USART_Cmd(UART5, DISABLE);
	// Activate Reset mode of ZIG-110 module
//    GPIO_SetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);    // original
	GPIO_ResetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);// correct?
}

//##############################################################################
u8 zgb_hal_tx(u8 *pPacket, u8 numPacket)
{
	u8 i;
	for (i=0; i<numPacket; i++)
	{
		//SetLED(TXD, 1);

		USART_SendData(UART5,pPacket[i]);
		while (USART_GetFlagStatus(UART5, USART_FLAG_TC)==RESET);

		//SetLED(TXD, 0);
	}

	return numPacket;
}

//##############################################################################
u8 zgb_hal_rx(u8 *pPacket, u8 numPacket)
{
	u8 i;
	for (i=0; i<numPacket; i++)
	{
		if (gbZigRead!=gbZigWrite)
		{
			pPacket[i] = gbpZigBuffer[gbZigRead++];
			if (gbZigRead>(ZIGBEE_BUFFER_LENGTH-1))
			gbZigRead = 0;
		}
		else
		return i;
	}

	return numPacket;
}
//##############################################################################
void RxD_ZIG_Interrupt(void)
{
	u8 temp;
	if (USART_GetITStatus(UART5, USART_IT_RXNE)!=RESET)
	{
		temp = USART_ReceiveData(UART5);
		if (temp=='#'){
		ReBootToBootLoader++;

		}
		else
		ReBootToBootLoader=0;

		if (ReBootToBootLoader>15)
		{

#ifdef USING_BREAK_TO_BOOTLOADER
			BreakToBootLoader();
#endif
		}
	}
	else
	return;

	//SetLED(RXD, 1);

	if (gbZigWrite<(ZIGBEE_BUFFER_LENGTH-1))
	{
		gbpZigBuffer[gbZigWrite++] = temp;
	}
	else
	{
		gbpZigBuffer[gbZigWrite] = temp;
		gbZigWrite = 0;
	}

	if (gbZigRead==gbZigWrite)
	gbZigRead++;
	if (gbZigRead>(ZIGBEE_BUFFER_LENGTH-1))
	gbZigRead=0;

	//SetLED(RXD, 0);
}

void zgb_clear_queue(void) {
	gbZigWrite = 0;
	gbZigRead = 0;
}

#endif

#ifdef USING_DYNAMIXEL
//##############################################################################
u8 dxl_hal_open(u32 baudrate) {
	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_DeInit(USART1);
	mDelay(10);
	// Configure USART1 (dynamixel)
	USART_Init(USART1, &USART_InitStructure);

	// Enable USART1 (dynamixel) Receive interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	// Enable USART1 (dynamixel)
	USART_Cmd(USART1, ENABLE);

	return 1;
}

//##############################################################################
void dxl_hal_close(void) {
	// Disable USART1 (dynamixel)
	USART_Cmd(USART1, DISABLE);
}

//##############################################################################
void dxl_hal_clear(void) {
	// Clear communication buffer
	u16 i;
	for (i = 0; i < DXL_BUFFER_LENGTH; i++)
		gbpDxlBuffer[i] = 0;
	gbDxlRead = 0;
	gbDxlWrite = 0;
}

//##############################################################################
u8 dxl_hal_tx(u8 *pPacket, u8 numPacket) {
	u8 i;
	for (i = 0; i < numPacket; i++) {
		// RX Disable
		GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);
		// TX Enable
		GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);

		USART_SendData(USART1, pPacket[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			;

		// TX Disable
		GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);
		// RX Enable
		GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);
	}

	return numPacket;
}

//##############################################################################
u8 dxl_hal_rx(u8 *pPacket, u8 numPacket) {
	u8 i;
	for (i = 0; i < numPacket; i++) {
		if (gbDxlRead != gbDxlWrite) {
			pPacket[i] = gbpDxlBuffer[gbDxlRead++];
			if (gbDxlRead > (DXL_BUFFER_LENGTH - 1))
				gbDxlRead = 0;
		} else
			return i;
	}

	return numPacket;
}

//##############################################################################
void dxl_hal_set_timeout(u8 NumRcvByte) {
	start_timeout_dxl(NumRcvByte * 30);
	//StartDiscount(NumRcvByte*30);
	//StartDiscount(NumRcvByte*100);
}

//##############################################################################
u8 dxl_hal_timeout(void) {
	if (glDxlTimeoutCounter == 0)
		return 1;
	else
		return 0;
//	/return CheckTimeOut();
}

//##############################################################################
void RxD_DXL_Interrupt(void) {
	u8 temp;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		temp = USART_ReceiveData(USART1);
	} else
		return;

	if (gbDxlWrite < (DXL_BUFFER_LENGTH - 1)) {
		gbpDxlBuffer[gbDxlWrite++] = temp;
	} else {
		gbpDxlBuffer[gbDxlWrite] = temp;
		gbDxlWrite = 0;
	}

	if (gbDxlRead == gbDxlWrite)
		gbDxlRead++;
	if (gbDxlRead > (DXL_BUFFER_LENGTH - 1))
		gbDxlRead = 0;
}


#endif
