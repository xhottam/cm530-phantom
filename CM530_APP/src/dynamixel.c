/*
 * dynamixel.c
 *
 *  Created on: 9 Jun 2016
 *      Author: E1193262
 */


#include "dynamixel.h"
#include "system_func.h"
#include "dynamixel_address_tables.h"


u8 gbInstructionPacket[DXL_MAXNUM_TXPARAM] = { 0 };
u8 gbStatusPacket[DXL_MAXNUM_RXPARAM] = { 0 };
u8 gbRxPacketLength = 0;
u8 gbRxGetLength = 0;
volatile u16 gbCommStatus = DXL_RXSUCCESS;
volatile u8 giBusUsing = 0;

//##############################################################################
u8 dxl_initialize(u32 baudrate) {
	if (dxl_hal_open(baudrate) == 0)
		return 0;

	gbCommStatus = DXL_RXSUCCESS;
	giBusUsing = 0;

	return 1;
}

//##############################################################################
void dxl_terminate(void) {
	dxl_hal_close();
}

//##############################################################################
void dxl_tx_packet(void) {
	u8 i;
	u8 TxNumByte, RealTxNumByte;
	u8 checksum = 0;

	if (giBusUsing == 1)
		return;

	giBusUsing = 1;

	gbCommStatus = 0;

	if (gbInstructionPacket[DXL_PKT_LEN] > (DXL_MAXNUM_TXPARAM + 2)) {
		gbCommStatus |= DXL_TXERROR;
		giBusUsing = 0;
		return;
	}

	if ((gbInstructionPacket[DXL_PKT_INST] != INST_PING)
			&& (gbInstructionPacket[DXL_PKT_INST] != INST_READ_DATA)
			&& (gbInstructionPacket[DXL_PKT_INST] != INST_WRITE_DATA)
			&& (gbInstructionPacket[DXL_PKT_INST] != INST_REG_WRITE)
			&& (gbInstructionPacket[DXL_PKT_INST] != INST_ACTION)
			&& (gbInstructionPacket[DXL_PKT_INST] != INST_RESET)
			&& (gbInstructionPacket[DXL_PKT_INST] != INST_SYNC_WRITE)
			&& (gbInstructionPacket[DXL_PKT_INST] != INST_CAP_REGION)) {
		gbCommStatus |= DXL_BAD_INST;
		giBusUsing = 0;
		return;
	}

	gbInstructionPacket[0] = 0xFF;
	gbInstructionPacket[1] = 0xFF;
	for (i = 0; i < (gbInstructionPacket[DXL_PKT_LEN] + 1); i++)
		checksum += gbInstructionPacket[i + 2];
	gbInstructionPacket[gbInstructionPacket[DXL_PKT_LEN] + 3] = ~checksum;

	if (gbCommStatus
			& (DXL_RXFAIL | DXL_RXTIMEOUT | DXL_RXCHECKSUM | DXL_RXLENGTH
					| DXL_BAD_INST | DXL_BAD_ID)) {
		dxl_hal_clear();
	}

	TxNumByte = gbInstructionPacket[DXL_PKT_LEN] + 4;
	RealTxNumByte = dxl_hal_tx((u8*) gbInstructionPacket, TxNumByte);

	if (TxNumByte != RealTxNumByte) {
		gbCommStatus |= DXL_TXFAIL;
		giBusUsing = 0;
		return;
	}

	if (gbInstructionPacket[DXL_PKT_INST] == INST_READ_DATA)
		dxl_hal_set_timeout(gbInstructionPacket[DXL_PKT_PARA + 1] + 6);
	else
		dxl_hal_set_timeout(6);

	gbCommStatus = DXL_TXSUCCESS;
}

//##############################################################################
void dxl_rx_packet(void) {
	u8 i, j, nRead;
	u8 checksum = 0;

	if (giBusUsing == 0)
		return;

	giBusUsing = 1;

	if (gbInstructionPacket[DXL_PKT_ID] == BROADCAST_ID) {
		gbCommStatus = DXL_RXSUCCESS;
		giBusUsing = 0;
		return;
	}

	if (gbCommStatus & DXL_TXSUCCESS) {
		gbRxGetLength = 0;
		gbRxPacketLength = 6;
	}

	nRead = dxl_hal_rx((u8*) &gbStatusPacket[gbRxGetLength],
			gbRxPacketLength - gbRxGetLength);

	gbRxGetLength += nRead;
	if (gbRxGetLength < gbRxPacketLength) {
		if (dxl_hal_timeout() == 1) {
			if (gbRxGetLength == 0)
				gbCommStatus = DXL_RXTIMEOUT;
			else
				gbCommStatus = DXL_RXLENGTH;
			giBusUsing = 0;
			return;
		}
	}

	// Find packet header
	for (i = 0; i < (gbRxGetLength - 1); i++) {
		if ((gbStatusPacket[i] == 0xFF) && (gbStatusPacket[i + 1] == 0xFF)) {
			break;
		} else if ((i == gbRxGetLength - 2)
				&& (gbStatusPacket[gbRxGetLength - 1] == 0xFF)) {
			break;
		}
	}
	if (i > 0) {
		for (j = 0; j < (gbRxGetLength - i); j++)
			gbStatusPacket[j] = gbStatusPacket[j + i];

		gbRxGetLength -= i;
	}

	// Check if received full packet
	if (gbRxGetLength < gbRxPacketLength) {
		gbCommStatus = DXL_RXWAITING;
		return;
	}

	// Check id pairing
	if (gbInstructionPacket[DXL_PKT_ID] != gbStatusPacket[DXL_PKT_ID]) {
		gbCommStatus = DXL_BAD_ID | DXL_RXFAIL;
		giBusUsing = 0;
		return;
	}

	gbRxPacketLength = gbStatusPacket[DXL_PKT_LEN] + 4;
	if (gbRxGetLength < gbRxPacketLength) {
		nRead = dxl_hal_rx((u8*) &gbStatusPacket[gbRxGetLength],
				gbRxPacketLength - gbRxGetLength);
		gbRxGetLength += nRead;
		if (gbRxGetLength < gbRxPacketLength) {
			gbCommStatus = DXL_RXWAITING;
			return;
		}
	}

	// Check checksum
	for (i = 0; i < (gbStatusPacket[DXL_PKT_LEN] + 1); i++)
		checksum += gbStatusPacket[i + 2];
	checksum = ~checksum;

	if (gbStatusPacket[gbStatusPacket[DXL_PKT_LEN] + 3] != checksum) {
		gbCommStatus = DXL_RXCHECKSUM | DXL_RXFAIL;
		giBusUsing = 0;
		return;
	}

	gbCommStatus = DXL_RXSUCCESS;
	giBusUsing = 0;
}

//##############################################################################
void dxl_txrx_packet(void) {

	dxl_tx_packet();

	if (!(gbCommStatus & DXL_TXSUCCESS))
		return;

#ifdef DEBUG_DYNAMIXEL
		PrintString("Dynamixel  --- dxl_txrx_packet ---\n");
		PrintString("TX Comm Status\n");
		TxD_Dec_U16(gbCommStatus);
		PrintString("\n");
#endif

	dxl_clear_statpkt();
	do {
		dxl_rx_packet();
		//uDelay(50);
	} while (gbCommStatus & DXL_RXWAITING);

#ifdef DEBUG_DYNAMIXEL
		PrintString("Dynamixel  --- dxl_txrx_packet ---\n");
		PrintString("RX Comm Status\n");
		TxD_Dec_U16(gbCommStatus);
		PrintString("\n");
#endif
		//zgb_PrintCommStatus(gbCommStatus,gbInstructionPacket[DXL_PKT_ID]);
		//zgb_PrintErrorCode(gbInstructionPacket[DXL_PKT_ID]);

}

//##############################################################################
u16 dxl_get_result(void) {
	return gbCommStatus;
}

//##############################################################################
void dxl_set_txpacket_id(u8 id) {
	gbInstructionPacket[DXL_PKT_ID] = id;
}

//##############################################################################
void dxl_set_txpacket_instruction(u8 instruction) {
	gbInstructionPacket[DXL_PKT_INST] = instruction;
}

//##############################################################################
void dxl_set_txpacket_parameter(u8 index, u8 value) {
	gbInstructionPacket[DXL_PKT_PARA + index] = value;
}

//##############################################################################
void dxl_set_txpacket_length(u8 length) {
	gbInstructionPacket[DXL_PKT_LEN] = length;
}

//##############################################################################
u8 dxl_get_rxpacket_error(u8 errbit) {
	if ((gbCommStatus & DXL_RXFAIL))
		return 0x80;

	if (gbStatusPacket[DXL_PKT_ERR] & errbit)
		return 1;

	return 0;
}

//##############################################################################
u8 dxl_get_rxpacket_length(void) {
	if ((gbCommStatus & DXL_RXFAIL))
		return 0;

	return gbStatusPacket[DXL_PKT_LEN];
}

//##############################################################################
u8 dxl_get_rxpacket_parameter(u8 index) {
	if ((gbCommStatus & DXL_RXFAIL))
		return 0;

	return gbStatusPacket[DXL_PKT_PARA + index];
}

//##############################################################################
u16 dxl_makeword(u8 lowbyte, u8 highbyte) {
	u16 word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return word;
}

//##############################################################################
u8 dxl_get_lowbyte(u16 word) {
	u16 temp = (word & 0x00FF);
	return (u8) temp;
}

//##############################################################################
u8 dxl_get_highbyte(u16 word) {
	u16 temp = ((word & 0xFF00) >> 8);
	return (u8) temp;
}

//##############################################################################
void dxl_ping(u8 id) {
	while (giBusUsing)
		;

	gbInstructionPacket[DXL_PKT_ID] = id;
	gbInstructionPacket[DXL_PKT_INST] = INST_PING;
	gbInstructionPacket[DXL_PKT_LEN] = 2;

	dxl_txrx_packet();
}

//##############################################################################
u8 dxl_read_byte(u8 id, u8 address) {
	while (giBusUsing)
		;

	gbInstructionPacket[DXL_PKT_ID] = id;
	gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
	gbInstructionPacket[DXL_PKT_PARA] = address;
	gbInstructionPacket[DXL_PKT_PARA + 1] = 1;
	gbInstructionPacket[DXL_PKT_LEN] = 4;

	dxl_txrx_packet();

	if ((gbCommStatus & DXL_RXFAIL))
		return 0;

	return gbStatusPacket[DXL_PKT_PARA];
}

//##############################################################################
void dxl_write_byte(u8 id, u8 address, u8 value) {
	while (giBusUsing)
		;

	gbInstructionPacket[DXL_PKT_ID] = id;
	gbInstructionPacket[DXL_PKT_INST] = INST_WRITE_DATA;
	gbInstructionPacket[DXL_PKT_PARA] = address;
	gbInstructionPacket[DXL_PKT_PARA + 1] = value;
	gbInstructionPacket[DXL_PKT_LEN] = 4;

	dxl_txrx_packet();
}

//##############################################################################
u16 dxl_read_word(u8 id, u8 address) {
	while (giBusUsing)
		;

	gbInstructionPacket[DXL_PKT_ID] = id;
	gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
	gbInstructionPacket[DXL_PKT_PARA] = address;
	gbInstructionPacket[DXL_PKT_PARA + 1] = 2;
	gbInstructionPacket[DXL_PKT_LEN] = 4;

	dxl_txrx_packet();

	if ((gbCommStatus & DXL_RXFAIL))
		return 0;

	return dxl_makeword(gbStatusPacket[DXL_PKT_PARA],
			gbStatusPacket[DXL_PKT_PARA + 1]);
}

//##############################################################################
void dxl_write_word(u8 id, u8 address, u16 value) {
	while (giBusUsing)
		;

	gbInstructionPacket[DXL_PKT_ID] = id;
	gbInstructionPacket[DXL_PKT_INST] = INST_WRITE_DATA;
	gbInstructionPacket[DXL_PKT_PARA] = address;
	gbInstructionPacket[DXL_PKT_PARA + 1] = dxl_get_lowbyte(value);
	gbInstructionPacket[DXL_PKT_PARA + 2] = dxl_get_highbyte(value);
	gbInstructionPacket[DXL_PKT_LEN] = 5;

	dxl_txrx_packet();
}

//##############################################################################
void dxl_clear_statpkt(void) {
	u8 i, max = gbStatusPacket[DXL_PKT_LEN];
	if ((max > 0) && (max < DXL_MAXNUM_RXPARAM)) {
		for (i = 0; i < (max + 4); i++)
			gbStatusPacket[i] = 0;
	} else {
		for (i = 0; i < 6; i++)
			gbStatusPacket[i] = 0;
	}
}

//##############################################################################
void dxl_capture(u8 id) {
//    while(giBusUsing);

//    gbInstructionPacket[DXL_PKT_ID] = id;
//    gbInstructionPacket[DXL_PKT_INST] = INST_CAP_REGION;
//    gbInstructionPacket[DXL_PKT_LEN] = 2;

//    dxl_txrx_packet();

	dxl_write_byte(id, 0, 0);
}
//#####################################################################################
byte checkSumatory(byte  data[], int length)
{
    int cs = 0;
    int i ;
    for (i = 2 ; i < length; i++)
    {
        cs += data[i];
    }
    return (byte)~cs;
}
