/*
 * zigbee.c
 *
 *  Created on: 9 Jun 2016
 *      Author: E1193262
 */


#include "zigbee.h"

#ifdef USING_ZIGBEE

u8 gbRcvPacket[PACKET_LENGTH];
u8 gbRcvPacketArduino[PACKET_LENGTH_ARDUINO];
u8 gbRcvPacketNum;
u16 gwRcvData;
volatile u8 gbRcvFlag;

u8 right_V;
u8 right_H;
u8 left_V;
u8 left_H;
u8 buttons_;
u8 extra;


//##############################################################################
u8 zgb_initialize(u32 baudrate)
{
	if (zgb_hal_open(baudrate)==0)
	return 0;

	gbRcvFlag = 0;
	gwRcvData = 0;
	gbRcvPacketNum = 0;
	return 1;
}

//##############################################################################
void zgb_terminate(void)
{
	zgb_hal_close();
}

//##############################################################################
u8 zgb_tx_data(u16 word)
{
	u8 SndPacket[6];
	u8 lowbyte = (u8) (word&0xFF);
	u8 highbyte = (u8) ((word>>8)&0xFF);

	SndPacket[0] = 0xFF;
	SndPacket[1] = 0x55;
	SndPacket[2] = lowbyte;
	SndPacket[3] = ~lowbyte;
	SndPacket[4] = highbyte;
	SndPacket[5] = ~highbyte;

	if (zgb_hal_tx(SndPacket, 6)!=6)
	return 0;

	return 1;
}

//##############################################################################
u8 zgb_rx_check(void)
{
	u8 RcvNum;
	u8 checksum;
	u8 i, j;

	if (gbRcvFlag==1)
	return 1;

	// Fill packet buffer
	if (gbRcvPacketNum<6)
	{
		RcvNum = zgb_hal_rx((u8*)&gbRcvPacket[gbRcvPacketNum], (6-gbRcvPacketNum));
		if (RcvNum!=-1)
		gbRcvPacketNum += RcvNum;
	}

	// Find header
	if (gbRcvPacketNum>=2)
	{
		for (i=0; i<gbRcvPacketNum; i++)
		{
			if (gbRcvPacket[i]==0xFF)
			{
				if (i<=(gbRcvPacketNum-2))
				{
					if (gbRcvPacket[i+1]==0x55)
					break;
				}
			}
		}

		if (i>0)
		{
			if (i==gbRcvPacketNum)
			{
				// Cannot find header
				if (gbRcvPacket[i-1]==0xFF)
				i--;
			}

			// Remove data before header
			for (j=i; j<gbRcvPacketNum; j++)
			{
				gbRcvPacket[j-i] = gbRcvPacket[j];
			}
			gbRcvPacketNum -= i;
		}
	}

	// Verify packet
	if (gbRcvPacketNum==6)
	{
		if ( (gbRcvPacket[0]==0xFF) && (gbRcvPacket[1]==0x55) )
		{
			checksum = ~gbRcvPacket[3];
			if (gbRcvPacket[2]==checksum)
			{
				checksum = ~gbRcvPacket[5];
				if (gbRcvPacket[4]==checksum)
				{
					gwRcvData = (u16) ((gbRcvPacket[4]<<8)&0xFF00);
					gwRcvData += gbRcvPacket[2];
					gbRcvFlag = 1;
				}
			}
		}
		gbRcvPacket[0] = 0x00;
		gbRcvPacketNum = 0;
	}

	return gbRcvFlag;
}

//##############################################################################
u16 zgb_rx_data(void)
{
	gbRcvFlag = 0;
	return gwRcvData;
}

//##############################################################################
u8 zgb_rx_check_arduino(void)
{

    u8 RcvNum;
    u8 checksum;
    u8 i, j;

    if (gbRcvFlag==1){
    	//PrintString("\n NO \n");
    	return 1;
    }

    // Fill packet buffer nueve bytes
    if (gbRcvPacketNum<9) {
    	// Fill packet array until 9 bytes.
        RcvNum = zgb_hal_rx((u8*)&gbRcvPacketArduino[gbRcvPacketNum], (9-gbRcvPacketNum));
        if (RcvNum!=-1)
            gbRcvPacketNum += RcvNum;
    }


    // Find header
    if (gbRcvPacketNum>=2)
    {
        for (i=0; i<gbRcvPacketNum; i++)
        {
            if (gbRcvPacketArduino[i]==0xFF)
            {
                if (i<=(gbRcvPacketNum-2))
                {
                    if (gbRcvPacketArduino[i+1]==0x55)
                        break;
                }
            }
        }
        if (i>0)
        {
            if (i==gbRcvPacketNum)
            {
                // Cannot find header
                if (gbRcvPacketArduino[i-1]==0xFF)

                    i--;
            }
            // Remove data before header
            for (j=i; j<gbRcvPacketNum; j++)
            {

                gbRcvPacketArduino[j-i] = gbRcvPacketArduino[j];
            }
            gbRcvPacketNum -= i;
        }
    }


    // Verify packet
    if (gbRcvPacketNum==9)
    {

        if ( (gbRcvPacketArduino[0]==0xFF) && (gbRcvPacketArduino[1]==0x55) )
        {
            /**checksum = ~gbRcvPacket[3];
            if (gbRcvPacket[2]==checksum)
            {
                checksum = ~gbRcvPacket[5];
                if (gbRcvPacket[4]==checksum)
                {
                    gwRcvData = (u16) ((gbRcvPacket[4]<<8)&0xFF00);
                    gwRcvData += gbRcvPacket[2];
                    gbRcvFlag = 1;
                }
            }*/


                  		left_V  =   gbRcvPacketArduino[2];
        	            left_H  =   gbRcvPacketArduino[3];
        	            right_V =   gbRcvPacketArduino[4];
        	            right_H =   gbRcvPacketArduino[5];
        	            buttons_=   gbRcvPacketArduino[6];
        	            extra   =   gbRcvPacketArduino[7];
        	            checksum=   gbRcvPacketArduino[8];
#ifdef DEBUG_ZIGBEE
	PrintString("ZIGBEE --- zgb_rx_check_arduino ---\n");
	PrintString("rightV rightH leftV leftH buttons ext\n");
	TxD_Dec_U8(right_V);
	PrintString(" ");
	TxD_Dec_U8(right_H);
	PrintString(" ");
	TxD_Dec_U8(left_V);
	PrintString(" ");
	TxD_Dec_U8(left_H);
	PrintString(" ");
	TxD_Dec_U8(buttons_);
	PrintString(" ");
	TxD_Dec_U8(extra);
	PrintString("\n");
#endif

			// No sumo extra al checksum see esploraBioloid.ino
            if (checksum == (255 - (right_V+right_H+left_V+left_H+buttons_)%256)){        	  //if (checksum == (255 - (gbRcvPacketArduino[2]+gbRcvPacketArduino[3]+gbRcvPacketArduino[4]+gbRcvPacketArduino[5]+gbRcvPacketArduino[6])%256)){


				 gbRcvFlag = 1;
            }

        }
        gbRcvPacketArduino[0] = 0x00;
        gbRcvPacketNum = 0;
    }

    return gbRcvFlag;
}

//##############################################################################
u16 zgb_rx_data_right_V(void)
{
    //gbRcvFlag = 0;
    return right_V;
}
//##############################################################################
u16 zgb_rx_data_right_H(void)
{
    //gbRcvFlag = 0;
    return right_H;
}
//##############################################################################
u16 zgb_rx_data_left_V(void)
{
    //gbRcvFlag = 0;
    return left_V;
}
//##############################################################################
u16 zgb_rx_data_left_H(void)
{
    //gbRcvFlag = 0;
    return left_H;
}
//##############################################################################
u8 zgb_rx_data_buttons(void)
{
    //gbRcvFlag = 0;
    return buttons_;
}
//##############################################################################
u8 zgb_rx_data_extra(void)
{
    gbRcvFlag = 0;
    return 0;

}
//##############################################################################
signed char  zgb_rx_data_right_V_(void)
{
    //gbRcvFlag = 0;
    return (right_V - 128);
}
//##############################################################################
signed char  zgb_rx_data_right_H_(void)
{
    //gbRcvFlag = 0;
    return (right_H - 128);
}
//##############################################################################
signed char  zgb_rx_data_left_V_(void)
{
    //gbRcvFlag = 0;
    return (left_V - 128);
}
//##############################################################################
signed char  zgb_rx_data_left_H_(void)
{
    //gbRcvFlag = 0;
    return (left_H- 128);
}
//##############################################################################
bool CheckZBNewArrive(void) {

	if (gbZigRead != gbZigWrite)
		return TRUE;
	else
		return FALSE;
}
//##############################################################################
int zgb_PrintString(const char* s) {
	return zgb_std_puts(s);
	mDelay(10);
}
//##############################################################################
int zgb_std_putchar(char c) {
	if (c == '\n') {
		zgb_pcu_put_byte((u8) '\r'); //0x0D
		zgb_pcu_put_byte((u8) '\n'); //0x0A
	} else {
		zgb_pcu_put_byte((u8) c);
	}

	return c;
}

//##############################################################################
int zgb_std_puts(const char *str) {
	int n = 0;
	while (str[n])
		zgb_std_putchar(str[n++]);

	return n;
}
//##############################################################################
void zgb_pcu_put_byte(u8 bTxdData) {
	//setLED(TXD, 1);

	USART_SendData(UART5,bTxdData);
	while (USART_GetFlagStatus(UART5, USART_FLAG_TC)==RESET);

	//SetLED(TXD, 0);
}
//##############################################################################
void zgb_Printu32d(u32 lNum) {
	u32 temp, div = 1000000000;
	char out[11];
	u8 i, j;

	for (i = 0; i < 10; i++) {
		temp = (char) (lNum / div);
		lNum = (lNum % div);
//        lNum -= (u32) (temp*div);
//        out[i] = (char) (temp&0x0000000F)+0x30;
		out[i] = (char) ((temp & 0x0F) + 0x30);
		div /= 10;
	}
	out[i] = '\0';

	for (i = 0; i < 10; i++) {
		if (out[0] == '0') {
			for (j = 0; j < 10; j++) {
				out[j] = out[j + 1];
				if (out[j] == '\0')
					break;
			}
		}
	}

	zgb_std_puts(out);
	mDelay(10);
	return;
}
//##############################################################################
bool zgb_PrintCommStatus(u16 Status,u8 id) {
	if (Status & DXL_TXFAIL){
		zgb_std_puts("DXL_TXFAIL: Failed transmit instruction packet!\n");
		zgb_TxD_Dec_U8(id);
		return FALSE;
	}

	if (Status & DXL_RXFAIL){
		zgb_std_puts("DXL_RXFAIL: Failed get status packet from device!\n");
		zgb_TxD_Dec_U8(id);
		return FALSE;
	}

	if (Status & DXL_TXERROR){
		zgb_std_puts("DXL_TXERROR: Incorrect instruction packet!\n");
		zgb_TxD_Dec_U8(id);
		return FALSE;
	}

	if (Status & DXL_BAD_INST){
		zgb_std_puts("DXL_BAD_INST: Invalid Instruction byte\n");
		zgb_TxD_Dec_U8(id);
		return FALSE;
	}

	if (Status & DXL_BAD_ID){
		zgb_std_puts("DXL_BAD_ID: ID's not same for instruction and status packets\n");
		zgb_TxD_Dec_U8(id);
		return FALSE;
	}

	if (Status & DXL_RXWAITING){
		zgb_std_puts("DXL_RXWAITING: Now receiving status packet!\n");
		zgb_TxD_Dec_U8(id);
		return FALSE;
	}

	if (Status & DXL_RXTIMEOUT){
		zgb_std_puts("DXL_RXTIMEOUT: There is no status packet!\n");
		zgb_TxD_Dec_U8(id);
		return FALSE;
	}

	if (Status & DXL_RXCHECKSUM){
		zgb_std_puts("DXL_RXCHECKSUM: Incorrect status packet checksum!\n");
		zgb_TxD_Dec_U8(id);
		return FALSE;
	}

	return TRUE;

//    else
//        std_puts("\nThis is unknown error code!\n");
}
//##############################################################################
void zgb_TxD_Dec_U16(u16 wData)
{
    u8 bCount, bPrinted;
    u16 wTmp,wDigit;
    bPrinted = 0;

    wDigit = 10000;
    for(bCount = 0; bCount < 5; bCount++)
    {
        wTmp = (wData/wDigit);
        if(wTmp)
        {
            zgb_pcu_put_byte( ((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) zgb_pcu_put_byte( ((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) zgb_pcu_put_byte( ' ');
                else zgb_pcu_put_byte( '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
    mDelay(10);
}
//##############################################################################
void zgb_TxD_Dec_U8(u8 bByte)
{
    u8 bTmp;
    bTmp = bByte/100;
    /*if(bTmp)*/ zgb_pcu_put_byte(bTmp+'0');
    bByte -= bTmp*100;
    bTmp = bByte/10;
    /*if(bTmp)*/ zgb_pcu_put_byte( bTmp+'0');
    zgb_pcu_put_byte( bByte - bTmp*10+'0');
    mDelay(10);
}
//##############################################################################
void zgb_TxD_Dec_U32(u32 wData)
{
    u8 bCount, bPrinted;
    u32 wTmp,wDigit;
    bPrinted = 0;

    wDigit = 1000000000;

    for(bCount = 0; bCount < 10; bCount++)
    {
        wTmp = (wData/wDigit);
        if(wTmp)
        {
            zgb_pcu_put_byte( ((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) zgb_pcu_put_byte( ((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) zgb_pcu_put_byte( ' ');
                else zgb_pcu_put_byte( '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
    mDelay(10);
}
//##############################################################################
void zgb_TxD_Dec_S8(s8 wData)
{
    u8 bCount, bPrinted;
    u16 wTmp,wDigit;
    u8 bMinus = 0;

    bPrinted = 0;

    if (wData&0x80) {
        bMinus = 1;
        wData = -wData;
    }

    wDigit = 100;
    for(bCount = 0; bCount < 3; bCount++)
    {
        wTmp = (wData/wDigit);
        if(wTmp && !bPrinted)
        {
            if (bMinus) zgb_pcu_put_byte( '-');
            zgb_pcu_put_byte( ((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) zgb_pcu_put_byte( ((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) zgb_pcu_put_byte( ' ');
                else zgb_pcu_put_byte(  '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
    mDelay(10);
}
//##############################################################################
void zgb_TxD_Dec_S16(s16 wData)
{
    u8 bCount, bPrinted;
    u16 wTmp,wDigit;
    u8 bMinus = 0;

    bPrinted = 0;

    if (wData&0x8000) {
        bMinus = 1;
        wData = -wData;
    }

    wDigit = 10000;
    for(bCount = 0; bCount < 5; bCount++)
    {
        wTmp = (wData/wDigit);
        if(wTmp && !bPrinted)
        {
            if (bMinus) zgb_pcu_put_byte( '-');
            zgb_pcu_put_byte( ((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) zgb_pcu_put_byte( ((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) zgb_pcu_put_byte( ' ');
                else zgb_pcu_put_byte(  '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
    mDelay(10);
}
//##############################################################################
void zgb_PrintErrorCode(u8 id) {

	if (dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1){
		zgb_TxD_Dec_U8(id);
		zgb_PrintString("\nInput voltage error!\n");
	}

	if (dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1){
		zgb_TxD_Dec_U8(id);
		zgb_PrintString("\nAngle limit error!\n");
	}

	if (dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1){
		zgb_TxD_Dec_U8(id);
		zgb_PrintString("\nOverheat error!\n");
	}

	if (dxl_get_rxpacket_error(ERRBIT_RANGE) == 1){
		zgb_TxD_Dec_U8(id);
		zgb_PrintString("\nOut of range error!\n");
	}

	if (dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1){
		zgb_TxD_Dec_U8(id);
		zgb_PrintString("\nChecksum error!\n");
	}

	if (dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1){
		zgb_TxD_Dec_U8(id);
		zgb_PrintString("\nOverload error!\n");
	}

	if (dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1){
		zgb_TxD_Dec_U8(id);
		zgb_PrintString("\nInstruction code error!\n");
	}
}
#endif
