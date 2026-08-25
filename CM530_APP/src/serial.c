/*
 * serial.c
 *
 *  Created on: 9 Jun 2016
 *      Author: E1193262
 */


#include "serial.h"
#include "usart.h"

#ifdef USING_PC_UART


u8 gbRcvPacketOdroid[PACKET_LENGTH_ODROID];
u8 gbRcvPacketNumOdroid;
u16 gwRcvDataOdroid;
volatile u8 gbRcvFlagOdroid;

u8 right_V_Odroid;
u8 right_H_Odroid;
u8 left_V_Odroid;
u8 left_H_Odroid;
u8 buttons_Odroid;
u8 extra_Odroid;

byte CheckPCNewArrive(void) {
	if (gbPcuRead != gbPcuWrite)
		return 1;
	else
		return 0;
}
byte RxDByte_PC(void) {
	byte bTemp;

	while (1) {
		if (gbPcuRead != gbPcuWrite)
			break;
	}

	bTemp = gbpPcuBuffer[gbPcuRead];
	gbPcuRead++;
	return bTemp;
}
//##############################################################################
u8 pc_rx_check_odroid(void)
{

    u8 RcvNum;
    u8 checksum;
    u8 i, j;

    if (gbRcvFlagOdroid==1){
    	//PrintString("\n NO \n");
    	return 1;
    }

    // Fill packet buffer nueve bytes
    if (gbRcvPacketNumOdroid<9) {
    	// Fill packet array until 9 bytes.
        RcvNum = pcu_hal_rx((u8*)&gbRcvPacketOdroid[gbRcvPacketNumOdroid], (9-gbRcvPacketNumOdroid));
        if (RcvNum!=-1)
            gbRcvPacketNumOdroid += RcvNum;
    }


    // Find header
    if (gbRcvPacketNumOdroid>=2)
    {
        for (i=0; i<gbRcvPacketNumOdroid; i++)
        {
            if (gbRcvPacketOdroid[i]==0xFF)
            {
                if (i<=(gbRcvPacketNumOdroid-2))
                {
                    if (gbRcvPacketOdroid[i+1]==0x55)
                        break;
                }
            }
        }
        if (i>0)
        {
            if (i==gbRcvPacketNumOdroid)
            {
                // Cannot find header
                if (gbRcvPacketOdroid[i-1]==0xFF)

                    i--;
            }
            // Remove data before header
            for (j=i; j<gbRcvPacketNumOdroid; j++)
            {

                gbRcvPacketOdroid[j-i] = gbRcvPacketOdroid[j];
            }
            gbRcvPacketNumOdroid -= i;
        }
    }


    // Verify packet
    if (gbRcvPacketNumOdroid==9)
    {

        if ( (gbRcvPacketOdroid[0]==0xFF) && (gbRcvPacketOdroid[1]==0x55) )
        {
            /**checksum = ~gbRcvPacket[3];
            if (gbRcvPacket[2]==checksum)
            {
                checksum = ~gbRcvPacket[5];
                if (gbRcvPacket[4]==checksum)
                {
                    gwRcvData = (u16) ((gbRcvPacket[4]<<8)&0xFF00);
                    gwRcvData += gbRcvPacket[2];
                    gbRcvFlagOdroid = 1;
                }
            }*/


                  		left_V_Odroid  =   gbRcvPacketOdroid[2];
        	            left_H_Odroid  =   gbRcvPacketOdroid[3];
        	            right_V_Odroid =   gbRcvPacketOdroid[4];
        	            right_H_Odroid =   gbRcvPacketOdroid[5];
        	            buttons_Odroid =   gbRcvPacketOdroid[6];
        	            extra_Odroid   =   gbRcvPacketOdroid[7];
        	            checksum       =   gbRcvPacketOdroid[8];
#ifdef DEBUG_ZIGBEE
	PrintString("ZIGBEE --- pcu_rx_check_arduino ---\n");
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
            if (checksum == (255 - (right_V_Odroid+right_H_Odroid+left_V_Odroid+left_H_Odroid+buttons_Odroid)%256)){        	  //if (checksum == (255 - (gbRcvPacketOdroid[2]+gbRcvPacketOdroid[3]+gbRcvPacketOdroid[4]+gbRcvPacketOdroid[5]+gbRcvPacketOdroid[6])%256)){


				 gbRcvFlagOdroid = 1;
            }

        }
        gbRcvPacketOdroid[0] = 0x00;
        gbRcvPacketNumOdroid = 0;
    }

    return gbRcvFlagOdroid;
}
//##############################################################################
u8 pc_rx_check_odroid_virtualCommander(void)
{

    u8 RcvNum;
    u8 checksum;
    u8 i, j;

    if (gbRcvFlagOdroid==1){
    	//PrintString("\n NO \n");
    	return 1;
    }

    // Fill packet buffer nueve bytes
    if (gbRcvPacketNumOdroid<9) {
    	// Fill packet array until 9 bytes.
        RcvNum = pcu_hal_rx((u8*)&gbRcvPacketOdroid[gbRcvPacketNumOdroid], (8-gbRcvPacketNumOdroid));
        if (RcvNum!=-1)
            gbRcvPacketNumOdroid += RcvNum;
    }


    // Find header
    if (gbRcvPacketNumOdroid>=2)
    {
        for (i=0; i<gbRcvPacketNumOdroid; i++)
        {
            if (gbRcvPacketOdroid[i]==0xff)
            {
                if (i<=(gbRcvPacketNumOdroid-2))
                {
                    if (gbRcvPacketOdroid[i+1]==0x55)
                        break;
                }
            }
        }
        if (i>0)
        {
            if (i==gbRcvPacketNumOdroid)
            {
                // Cannot find header
                if (gbRcvPacketOdroid[i-1]==0xff)

                    i--;
            }
            // Remove data before header
            for (j=i; j<gbRcvPacketNumOdroid; j++)
            {

                gbRcvPacketOdroid[j-i] = gbRcvPacketOdroid[j];
            }
            gbRcvPacketNumOdroid -= i;
        }
    }


    // Verify packet
    if (gbRcvPacketNumOdroid==9)
    {

        if ( (gbRcvPacketOdroid[0]==0xff)  && (gbRcvPacketOdroid[0]==0x55))
        {
            /**checksum = ~gbRcvPacket[3];
            if (gbRcvPacket[2]==checksum)
            {
                checksum = ~gbRcvPacket[5];
                if (gbRcvPacket[4]==checksum)
                {
                    gwRcvData = (u16) ((gbRcvPacket[4]<<8)&0xFF00);
                    gwRcvData += gbRcvPacket[2];
                    gbRcvFlagOdroid = 1;
                }
            }*/


                  		left_V_Odroid  =   gbRcvPacketOdroid[2];
        	            left_H_Odroid  =   gbRcvPacketOdroid[3];
        	            right_V_Odroid =   gbRcvPacketOdroid[4];
        	            right_H_Odroid =   gbRcvPacketOdroid[5];
        	            buttons_Odroid =   gbRcvPacketOdroid[6];
        	            extra_Odroid   =   gbRcvPacketOdroid[7];
        	            checksum       =   gbRcvPacketOdroid[8];
#ifdef DEBUG_ZIGBEE
	PrintString("ZIGBEE --- pcu_rx_check_arduino ---\n");
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
            if (checksum == (255 - (right_V_Odroid+right_H_Odroid+left_V_Odroid+left_H_Odroid+buttons_Odroid)%256)){        	  //if (checksum == (255 - (gbRcvPacketOdroid[2]+gbRcvPacketOdroid[3]+gbRcvPacketOdroid[4]+gbRcvPacketOdroid[5]+gbRcvPacketOdroid[6])%256)){


				 gbRcvFlagOdroid = 1;
            }

        }
        gbRcvPacketOdroid[0] = 0x00;
        gbRcvPacketNumOdroid = 0;
    }

    return gbRcvFlagOdroid;
}
//##############################################################################
u8 pcu_rx_data_buttons(void)
{
    //gbRcvFlag = 0;
    return buttons_Odroid;
}
//##############################################################################
u8 pcu_rx_data_extra(void)
{
	gbRcvFlagOdroid = 0;
    return 0;

}
//##############################################################################
signed char  pcu_rx_data_right_V_(void)
{
    //gbRcvFlag = 0;
    return (right_V_Odroid- 128);
}
//##############################################################################
signed char  pcu_rx_data_right_H_(void)
{
    //gbRcvFlag = 0;
    return (right_H_Odroid - 128);
}
//##############################################################################
signed char  pcu_rx_data_left_V_(void)
{
    //gbRcvFlag = 0;
    return (left_V_Odroid - 128);
}
//##############################################################################
signed char  pcu_rx_data_left_H_(void)
{
    //gbRcvFlag = 0;
    return (left_H_Odroid- 128);
}

//##############################################################################
u8 pcu_initialize(u32 baudrate) {
	if (pcu_hal_open(baudrate) == 0)
		return 0;

	return 1;
}

//##############################################################################
void pcu_terminate(void) {
	pcu_hal_close();
}

//##############################################################################
int std_putchar(char c) {
	if (c == '\n') {
		pcu_put_byte((u8) '\r'); //0x0D
		pcu_put_byte((u8) '\n'); //0x0A
	} else {
		pcu_put_byte((u8) c);
	}

	return c;
}

//##############################################################################
int std_puts(const char *str) {
	int n = 0;
	while (str[n])
		std_putchar(str[n++]);

	return n;
}

//##############################################################################
int std_getchar(void) {
	char c;

	pcu_hal_set_timeout(10);
	while ((pcu_hal_timeout() == 0) && (pcu_get_qstate() == 0))
		;
	if (pcu_get_qstate() == 0)
		return 0xFF;

	c = pcu_get_queue();

	if (c == '\r')
		c = '\n';

	return c;
}

//##############################################################################
char* std_gets(char *str) {
	u8 c, len = 0;

	while (len < 128) {
		pcu_hal_set_timeout(10);
		while ((pcu_hal_timeout() == 0) && (pcu_get_qstate() == 0))
			;
		if (pcu_get_qstate() == 0) {
			if (len == 0) {
				return 0; //NULL;
			} else {
				str[len] = '\0';
				return str;
			}
		}

		c = pcu_get_queue();
		if ((c == '\n') || (c == '\0')) {
			if (len == 0) {
				return 0; //NULL;
			} else {
				str[len] = '\0';
				return str;
			}
		} else
			str[len++] = (s8) c;
	}

	return str;
}

//##############################################################################
void PrintCommStatus(u16 Status) {
	if (Status & DXL_TXFAIL)
		std_puts("\nDXL_TXFAIL: Failed transmit instruction packet!\n");

	if (Status & DXL_RXFAIL)
		std_puts("\nDXL_RXFAIL: Failed get status packet from device!\n");

	if (Status & DXL_TXERROR)
		std_puts("\nDXL_TXERROR: Incorrect instruction packet!\n");

	if (Status & DXL_BAD_INST)
		std_puts("\nDXL_BAD_INST: Invalid Instruction byte\n");

	if (Status & DXL_BAD_ID)
		std_puts(
				"\nDXL_BAD_ID: ID's not same for instruction and status packets\n");

	if (Status & DXL_RXWAITING)
		std_puts("\nDXL_RXWAITING: Now receiving status packet!\n");

	if (Status & DXL_RXTIMEOUT)
		std_puts("\nDXL_RXTIMEOUT: There is no status packet!\n");

	if (Status & DXL_RXCHECKSUM)
		std_puts("\nDXL_RXCHECKSUM: Incorrect status packet checksum!\n");

//    else
//        std_puts("\nThis is unknown error code!\n");
}

//##############################################################################
void PrintErrorCode(void) {
	if (dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		std_puts("\nInput voltage error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		std_puts("\nAngle limit error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		std_puts("\nOverheat error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		std_puts("\nOut of range error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		std_puts("\nChecksum error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		std_puts("\nOverload error!\n");

	if (dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		std_puts("\nInstruction code error!\n");
}

//##############################################################################
int PrintChar(char c) {
	return std_putchar(c);
}

//##############################################################################
int PrintString(const char* s) {
	return std_puts(s);
}

//##############################################################################
int GetChar(void) {
	return std_getchar();
}

//##############################################################################
char* GetString(char* s) {
	return std_gets(s);
}

//##############################################################################
void Printu32d(u32 lNum) {
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

	std_puts(out);
	return;
}

//##############################################################################
void Prints32d(s32 lNumS) {
	u32 temp, lNum, div = 1000000000;
	char out[12];
	u8 i, j;

	if (lNum < 0) {
		out[0] = '-';
		lNum = (u32) ((~lNumS) + 1);
	} else {
		out[0] = '+';
		lNum = (u32) (lNumS);
	}

	for (i = 1; i < 11; i++) {
		temp = (lNum / div);
		lNum = (lNum % div);
//        lNum -= (u32) (temp*div);
//        out[i] = (char) (temp&0x0000000F)+0x30;
		out[i] = (char) ((temp & 0x0F) + 0x30);
		div /= 10;
	}
	out[i] = '\0';

	for (i = 0; i < 11; i++) {
		if (out[0] == '0') {
			for (j = 0; j < 11; j++) {
				out[j] = out[j + 1];
				if (out[j] == '\0')
					break;
			}
		}
	}

	std_puts(out);
	return;
}

//##############################################################################
void Printu16h(u16 wNum) {
	char out[7];
	out[0] = '0';
	out[1] = 'x';
	out[6] = '\0';

	out[2] = (char) ((wNum >> 12) & 0x0F) + 0x30;
	if (out[2] > '9')
		out[2] += 7;

	out[3] = (char) ((wNum >> 8) & 0x0F) + 0x30;
	if (out[3] > '9')
		out[3] += 7;

	out[4] = (char) ((wNum >> 4) & 0x0F) + 0x30;
	if (out[4] > '9')
		out[4] += 7;

	out[5] = (char) (wNum & 0x0F) + 0x30;
	if (out[5] > '9')
		out[5] += 7;

	std_puts(out);
	return;
}

//##############################################################################
void Printu8h(u8 bNum) {
	char out[5];
	out[0] = '0';
	out[1] = 'x';
	out[4] = '\0';

	out[2] = (char) ((bNum >> 4) & 0x0F) + 0x30;
	if (out[2] > '9')
		out[2] += 7;

	out[3] = (char) (bNum & 0x0F) + 0x30;
	if (out[3] > '9')
		out[3] += 7;

	std_puts(out);
	return;
}

//##############################################################################
void TxD_Dec_U8(u8 bByte)
{
    u8 bTmp;
    bTmp = bByte/100;
    /*if(bTmp)*/ pcu_put_byte(bTmp+'0');
    bByte -= bTmp*100;
    bTmp = bByte/10;
    /*if(bTmp)*/ pcu_put_byte( bTmp+'0');
    pcu_put_byte( bByte - bTmp*10+'0');
}

//##############################################################################
void TxD_Dec_U16(u16 wData)
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
            pcu_put_byte( ((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) pcu_put_byte( ((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) pcu_put_byte( ' ');
                else pcu_put_byte( '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}

//##############################################################################
void TxD_Dec_U32(u32 wData)
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
            pcu_put_byte( ((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) pcu_put_byte( ((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) pcu_put_byte( ' ');
                else pcu_put_byte( '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}

//##############################################################################
void TxD_Dec_S8(s8 wData)
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
            if (bMinus) pcu_put_byte( '-');
            pcu_put_byte( ((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) pcu_put_byte( ((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) pcu_put_byte( ' ');
                else pcu_put_byte(  '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}
//##############################################################################
void TxD_Dec_S16(s16 wData)
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
            if (bMinus) pcu_put_byte( '-');
            pcu_put_byte( ((u8)wTmp)+'0');
            bPrinted = 1;
        }
        else
        {
            if(bPrinted) pcu_put_byte( ((u8)wTmp)+'0');
            else
            {
                if(bCount < 4) pcu_put_byte( ' ');
                else pcu_put_byte(  '0');
            }
        }
        wData -= wTmp*wDigit;
        wDigit /= 10;
    }
}

//##############################################################################
void TxD_Dec_S32(s32 lLong)
{
    u8 bCount, bPrinted;
    s32 lTmp,lDigit;
    bPrinted = 0;
    if(lLong < 0)
    {
        lLong = -lLong;
        pcu_put_byte(  '-');
    }
    lDigit = 1000000000L;
    for(bCount = 0; bCount < 9; bCount++)
    {
        lTmp = (u8)(lLong/lDigit);
        if(lTmp)
        {
            pcu_put_byte( ((u8)lTmp)+'0');
            bPrinted = 1;
        }
        else if(bPrinted) pcu_put_byte( ((u8)lTmp)+'0');
        lLong -= ((u32)lTmp)*lDigit;
        lDigit = lDigit/10;
    }
    lTmp = (u8)(lLong/lDigit);
    /*if(lTmp)*/ pcu_put_byte(  ((u8)lTmp)+'0');
}
#endif
