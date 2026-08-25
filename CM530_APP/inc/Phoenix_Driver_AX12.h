/*
 * Phoenix_Driver_AX12.h
 *
 *  Created on: 15/02/2016
 *      Author: xottam
 */

#ifndef PHOENIX_DRIVER_AX12_H_
#define PHOENIX_DRIVER_AX12_H_

//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use AX-12 type servos using the
// Arbotix AX12 and bioloid libraries (which may have been updated)
//====================================================================


#include "dynamixel_address_tables.h"
#include "dynamixel.h"
#include "serial.h"
#include "wiring.h"
#include "adc.h"
#include "system_func.h"



#define NUMSERVOSPERLEG 3


#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS)

#define cPwmMult      128
#define cPwmDiv       375  
#define cPFConst      512    // half of our 1024 range

// Some defines for Voltage processing
#define VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE 4000  // Min time in us Until we should do next interpolation, as to not interfer.
#define VOLTAGE_MIN_TIME_BETWEEN_CALLS 150      // Max 6+ times per second
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 1000    // call at least once per second...
#define VOLTAGE_TIME_TO_ERROR          3000    // Error out if no valid item is returned in 3 seconds...

#define USE_BIOLOIDEX            // Use the Bioloid code to control the AX12 servos...
#define USE_AX12_SPEED_CONTROL   // Experiment to see if the speed control works well enough...
//word g_awGoalAXPos[NUMSERVOS];
bool g_fAXSpeedControl;      // flag to know which way we are doing output...
#include "BioloidEx.h"
#include "ServoDriver.h"

#ifdef USE_AX12_SPEED_CONTROL
// Current positions in AX coordinates
word g_awCurAXPos[NUMSERVOS];
word g_awGoalAXPos[NUMSERVOS];
#endif


#define ServosEnabled  (TRUE)      // always true compiler should remove if...


//=============================================================================
// Global - Local to this file only...
//=============================================================================

static const byte cPinTable[] = { cRRCoxaPin, cRMCoxaPin, cRFCoxaPin,
		cLRCoxaPin, cLMCoxaPin, cLFCoxaPin, cRRFemurPin, cRMFemurPin,
		cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin, cRRTibiaPin,
		cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin

};

#define FIRSTCOXAPIN     0
#define FIRSTFEMURPIN    (CNT_LEGS)
#define FIRSTTIBIAPIN    (CNT_LEGS*2)

#define FIRSTTURRETPIN   (CNT_LEGS*3)

// Not sure yet if I will use the controller class or not, but...

bool g_fServosFree;    // Are the servos in a free state?





// Pose is just an array of words...

// A sequence is stored as:
//<header> <sequences><poses>

// Some forward references
extern void MakeSureServosAreOn(void);
extern void DoPyPose(byte *psz);
extern void EEPROMReadData(word wStart, u8 *pv, byte cnt);
extern void EEPROMWriteData(word wStart, u8 *pv, byte cnt);
extern void TCSetServoID(byte *psz);
extern void TCTrackServos();
extern void SetRegOnAllServos(u8 bReg, u8 bVal);

//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void Servo_Init(void) {
	// First lets get the actual servo positions for all of our servos...
	//pinMode(0, OUTPUT);
	g_fServosFree = TRUE;

  poseSize = NUMSERVOS;
  BioloidControllerEx_readPose();

#ifdef cVoltagePin
	for (byte i=0; i < 8; i++)
	GetBatteryVoltage();  // init the voltage pin
#endif

	g_fAXSpeedControl = FALSE;



	// Added - try to speed things up later if we do a query...
	SetRegOnAllServos(AXM_RETURN_DELAY_TIME, 0); // tell servos to give us back their info as quick as they can...

}

//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cVoltagePin
word g_awVoltages[8]= {
	0,0,0,0,0,0,0,0};
word g_wVoltageSum = 0;
byte g_iVoltages = 0;

word GetBatteryVoltage(void) {
	g_iVoltages = (g_iVoltages + 1)&0x7;  // setup index to our array...
	g_wVoltageSum -= g_awVoltages[g_iVoltages];
	g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
	g_wVoltageSum += g_awVoltages[g_iVoltages];

#ifdef CVREF
	return ((long)((long)g_wVoltageSum*CVREF*(CVADR1+CVADR2))/(long)(8192*(long)CVADR2));
#else
	return ((long)((long)g_wVoltageSum*125*(CVADR1+CVADR2))/(long)(2048*(long)CVADR2));
#endif
}

#else
word g_wLastVoltage = 0xffff;    // save the last voltage we retrieved...
byte g_bLegVoltage = 0;		// what leg did we last check?
unsigned long g_ulTimeLastBatteryVoltage;
#define VOLTAGE_REPEAT_MAX  3
word GetBatteryVoltage(void) {
	if (interpolating && (g_wLastVoltage != 0xffff)
			&& ((getMillis_TIM2() - g_ulTimeLastBatteryVoltage)
					< VOLTAGE_MAX_TIME_BETWEEN_CALLS))
		return g_wLastVoltage;

	register u8 bLoopCnt = VOLTAGE_REPEAT_MAX;
	do {
		register word wVoltage = dxl_read_byte(1, AXM_PRESENT_VOLTAGE);
		if (wVoltage != 0xffff) {
			g_ulTimeLastBatteryVoltage = getMillis_TIM2();
			g_wLastVoltage = wVoltage * 10;
			return g_wLastVoltage;
		}
	} while (--bLoopCnt);

	return 0;

}
#endif

//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void BeginServoUpdate(void)    // Start the update
		{

	MakeSureServosAreOn();
	if (ServosEnabled) {

		if (g_fAXSpeedControl) {

#ifdef USE_AX12_SPEED_CONTROL
			// If we are trying our own Servo control need to save away the new positions...
			byte i;
			for (i = 0; i < NUMSERVOS; i++) {
				g_awCurAXPos[i] = g_awGoalAXPos[i];
			}
#endif
		} else
			BioloidControllerEx_interpolateStep(TRUE); // Make sure we call at least once

	}
}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------


void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1,
		short sTibiaAngle1)

		{
	word wCoxaSDV;        // Coxa value in servo driver units
	word wFemurSDV;        //
	word wTibiaSDV;        //

	// The Main code now takes care of the inversion before calling.
	wCoxaSDV = (((long) (sCoxaAngle1)) * cPwmMult) / cPwmDiv + cPFConst;
	wFemurSDV = (((long) ((long) (sFemurAngle1)) * cPwmMult) / cPwmDiv
			+ cPFConst);
	wTibiaSDV = (((long) (sTibiaAngle1)) * cPwmMult) / cPwmDiv + cPFConst;

	if (ServosEnabled) {
		if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
			// Save away the new positions...
			g_awGoalAXPos[FIRSTCOXAPIN + LegIndex] = wCoxaSDV; // What order should we store these values?
			g_awGoalAXPos[FIRSTFEMURPIN + LegIndex] = wFemurSDV;
			g_awGoalAXPos[FIRSTTIBIAPIN + LegIndex] = wTibiaSDV;

#endif
		} else {
			BioloidControllerEx_setNextPose(
					pgm_read_byte(&cPinTable[FIRSTCOXAPIN + LegIndex]),
					wCoxaSDV);
			BioloidControllerEx_setNextPose(
					pgm_read_byte(&cPinTable[FIRSTFEMURPIN + LegIndex]),
					wFemurSDV);
			BioloidControllerEx_setNextPose(
					pgm_read_byte(&cPinTable[FIRSTTIBIAPIN + LegIndex]),
					wTibiaSDV);

		}
	}
	CommanderInputController_AllowControllerInterrupts(TRUE); // Ok for hserial again...
}

//==============================================================================
// Calculate servo speeds to achieve desired pose timing
// We make the following assumptions:
// AX-12 speed is 59rpm @ 12V which corresponds to 0.170s/60deg
// The AX-12 manual states this as the 'no load speed' at 12V
// The Moving Speed control table entry states that 0x3FF = 114rpm
// and according to Robotis this means 0x212 = 59rpm and anything greater 0x212 is also 59rpm
#ifdef USE_AX12_SPEED_CONTROL
word CalculateAX12MoveSpeed(word wCurPos, word wGoalPos, word wTime) {
	word wTravel;
	u32 factor;
	word wSpeed;
	// find the amount of travel for each servo
	if (wGoalPos > wCurPos) {
		wTravel = wGoalPos - wCurPos;
	} else {
		wTravel = wCurPos - wGoalPos;
	}

	// now we can calculate the desired moving speed
	// for 59pm the factor is 847.46 which we round to 848
	// we need to use a temporary 32bit integer to prevent overflow
	factor = (u32) 848 * wTravel;

	wSpeed = (u16)(factor / wTime);
	// if the desired speed exceeds the maximum, we need to adjust
	if (wSpeed > 1023)
		wSpeed = 1023;
	// we also use a minimum speed of 26 (5% of 530 the max value for 59RPM)
	if (wSpeed < 26)
		wSpeed = 26;

	return wSpeed;
}
#endif


//--------------------------------------------------------------------
//[SetRegOnAllServos] Function that is called to set the state of one
//  register in all of the servos, like Torque on...
//--------------------------------------------------------------------
void SetRegOnAllServos(u8 bReg, u8 bVal) {
	// Need to first output the header for the Sync Write
	/*int length = 4 + (NUMSERVOS * 2);   // 2 = id + val
	 int checksum = 254 + length + AX_SYNC_WRITE + 1 + bReg;
	 setTXall();
	 ax12write(0xFF);
	 ax12write(0xFF);
	 ax12write(0xFE);
	 ax12write(length);
	 ax12write(AX_SYNC_WRITE);
	 ax12write(bReg);
	 ax12write(1);    // number of bytes per servo (plus the ID...)
	 for (int i = 0; i < NUMSERVOS; i++) {
	 byte id = pgm_read_byte(&cPinTable[i]);
	 checksum += id + bVal;
	 ax12write(id);
	 ax12write(bVal);

  }
  ax12write(0xff - (checksum % 256));
  setRX(0);*/
	 dxl_set_txpacket_id (BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0,bReg);
	dxl_set_txpacket_parameter(1,1);
  int i;
	for (i = 0; i < poseSize; i++) {
		byte id = pgm_read_byte(&cPinTable[i]);
		dxl_set_txpacket_parameter(2 + 2 * i, id);
		dxl_set_txpacket_parameter(2 + 2 * i + 1, (bVal));
		//dxl_set_txpacket_parameter(2 + 3 * i + 1, dxl_get_lowbyte(bVal));
		//dxl_set_txpacket_parameter(2 + 3 * i + 2, dxl_get_highbyte(bVal));
	}

	dxl_set_txpacket_length((1+1)*poseSize+4);
#ifdef DEBUG_BIOLOIDEX
	PrintString("DriverAX12 --- SetRegOnAllServos ---\n");
#endif
	dxl_txrx_packet();

	u16 CommStatus = dxl_get_result();
#ifdef USING_PC_UART
	if (CommStatus == DXL_RXSUCCESS)
		PrintErrorCode();
	else
		PrintErrorCode();
		PrintCommStatus(CommStatus);
#endif
}
//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly
//        get the next command to start
//--------------------------------------------------------------------
void CommitServoDriver(word wMoveTime) {


	CommanderInputController_AllowControllerInterrupts(FALSE); // If on xbee on hserial tell hserial to not processess...
	if (ServosEnabled) {
		if (g_fAXSpeedControl) {

#ifdef USE_AX12_SPEED_CONTROL
			// Need to first output the header for the Sync Write
			//int length = 4 + (NUMSERVOS * 5); // 5 = id + pos(2byte) + speed(2 bytes)
			/* int checksum = 254 + length + AX_SYNC_WRITE + 4 + AX_GOAL_POSITION_L;
			 word wSpeed;
			 setTXall();
			 ax12write(0xFF);
			 ax12write(0xFF);
			 ax12write(0xFE);
			 ax12write(length);
			 ax12write(AX_SYNC_WRITE);
			 ax12write(AX_GOAL_POSITION_L);
			 ax12write(4);    // number of bytes per servo (plus the ID...)
			 for (int i = 0; i < NUMSERVOS; i++) {
			 wSpeed = CalculateAX12MoveSpeed(g_awCurAXPos[i], g_awGoalAXPos[i], wMoveTime);    // What order should we store these values?
			 byte id = pgm_read_byte(&cPinTable[i]);
			 checksum += id + (g_awGoalAXPos[i]&0xff) + (g_awGoalAXPos[i]>>8) + (wSpeed>>8) + (wSpeed & 0xff);
			 ax12write(id);
			 ax12write(g_awGoalAXPos[i]&0xff);
			 ax12write(g_awGoalAXPos[i]>>8);
			 ax12write(wSpeed&0xff);
			 ax12write(wSpeed>>8);

      }
      ax12write(0xff - (checksum % 256));
      setRX(0);
*/
  	    dxl_set_txpacket_id (BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, AXM_GOAL_POSITION_L);
		dxl_set_txpacket_parameter(1, 4);
      int i;
      word wSpeed;
		for (i = 0; i < poseSize; i++) {
			wSpeed = CalculateAX12MoveSpeed(g_awCurAXPos[i], g_awGoalAXPos[i], wMoveTime);    // What order should we store these values?
			//dxl_set_txpacket_parameter(2 + 3 * i, id_[i]);
			byte id = pgm_read_byte(&cPinTable[i]);
			dxl_set_txpacket_parameter(2 + 5 * i, id);
			dxl_set_txpacket_parameter(2 + 5 * i + 1, dxl_get_lowbyte(g_awGoalAXPos[i]));
			dxl_set_txpacket_parameter(2 + 5 * i + 2, dxl_get_highbyte(g_awGoalAXPos[i]));
			dxl_set_txpacket_parameter(2 + 5 * i + 3, dxl_get_lowbyte(wSpeed));
		    dxl_set_txpacket_parameter(2 + 5 * i + 4, dxl_get_highbyte(wSpeed));
		}

		dxl_set_txpacket_length((4 + 1) * poseSize + 4);
#ifdef DEBUG_BIOLOIDEX
	PrintString("DriverAX12 --- CommitServoDriver ---\n");
#endif
		dxl_txrx_packet();

			u16 CommStatus = dxl_get_result();
#ifdef USING_PC_UART
			if (CommStatus == DXL_RXSUCCESS)
				PrintErrorCode();
			else
				PrintCommStatus(CommStatus);
#endif
#endif
		} else {
			BioloidControllerEx_interpolateSetup(wMoveTime);
		}

	}

	CommanderInputController_AllowControllerInterrupts(TRUE);

}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void FreeServos(void) {

	if (!g_fServosFree) {
		CommanderInputController_AllowControllerInterrupts(FALSE); // If on xbee on hserial tell hserial to not processess...
		SetRegOnAllServos(AXM_TORQUE_ENABLE, 0);  // do this as one statement...
		byte i;
		for (i = 0; i < NUMSERVOS; i++) {
			Relax(pgm_read_byte(&cPinTable[i]));
		}
		CommanderInputController_AllowControllerInterrupts(TRUE);
		g_fServosFree = TRUE;
	}
}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
static u8 g_iIdleServoNum = (u8) - 1;
static u8 g_iIdleLedState = 1;  // what state to we wish to set...
void IdleTime(void) {
	// Each time we call this set servos LED on or off...
	g_iIdleServoNum++;
	if (g_iIdleServoNum >= NUMSERVOS) {
		g_iIdleServoNum = 0;
		g_iIdleLedState = 1 - g_iIdleLedState;
	}
	dxl_write_byte(pgm_read_byte(&cPinTable[g_iIdleServoNum]), AXM_LED,
			g_iIdleLedState);
	//ax12ReadPacket(6);  // get the response...


}

//--------------------------------------------------------------------
//[MakeSureServosAreOn] Function that is called to handle when you are
//  transistioning from servos all off to being on.  May need to read
//  in the current pose...
//--------------------------------------------------------------------
void MakeSureServosAreOn(void) {

	if (ServosEnabled) {

		if (!g_fServosFree)
			return;    // we are not free

		CommanderInputController_AllowControllerInterrupts(FALSE); // If on xbee on hserial tell hserial to not processess...
		if (g_fAXSpeedControl) {

			int i;
			for (i = 0; i < NUMSERVOS; i++) {
				g_awGoalAXPos[i] = dxl_read_word(pgm_read_byte(&cPinTable[i]),
						AXM_PRESENT_POSITION_L);
				mDelay(25);
			}
		} else {
			BioloidControllerEx_readPose();
		}

		SetRegOnAllServos(AXM_TORQUE_ENABLE, 1);  // Use sync write to do it.

		CommanderInputController_AllowControllerInterrupts(TRUE);
		g_fServosFree = FALSE;
	}

}

//==============================================================================
// BackgroundProcess - Allows us to have some background processing for those
//    servo drivers that need us to do things like polling...
//==============================================================================
void BackgroundProcess(void) {
	if (g_fAXSpeedControl)
		return;  // nothing to do in this mode...

	if (ServosEnabled) {
		//DebugToggle(A3);
		BioloidControllerEx_interpolateStep(FALSE); // Do our background stuff...
		Battery_Monitor_Alarm();
	}
}

//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos...
// 		Will use the new servo function to set the actual pwm rate and see
//		how well that works...
//==============================================================================
#ifdef OPT_FIND_SERVO_OFFSETS

void FindServoOffsets()
{

}
#endif  // OPT_FIND_SERVO_OFFSETS

#endif /* PHOENIX_DRIVER_AX12_H_ */
