/*
 * Phoenix_Input_Commander.h
 *
 *  Created on: 20 May 2016
 *      Author: E1193262
 */

#ifndef EASY_FUNCTIONS_INC_PHOENIX_INPUT_COMMANDER_H_
#define EASY_FUNCTIONS_INC_PHOENIX_INPUT_COMMANDER_H_

/*
 * Phoenix_Input_Commander.h
 *
 *  Created on: 13/02/2016
 *      Author: xottam
 */

#ifndef PHOENIX_INPUT_COMMANDER_H_
#define PHOENIX_INPUT_COMMANDER_H_

#include "stm32f10x_type.h"
#include "usart.h"
#include "system_func.h"
#include "mic.h"
#include "zigbee.h"
#include "serial.h"
#include "wiring.h"
#include "Phoenix.h"


typedef enum {FAL = 0, TRU = !FAL} boolean;

//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Arbotix Commander version - Try to emulate most of PS2, but PS2 has 16 buttons and Commander
// has 10. so some things may not be there, others may be doubled up.
//
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
// Quick and Dirty description of controls... WIP
// In most cases I try to mention what button on the PS2 things coorespond to..
// On/OFF - Turning the commander 2 on and off (PS2 start button)
// R1 - options (Change walk gait, Change Leg in Single Leg, Change GP sequence) (Select on PS2)
// R2 - Toggle walk method...  Run Sequence in GP mode
// R3 - Walk method (Not done yet) - (PS2 R3)
// L4 - Ballance mode on and off
// L5 - Stand/Sit (Triangle on PS2)
// L6+Right Joy UP/DOWN - Body up/down - (PS2 Dpad Up/Down)
// L6+Right Joy Left/Right - Speed higher/lower - (PS2 DPad left/right)
// Right Top(S7) - Cycle through options of Normal walk/Double Height/Double Travel) - (PS2 R1, R2)
// Left Top(S8) - Cycle through modes (Walk, Translate, Rotate, Single Leg) (PS2: Circle, X, L1, L2)

//[CONSTANTS]
//[CONSTANTS]

//#define DEBUG_INPUTCOMMANDER

enum {
	WALKMODE = 0, TRANSLATEMODE, ROTATEMODE,SINGLELEGMODE,MODECNT
};
enum {
	NORM_NORM = 0, NORM_LONG, HIGH_NORM, HIGH_LONG
};

#define cTravelDeadZone 6      //The deadzone for the analog input from the remote

#define ARBOTIX_TO  1250        // if we don't get a valid message in this number of mills turn off

//=============================================================================
// I have included a modified version of the commander object here as I wish to
// decouple the usage of the joysticks from the names as well as on other robots
// I may not be using Serial as the the serial port I am communicating with
// So I also removed the Southpaw code.
//=============================================================================
/*
 Commander.h - Library for interfacing with ArbotiX Commander
 Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* bitmasks for buttons array */
#define BUT_R1      0x01
#define BUT_R2      0x02
#define BUT_R3      0x04
#define BUT_L4      0x08
#define BUT_L5      0x10
#define BUT_L6      0x20
#define BUT_RT      0x40
#define BUT_LT      0x80

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

/* the Commander will send out a frame at about 30hz, this class helps decipher the output. */

void Commander(void);
void begin(unsigned long baud);
int Commander_ReadMsgs(); // must be called regularly to clean out Serial buffer
int Commander_ReadMsgs_Odroid(); // must be called regularly to clean out Serial buffer

// joystick values are -125 to 125
signed char rightV;      // vertical stick movement = forward speed
signed char rightH;     // horizontal stick movement = sideways or angular speed
signed char leftV;      // vertical stick movement = tilt
signed char leftH; // horizontal stick movement = pan (when we run out of pan, turn body?)



// buttons are 0 or 1 (PRESSED), and bitmapped
static unsigned char buttons;  //
unsigned char ext;      // Extended function set

// Hooks are used as callbacks for button presses -- NOT IMPLEMENT YET

// internal variables used for reading messages
unsigned char vals[7];  // temporary values, moved after we confirm checksum
//int index;              // -1 = waiting for new packet
int checksum;

//=============================================================================
// Global - Local to this file only...
//=============================================================================
//Commander command = Commander();

unsigned long g_ulLastMsgTime;
short g_sGPSMController; // What GPSM value have we calculated. 0xff - Not used yet
boolean g_fDynamicLegXZLength = FAL; // Has the user dynamically adjusted the Leg XZ init pos (width)

static short g_BodyYOffset;
static short g_BodyYShift;
static byte ControlMode;
static byte HeightSpeedMode;
//static bool  DoubleHeightOn;
static bool DoubleTravelOn;
static byte bJoystickWalkMode;
byte GPSeq;             //Number of the sequence

static byte buttonsPrev;
static byte extPrev;

// some external or forward function references.
extern void CommanderTurnRobotOff(void);

extern void Send_fake_feedback_Odroid(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void CommanderInputController_Init(void) {

	g_BodyYOffset = 0;
	g_BodyYShift = 0;
	//begin(XBEE_BAUD);
	zgb_PrintString("Commander Init: PC_UART at ");
	zgb_Printu32d(Baudrate_PCU);
	zgb_PrintString("\n");
	GPSeq = 0;  // init to something...

	ControlMode = WALKMODE;
	HeightSpeedMode = NORM_NORM;
	//    DoubleHeightOn = false;
	DoubleTravelOn = FAL;
	bJoystickWalkMode = 0;
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void CommanderInputController_AllowControllerInterrupts(bool fAllow) {
	// We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
void CommanderInputController_ControlInput(void) {

#ifdef USING_PC_UART
	// See if we have a new command available...
	if (Commander_ReadMsgs_Odroid() > 0) {
#endif
//#ifdef	USING_ZIGBEE	// See if we have a new command available...
//	if (Commander_ReadMsgs() > 0) {
//#endif
		// If we receive a valid message than turn robot on...
		boolean fAdjustLegPositions = FAL;
		short sLegInitXZAdjust = 0;
		short sLegInitAngleAdjust = 0;

		if (!g_InControlState.fRobotOn) {
			g_InControlState.fRobotOn = TRU;
			fAdjustLegPositions = TRU;
		}

		// [SWITCH MODES]

		// Cycle through modes...
		if ((buttons & BUT_LT) && !(buttonsPrev & BUT_LT)) {

			if (++ControlMode >= MODECNT) {
				ControlMode = WALKMODE;    // cycled back around...
				Buzzed(50, 2000);
				Buzzed(50, 3000);

			} else {
				Buzzed(50, 2000);
			}
			if (ControlMode != SINGLELEGMODE)
				g_InControlState.SelectedLeg = 255;

		}

		//[Common functions]
		//Switch Balance mode on/off
		if ((buttons & BUT_L4) && !(buttonsPrev & BUT_L4)) {

			g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
			if (g_InControlState.BalanceMode) {
				Buzzed(250, 1500);
			} else {
				Buzzed(100, 2000);
				Buzzed(50, 4000);
			}
		}

		//Stand up, sit down
		if ((buttons & BUT_L5) && !(buttonsPrev & BUT_L5)) {
			if (g_BodyYOffset > 0)
				g_BodyYOffset = 0;
			else
				g_BodyYOffset = 35;
			fAdjustLegPositions = TRU;
			g_fDynamicLegXZLength = FAL;
		}

		// We will use L6 with the Right joystick to control both body offset as well as Speed...
		// We move each pass through this by a percentage of how far we are from center in each direction
		// We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
		// which give an idea, but only for those whoes robot has a speaker
		int lx = leftH;
		int ly = leftV;
		if (buttons & BUT_L6) {
			// raise or lower the robot on the joystick up /down
			// Maybe should have Min/Max
			int delta = rightV / 25;
			if (delta) {
				g_BodyYOffset = max(min(g_BodyYOffset + delta, MAX_BODY_Y), 0);
				fAdjustLegPositions = TRU;
			}

			// Also use right Horizontal to manually adjust the initial leg positions.
			sLegInitXZAdjust = lx / 10;        // play with this.
			sLegInitAngleAdjust = ly / 8;
			lx = 0;
			ly = 0;

			// Likewise for Speed control
			delta = rightH / 16;   //
			if ((delta < 0) && g_InControlState.SpeedControl) {
				if ((word)(-delta) < g_InControlState.SpeedControl)
					g_InControlState.SpeedControl += delta;
				else
					g_InControlState.SpeedControl = 0;

				Buzzed(50, 1000 + g_InControlState.SpeedControl);
			}
			if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
				g_InControlState.SpeedControl += delta;
				if (g_InControlState.SpeedControl > 2000)
					g_InControlState.SpeedControl = 2000;

				Buzzed(50, 1000 + g_InControlState.SpeedControl);
			}

			rightH = 0; // don't walk when adjusting the speed here...
		}

		//[Walk functions]
		if (ControlMode == WALKMODE) {

			//Switch gates
			if (((buttons & BUT_R1) && !(buttonsPrev & BUT_R1))
					&& abs(g_InControlState.TravelLength.x) < cTravelDeadZone
					&& abs(g_InControlState.TravelLength.z) < cTravelDeadZone
					&& abs(g_InControlState.TravelLength.y * 2)
							< cTravelDeadZone) {

				g_InControlState.GaitType = g_InControlState.GaitType + 1; // Go to the next gait...
				if (g_InControlState.GaitType < NUM_GAITS) { // Make sure we did not exceed number of gaits...
					Buzzed(50, 2000);
				} else {
					Buzzed(50, 2000);
					Buzzed(50, 2250);
					g_InControlState.GaitType = 0;
				}
				GaitSelect();
			}

			//Double leg lift height
			if ((buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
				Buzzed(50, 2000);
				HeightSpeedMode = (HeightSpeedMode + 1) & 0x3; // wrap around mode
				DoubleTravelOn = HeightSpeedMode & 0x1;
				if (HeightSpeedMode & 0x2)
					g_InControlState.LegLiftHeight = 80;
				else
					g_InControlState.LegLiftHeight = 50;
			}

			// Switch between Walk method 1 && Walk method 2
			if ((buttons & BUT_R2) && !(buttonsPrev & BUT_R2)) {

				if ((++bJoystickWalkMode) > 1)

					bJoystickWalkMode = 0;
				Buzzed(50, 2000 + bJoystickWalkMode * 250);
			}

			//Walking
			switch (bJoystickWalkMode) {
			case 0:
				g_InControlState.TravelLength.x = -lx;
				g_InControlState.TravelLength.z = -ly;
				g_InControlState.TravelLength.y = -(rightH) / 4; //Right Stick Left/Right
				break;
			case 1:
				g_InControlState.TravelLength.z = (rightV); //Right Stick Up/Down
				g_InControlState.TravelLength.y = -(rightH) / 4; //Right Stick Left/Right
				break;

			}

			if (!DoubleTravelOn) {  //(Double travel length)
				g_InControlState.TravelLength.x =
						g_InControlState.TravelLength.x / 2;
				g_InControlState.TravelLength.z =
						g_InControlState.TravelLength.z / 2;
			}

		}

		//[Translate functions]
		g_BodyYShift = 0;
		if (ControlMode == TRANSLATEMODE) {

		      g_InControlState.BodyPos.x =  SmoothControl(((lx)*2/3), g_InControlState.BodyPos.x, 4);//"Smooth division" factor for the smooth control function, a value of 3 to 5 is most suitable
		      g_InControlState.BodyPos.z =  SmoothControl(((ly)*2/3), g_InControlState.BodyPos.z, 4);
		      g_InControlState.BodyRot1.y = SmoothControl(((rightH)*2), g_InControlState.BodyRot1.y, 4);


			//      g_InControlState.BodyPos.x = (lx)/2;
			//      g_InControlState.BodyPos.z = -(ly)/3;
			//      g_InControlState.BodyRot1.y = (rightH)*2;
			g_BodyYShift = (-(rightV) / 2);
		}

		//[Rotate functions]
		if (ControlMode == ROTATEMODE) {
			g_InControlState.BodyRot1.x = (ly);
			g_InControlState.BodyRot1.y = (rightH) * 2;
			g_InControlState.BodyRot1.z = (lx);
			g_BodyYShift = (-(rightV) / 2);
		}

		//[Single leg functions]
#ifdef OPT_SINGLELEG
		if (ControlMode == SINGLELEGMODE) {

			//Switch leg for single leg control
			if ((buttons & BUT_R1) && !(buttonsPrev & BUT_R1)) {
				Buzzed(50,2000);
				if (g_InControlState.SelectedLeg < (CNT_LEGS-1)) {
					g_InControlState.SelectedLeg = g_InControlState.SelectedLeg + 1;
				} else {
					g_InControlState.SelectedLeg = 0;
				}
			}
			/**
			 g_InControlState.SLLeg.x= (signed char)((int)lx+128)/2; //Left Stick Right/Left
			 g_InControlState.SLLeg.y= (signed char)((int)rightV+128)/10; //Right Stick Up/Down
			 g_InControlState.SLLeg.z = (signed char)((int)ly+128)/2; //Left Stick Up/Down
			 */
#if 0
			   g_InControlState.SLLeg.x= (signed char)((int)((int)lx+128)/2); //Left Stick Right/Left
      		   g_InControlState.SLLeg.y= (signed char)((int)((int)rightV+128)/10); //Right Stick Up/Down
		       g_InControlState.SLLeg.z = (signed char)((int)((int)ly+128)/2); //Left Stick Up/Down

#else
			g_InControlState.SLLeg.x = lx; //Left Stick Right/Left
			g_InControlState.SLLeg.y = rightV / 3 - 20;//Right Stick Up/Down
			g_InControlState.SLLeg.z = ly;//Left Stick Up/Down

#endif
			// Hold single leg in place
			if ((buttons & BUT_RT) && !(buttonsPrev & BUT_RT)) {
				Buzzed(50,2000);
				g_InControlState.fSLHold = !g_InControlState.fSLHold;
			}
		}
#endif

		//Calculate walking time delay
		g_InControlState.InputTimeDelay = 128- max(max(abs(lx), abs(ly)), abs(rightH));

		//Calculate g_InControlState.BodyPos.y
		g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift, 0);

		if (sLegInitXZAdjust || sLegInitAngleAdjust) {
			// User asked for manual leg adjustment - only do when we have finished any previous adjustment

			if (!g_InControlState.ForceGaitStepCnt) {
				if (sLegInitXZAdjust)
					g_fDynamicLegXZLength = TRU;

				sLegInitXZAdjust += GetLegsXZLength(); // Add on current length to our adjustment...
				// Handle maybe change angles...
				if (sLegInitAngleAdjust)
					RotateLegInitAngles(sLegInitAngleAdjust);

				// Give system time to process previous calls
				AdjustLegPositions(sLegInitXZAdjust);
			}
		}

		if (fAdjustLegPositions && !g_fDynamicLegXZLength)
			AdjustLegPositionsToBodyHeight(); // Put main workings into main program file
		// Save away the buttons state as to not process the same press twice.
		buttonsPrev = buttons;
		extPrev = ext;
		g_ulLastMsgTime = getMillis_TIM2();
	} else {
		// We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
		if (g_InControlState.fRobotOn) {
			if ((getMillis_TIM2() - g_ulLastMsgTime) > ARBOTIX_TO)
				CommanderTurnRobotOff();
		}
	}

}


void Send_fake_feedback_Odroid(void){

	u8 joint_state[70];
	int pos = 0;
	joint_state[pos++]=0xFF;
	std_putchar(0xFF);
	joint_state[pos++]=0xFF;
	std_putchar(0xFF);
	byte leg = 0;
	for ( leg = 0 ; leg < CNT_LEGS; leg++) {

			joint_state[pos++]=leg;
			pcu_put_byte(leg);

				joint_state[pos++]=1;
				pcu_put_byte(1);

			joint_state[pos++]=dxl_get_lowbyte(1);
			pcu_put_byte(dxl_get_lowbyte(1));
			joint_state[pos++]=dxl_get_highbyte(1);
			pcu_put_byte(dxl_get_highbyte(1));

			joint_state[pos++]=0;
			pcu_put_byte(0);

			joint_state[pos++]=dxl_get_lowbyte(394);
			pcu_put_byte(dxl_get_lowbyte(394));
			joint_state[pos++]=dxl_get_highbyte(394);
			pcu_put_byte(dxl_get_highbyte(394));

			joint_state[pos++]=1;
			pcu_put_byte(1);

			joint_state[pos++]=dxl_get_lowbyte(1);
			pcu_put_byte(dxl_get_lowbyte(1));
			joint_state[pos++]=dxl_get_highbyte(1);
			pcu_put_byte(dxl_get_highbyte(1));
			//OutputServoInfoForLeg(LegIndex,CoxaAngle1[LegIndex],FemurAngle1[LegIndex], TibiaAngle1[LegIndex]);
	}

	byte checksum = checkSumatory(joint_state, pos);
	pcu_put_byte(checksum);
	joint_state[pos++] = checksum;
}
//==============================================================================
// CommanderTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void CommanderTurnRobotOff(void) {

	//Turn off
	g_InControlState.BodyPos.x = 0;
	g_InControlState.BodyPos.y = 0;
	g_InControlState.BodyPos.z = 0;
	g_InControlState.BodyRot1.x = 0;
	g_InControlState.BodyRot1.y = 0;
	g_InControlState.BodyRot1.z = 0;
	g_InControlState.TravelLength.x = 0;
	g_InControlState.TravelLength.z = 0;
	g_InControlState.TravelLength.y = 0;
	g_BodyYOffset = 0;
	g_BodyYShift = 0;
#ifdef OPT_SINGLELEG
	g_InControlState.SelectedLeg = 255;
#endif
	g_InControlState.fRobotOn = 0;

	g_fDynamicLegXZLength = FAL; // also make sure the robot is back in normal leg init mode...
}

//===============================================================================

//==============================================================================
// The below class code is based on the commander class by Michael Ferguson...
// I included and updated for my own usage...  As I may not always use
// Serial and I wish to decouple usage of joysticks from the names...
//==============================================================================

/*
 Commander.cpp - Library for interfacing with ArbotiX Commander
 Copyright (c) 2009-2012 Michael E. Ferguson.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

//==============================================================================
// Commander::Commander - Constructor
//==============================================================================
void Commander_Commander() {
	//index = -1;
}

//==============================================================================
// Commander::begin
//==============================================================================
void Commander_begin(unsigned long baud) {
	//XBeeSerial.begin(baud);
}

//==============================================================================
// ReadMsgs
//==============================================================================

/* process messages coming from Commander
 *  format = 0xFF 0x55 RIGHT_H RIGHT_V LEFT_H LEFT_V BUTTONS EXT CHECKSUM */
int Commander_ReadMsgs_Odroid() {

while (1) {

#ifdef	VIRTUAL_COMANDER
		if (pc_rx_check_odroid_virtualCommander() == 1) {
#endif
#ifndef	VIRTUAL_COMANDER
	  if (pc_rx_check_odroid() == 1) {
#endif
			/*leftV = (signed char) (zgb_rx_data_left_V_());
			leftH = (signed char) (zgb_rx_data_left_H_());
			rightV = (signed char) (zgb_rx_data_right_V_());
			rightH = (signed char) (zgb_rx_data_right_H_());*/
			leftV = (pcu_rx_data_left_V_());
			leftH = (pcu_rx_data_left_H_());
			rightV = (pcu_rx_data_right_V_());
			rightH = (pcu_rx_data_right_H_());
			buttons = pcu_rx_data_buttons();
			ext = pcu_rx_data_extra();
			return 1;
		}else{
			// Mandamos feeback para forzar break en Arbotix.cpp (void  ArbotixPro::_TxRx_CM530(unsigned char *txpacket,unsigned char *rxpacket,int priority))
			Send_fake_feedback_Odroid();
			return 0;
		}
	}
}

//==============================================================================
// ReadMsgs
//==============================================================================

/* process messages coming from Commander
 *  format = 0xFF RIGHT_H RIGHT_V LEFT_H LEFT_V BUTTONS EXT CHECKSUM */
int Commander_ReadMsgs() {

#ifdef	USING_ZIGBEE
	while (1) {
		if (zgb_rx_check_arduino() == 1) {

			/*leftV = (signed char) (zgb_rx_data_left_V_());
			leftH = (signed char) (zgb_rx_data_left_H_());
			rightV = (signed char) (zgb_rx_data_right_V_());
			rightH = (signed char) (zgb_rx_data_right_H_());*/
			leftV = (zgb_rx_data_left_V_());
			leftH = (zgb_rx_data_left_H_());
			rightV = (zgb_rx_data_right_V_());
			rightH = (zgb_rx_data_right_H_());
			buttons = zgb_rx_data_buttons();
			ext = zgb_rx_data_extra();

#ifdef DEBUG_INPUTCOMMANDER
	PrintString("INPUTCOMMANDER --- Commander_ReadMsgs ---\n");
	PrintString("rightV rightH leftV leftH buttons ext\n");
	TxD_Dec_S8(rightV);
	PrintString(" ");
	TxD_Dec_S8(rightH);
	PrintString(" ");
	TxD_Dec_S8(leftV);
	PrintString(" ");
	TxD_Dec_S8(leftH);
	PrintString(" ");
	TxD_Dec_U8(buttons);
	PrintString(" ");
	TxD_Dec_U8(ext);
	PrintString("\n");
#endif
			return 1;
		}

	}
#endif
	return 0;

}
//==============================================================================
//==============================================================================

#endif /* PHOENIX_INPUT_COMMANDER_H_ */



#endif /* EASY_FUNCTIONS_INC_PHOENIX_INPUT_COMMANDER_H_ */
