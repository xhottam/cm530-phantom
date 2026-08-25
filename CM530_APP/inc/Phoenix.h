//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
//
//
// Phoenix.h - This is the first header file that is needed to build
//			a Phoenix program for a specific Hex Robot.
//
//
// This file assumes that the main source file either directly or through include
// file has defined all of the configuration information for the specific robot.
// Each robot will also need to include:
//  
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
//==============================================================================
#ifndef _PHOENIX_CORE_H_
#define _PHOENIX_CORE_H_

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP 	1

#define	c1DEC		10
#define	c2DEC		100
#define	c4DEC		10000
#define	c6DEC		1000000


enum {
	cRR = 0, cRM, cRF, cLR, cLM, cLF, CNT_LEGS
};


#define	WTIMERTICSPERMSMUL  	64	// BAP28 is 16mhz need a multiplyer and divider to make the conversion with /8192
#define WTIMERTICSPERMSDIV  	125 // 
#define USEINT_TIMERAV

// BUGBUG: to make Dynamic first pass simpl make it a variable.
extern byte NUM_GAITS;
#define SmDiv        4  //"Smooth division" factor for the smooth control function, a value of 3 to 5 is most suitable
extern void GaitSelect(void);
extern short SmoothControl(short CtrlMoveInp, short CtrlMoveOut,
		byte CtrlDivider);

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
extern bool g_fDebugOutput;
extern bool g_fEnableServos; // Hack to allow me to turn servo processing off...
extern bool g_fRobotUpsideDown;    // Is the robot upside down?

extern void MSound(byte cNotes, ...);
extern bool CheckVoltage(void);

extern word GetLegsXZLength(void);
extern void AdjustLegPositions(word XZLength1);
extern void AdjustLegPositionsToBodyHeight();
extern void ResetLegInitAngles(void);
extern void RotateLegInitAngles(int iDeltaAngle);
extern long GetCmdLineNum(byte **ppszCmdLine);

// debug handler...
extern bool g_fDBGHandleError;



#ifdef OPT_BACKGROUND_PROCESS
#define DoBackgroundProcess()   BackgroundProcess()
#else
#define DoBackgroundProcess()   
#endif





//=============================================================================
//=============================================================================
// Define the class(s) for our Input controllers.  
//=============================================================================
//=============================================================================
/*class InputController {
 public:
 virtual void     Init(void);
 virtual void     ControlInput(void);
 virtual void     AllowControllerInterrupts(boolean fAllow);

 private:
 } ; */

#include "InputController.h"

typedef struct _Coord3D {
	long x;
	long y;
	long z;
} COORD3D;

//==============================================================================
// Define Gait structure/class - Hopefully allow specific robots to define their
// own gaits and/or define which of the standard ones they want.
//==============================================================================
typedef struct _PhoenixGait {
	short NomGaitSpeed;       //Nominal speed of the gait
	byte StepsInGait;         //Number of steps in gait
	byte NrLiftedPos;    //Number of positions that a single leg is lifted [1-3]
	byte FrontDownPos;        //Where the leg should be put down to ground
	byte LiftDivFactor;       //Normaly: 2, when NrLiftedPos=5: 4
	byte TLDivFactor; //Number of steps that a leg is on the floor while walking
	byte HalfLiftHeight;      // How high to lift at halfway up.

   
	byte GaitLegNr[CNT_LEGS]; //Init position of the leg

} PHOENIXGAIT;


#define GATENAME(name)


//==============================================================================
// class ControlState: This is the main structure of data that the Control 
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//==============================================================================
typedef struct _InControlState {
	bool fRobotOn;            //Switch to turn on Phoenix
	bool fPrev_RobotOn;       //Previous loop state


	bool showPromptTerminal;  //To allow print terminal when the robot never have been in ON.
	//Body position
	COORD3D BodyPos;
	COORD3D BodyRotOffset;      // Body rotation offset;

	//Body Inverse Kinematics
	COORD3D BodyRot1;            // X -Pitch, Y-Rotation, Z-Roll

	//[gait]
	byte GaitType;            //Gait type
	byte GaitStep;            //Actual current step in gait
	PHOENIXGAIT gaitCur;             // Definition of the current gait

	short LegLiftHeight;       //Current Travel height
	COORD3D TravelLength;        // X-Z or Length, Y is rotation.

	//[Single Leg Control]
//#ifdef OPT_SINGLELEG
	byte SelectedLeg;
	byte PrevSelectedLeg;
	COORD3D SLLeg;               //
	bool fSLHold;//Single leg control mode
//#endif

	//[Balance]
	bool BalanceMode;

	//[TIMING]
	byte InputTimeDelay; //Delay that depends on the input to get the "sneaking" effect
	word SpeedControl;   //Adjustible Delay
	byte ForceGaitStepCnt; // new to allow us to force a step even when not moving



	//

} INCONTROLSTATE;

//==============================================================================
//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================
//==============================================================================
/*class ServoDriver {
 public:
 void Init(void);

 word GetBatteryVoltage(void);

 #ifdef OPT_GPPLAYER
 inline boolean  FIsGPEnabled(void) {return _fGPEnabled;};
 boolean         FIsGPSeqDefined(uint8_t iSeq);
 inline boolean  FIsGPSeqActive(void) {return _fGPActive;};
 void            GPStartSeq(uint8_t iSeq);  // 0xff - says to abort...
 void            GPPlayer(void);
 uint8_t         GPNumSteps(void);          // How many steps does the current sequence have
 uint8_t         GPCurStep(void);           // Return which step currently on...
 void            GPSetSpeedMultiplyer(short sm) ;      // Set the Speed multiplier (100 is default)
 #endif
 void BeginServoUpdate(void);    // Start the update
 #ifdef c4DOF
 void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1);
 #else
 void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
 #endif
 void CommitServoDriver(word wMoveTime);
 void FreeServos(void);

 // Allow for background process to happen...
 #ifdef OPT_BACKGROUND_PROCESS
 void BackgroundProcess(void);
 #endif

 #ifdef OPT_TERMINAL_MONITOR
 void ShowTerminalCommandList(void);
 boolean ProcessTerminalCommand(byte *psz, byte bLen);
 #endif

 private:

 #ifdef OPT_GPPLAYER
 boolean _fGPEnabled;     // IS GP defined for this servo driver?
 boolean _fGPActive;      // Is a sequence currently active - May change later when we integrate in sequence timing adjustment code
 uint8_t    _iSeq;        // current sequence we are running
 short    _sGPSM;        // Speed multiplier +-200
 #endif

 } ;   */

//==============================================================================
//==============================================================================
// Define global class objects
//==============================================================================
//==============================================================================
//extern ServoDriver      g_ServoDriver;           // our global servo driver class
//extern InputController  g_InputController;       // Our Input controller
extern INCONTROLSTATE g_InControlState;	// State information that controller changes

#endif

