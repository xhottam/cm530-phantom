

#define HEXMODE
//#define MILLIS
//#define SENSOR

//#define DEBUG_BIOLOIDEX
#define M_PI           3.14159265358979323846

#include "stm32f10x_lib.h"
#include "wiring.h"
#include "system_init.h"
#include "Hex_Cfg.h"
#include "Phoenix.h"
#include "adc.h"
#include "zigbee.h"


INCONTROLSTATE g_InControlState; // This is our global Input control state object...



//const byte g_abHexIntXZ[]  = {cHexInitXZ, 144};
//const byte g_abHexMaxBodyY[] = { 20, MAX_BODY_Y};

//const byte g_abHexIntXZ[]  = {cHexInitXZ, 134};
//const byte g_abHexMaxBodyY[]  = { 20, MAX_BODY_Y};

#include "Phoenix_Input_Commander.h"
#include "Phoenix_Driver_AX12.h"

//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino
//
// Phoenix_Code.h
//
//     This contains the main code for the Phoenix project.  It is included in
//     all of the different configurations of the phoenix code.
//
//NEW IN V2.X
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#define BalanceDivFactor CNT_LEGS    //;Other values than 6 can be used, testing...CAUTION!! At your own risk ;)

//#include <Wire.h>
//#include <I2CEEProm.h>

//--------------------------------------------------------------------
//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1.
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//-    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//-    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//-    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total bytes: 277

static const byte GetACos[] = { 255, 254, 252, 251, 250, 249, 247, 246, 245,
		243, 242, 241, 240, 238, 237, 236, 234, 233, 232, 231, 229, 228, 227,
		225, 224, 223, 221, 220, 219, 217, 216, 215, 214, 212, 211, 210, 208,
		207, 206, 204, 203, 201, 200, 199, 197, 196, 195, 193, 192, 190, 189,
		188, 186, 185, 183, 182, 181, 179, 178, 176, 175, 173, 172, 170, 169,
		167, 166, 164, 163, 161, 160, 158, 157, 155, 154, 152, 150, 149, 147,
		146, 144, 142, 141, 139, 137, 135, 134, 132, 130, 128, 127, 125, 123,
		121, 119, 117, 115, 113, 111, 109, 107, 105, 103, 101, 98, 96, 94, 92,
		89, 87, 84, 81, 79, 76, 73, 73, 73, 72, 72, 72, 71, 71, 71, 70, 70, 70,
		70, 69, 69, 69, 68, 68, 68, 67, 67, 67, 66, 66, 66, 65, 65, 65, 64, 64,
		64, 63, 63, 63, 62, 62, 62, 61, 61, 61, 60, 60, 59, 59, 59, 58, 58, 58,
		57, 57, 57, 56, 56, 55, 55, 55, 54, 54, 53, 53, 53, 52, 52, 51, 51, 51,
		50, 50, 49, 49, 48, 48, 47, 47, 47, 46, 46, 45, 45, 44, 44, 43, 43, 42,
		42, 41, 41, 40, 40, 39, 39, 38, 37, 37, 36, 36, 35, 34, 34, 33, 33, 32,
		31, 31, 30, 29, 28, 28, 27, 26, 25, 24, 23, 23, 23, 23, 22, 22, 22, 22,
		21, 21, 21, 21, 20, 20, 20, 19, 19, 19, 19, 18, 18, 18, 17, 17, 17, 17,
		16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9,
		9, 8, 7, 6, 6, 5, 3, 0 };      //

//Sin table 90 deg, persision 0.5 deg [180 values]
static const word GetSin[] = { 0, 87, 174, 261, 348, 436, 523, 610, 697, 784,
		871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564, 1650, 1736, 1822,
		1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840,
		2923, 3007, 3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826,
		3907, 3987, 4067, 4146, 4226, 4305, 4383, 4461, 4539, 4617, 4694, 4771,
		4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664,
		5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494,
		6560, 6626, 6691, 6755, 6819, 6883, 6946, 7009, 7071, 7132, 7193, 7253,
		7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826, 7880, 7933,
		7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526,
		8571, 8616, 8660, 8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025,
		9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 9335, 9366, 9396, 9426,
		9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723,
		9743, 9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914,
		9925, 9935, 9945, 9953, 9961, 9969, 9975, 9981, 9986, 9990, 9993, 9996,
		9998, 9999, 10000 };      //

//Build tables for Leg configuration like I/O and MIN/ Max values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

// BUGBUG: Need a cleaner way to define...
// Lets allow for which legs servos to be inverted to be defined by the robot
// This is used by the Lynxmotion Symetrical Quad.
#ifndef cRRCoxaInv
#define cRRCoxaInv 1
#endif
#ifndef cRMCoxaInv
#define cRMCoxaInv 1
#endif
#ifndef cRFCoxaInv
#define cRFCoxaInv 1
#endif

#ifndef cLRCoxaInv
#define cLRCoxaInv 0
#endif
#ifndef cLMCoxaInv
#define cLMCoxaInv 0
#endif
#ifndef cLFCoxaInv
#define cLFCoxaInv 0
#endif

#ifndef cRRFemurInv
#define cRRFemurInv 1
#endif
#ifndef cRMFemurInv
#define cRMFemurInv 1
#endif
#ifndef cRFFemurInv
#define cRFFemurInv 1
#endif

#ifndef cLRFemurInv
#define cLRFemurInv 0
#endif
#ifndef cLMFemurInv
#define cLMFemurInv 0
#endif
#ifndef cLFFemurInv
#define cLFFemurInv 0
#endif

#ifndef cRRTibiaInv
#define cRRTibiaInv 1
#endif
#ifndef cRMTibiaInv
#define cRMTibiaInv 1
#endif
#ifndef cRFTibiaInv
#define cRFTibiaInv 1
#endif

#ifndef cLRTibiaInv
#define cLRTibiaInv 0
#endif
#ifndef cLMTibiaInv
#define cLMTibiaInv 0
#endif
#ifndef cLFTibiaInv
#define cLFTibiaInv 0
#endif

#ifndef cRRTarsInv
#define cRRTarsInv 1
#endif
#ifndef cRMTarsInv
#define cRMTarsInv 1
#endif
#ifndef cRFTarsInv
#define cRFTarsInv 1
#endif

#ifndef cLRTarsInv
#define cLRTarsInv 0
#endif
#ifndef cLMTarsInv
#define cLMTarsInv 0
#endif
#ifndef cLFTarsInv
#define cLFTarsInv 0
#endif

// Also define default BalanceDelay
#ifndef BALANCE_DELAY
#define BALANCE_DELAY 100
#endif

#ifdef HEXMODE
// Standard Hexapod...
// Servo Horn offsets

#define CFEMURHORNOFFSET1(LEGI)  (cFemurHornOffset1)
#define CTIBIAHORNOFFSET1(LEGI)  (cTibiaHornOffset1)

#endif

//Min / Max values
#ifndef SERVOS_DO_MINMAX
const short cCoxaMin1[] = {
cRRCoxaMin1, cRMCoxaMin1, cRFCoxaMin1, cLRCoxaMin1, cLMCoxaMin1, cLFCoxaMin1 };
const short cCoxaMax1[] = {
cRRCoxaMax1, cRMCoxaMax1, cRFCoxaMax1, cLRCoxaMax1, cLMCoxaMax1, cLFCoxaMax1 };
const short cFemurMin1[] = {
cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1,
		cLFFemurMin1 };
const short cFemurMax1[] = {
cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1,
		cLFFemurMax1 };
const short cTibiaMin1[] = {
cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1,
		cLFTibiaMin1 };
const short cTibiaMax1[] = {
cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1,
		cLFTibiaMax1 };

#endif

// Servo inverse direction
const bool cCoxaInv[] = { cRRCoxaInv, cRMCoxaInv, cRFCoxaInv, cLRCoxaInv,
		cLMCoxaInv, cLFCoxaInv };
bool cFemurInv[] = { cRRFemurInv, cRMFemurInv, cRFFemurInv, cLRFemurInv,
		cLMFemurInv, cLFFemurInv };
const bool cTibiaInv[] = { cRRTibiaInv, cRMTibiaInv, cRFTibiaInv, cLRTibiaInv,
		cLMTibiaInv, cLFTibiaInv };

//Leg Lengths
const byte cCoxaLength[] = {
cRRCoxaLength, cRMCoxaLength, cRFCoxaLength, cLRCoxaLength, cLMCoxaLength,
		cLFCoxaLength };
const byte cFemurLength[] = {
cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength,
		cLFFemurLength };
const byte cTibiaLength[] = {
cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength,
		cLFTibiaLength };

//Body Offsets [distance between the center of the body and the center of the coxa]
const short cOffsetX[] = {
cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX };
const short cOffsetZ[] = {
cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ };

//Default leg angle
const short cCoxaAngle1[] = {
cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1,
		cLFCoxaAngle1 };

//Start positions for the leg
const short cInitPosX[] = {
cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX };
const short cInitPosY[] = {
cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY };
const short cInitPosZ[] = {
cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ };

// Define some globals for debug information
bool g_fShowDebugPrompt;
bool g_fDebugOutput;
//boolean g_fEnableServos = true;
bool g_fEnableServos = TRUE;

//--------------------------------------------------------------------
//[ANGLES]
short CoxaAngle1[CNT_LEGS];   //Actual Angle of the horizontal hip, decimals = 1
short FemurAngle1[CNT_LEGS];   //Actual Angle of the vertical hip, decimals = 1
short TibiaAngle1[CNT_LEGS];   //Actual Angle of the knee, decimals = 1

short CoxaAngle_Odroid[CNT_LEGS];   //Actual Angle of the horizontal hip, decimals = 1
short FemurAngle_Odroid[CNT_LEGS];   //Actual Angle of the vertical hip, decimals = 1
short TibiaAngle_Odroid[CNT_LEGS];   //Actual Angle of the knee, decimals = 1

//[RADIANS]
short CoxaAngle1_rad[CNT_LEGS];   //Actual Angle of the horizontal hip, decimals = 1
short FemurAngle1_rad[CNT_LEGS];   //Actual Angle of the vertical hip, decimals = 1
short TibiaAngle1_rad[CNT_LEGS];   //Actual Angle of the knee, decimals = 1

//--------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]

short LegPosX[CNT_LEGS];    //Actual X Posion of the Leg
short LegPosY[CNT_LEGS];    //Actual Y Posion of the Leg
short LegPosZ[CNT_LEGS];    //Actual Z Posion of the Leg
//--------------------------------------------------------------------
//[VARIABLES]
byte Index;                    //Index universal used
byte LegIndex;                //Index used for leg Index Number

//GetSinCos / ArcCos
short AngleDeg1;        //Input Angle in degrees, decimals = 1
short sin4;             //Output Sinus of the given Angle, decimals = 4
short cos4;            //Output Cosinus of the given Angle, decimals = 4
short AngleRad4;        //Output Angle in radials, decimals = 4

//GetAtan2
short AtanX;            //Input X
short AtanY;            //Input Y
short Atan4;            //ArcTan2 output
long XYhyp2;            //Output presenting Hypotenuse of X and Y

//Body Inverse Kinematics
short PosX;            //Input position of the feet X
short PosZ;            //Input position of the feet Z
short PosY;            //Input position of the feet Y
long BodyFKPosX;        //Output Position X of feet with Rotation
long BodyFKPosY;        //Output Position Y of feet with Rotation
long BodyFKPosZ;        //Output Position Z of feet with Rotation

//Leg Inverse Kinematics
long IKFeetPosX;        //Input position of the Feet X
long IKFeetPosY;        //Input position of the Feet Y
long IKFeetPosZ;        //Input Position of the Feet Z
bool IKSolution;        //Output true if the solution is possible
bool IKSolutionWarning;    //Output true if the solution is NEARLY possible
bool IKSolutionError;    //Output true if the solution is NOT possible
//--------------------------------------------------------------------
//[TIMING]
unsigned long lTimerStart;    //Start time of the calculation cycles
unsigned long lTimerEnd;        //End time of the calculation cycles
byte CycleTime;        //Total Cycle time

word ServoMoveTime;        //Time for servo updates
word PrevServoMoveTime;    //Previous time for the servo updates

//--------------------------------------------------------------------
//[GLOABAL]
//--------------------------------------------------------------------

bool g_fLowVoltageShutdown; // If set the bot shuts down because the input voltage is to low
word Voltage;

//--boolean         g_InControlState.fRobotOn;            //Switch to turn on Phoenix
//--boolean         g_InControlState.fPrev_RobotOn;        //Previous loop state
//--------------------------------------------------------------------
//[Balance]
long TotalTransX;
long TotalTransZ;
long TotalTransY;
long TotalYBal1;
long TotalXBal1;
long TotalZBal1;
//[Single Leg Control]
bool AllDown;

//[gait - State]
// Note: Information about the current gait is now part of the g_InControlState...
bool TravelRequest;          //Temp to check if the gait is in motion

long GaitPosX[CNT_LEGS]; //Array containing Relative X position corresponding to the Gait
long GaitPosY[CNT_LEGS]; //Array containing Relative Y position corresponding to the Gait
long GaitPosZ[CNT_LEGS]; //Array containing Relative Z position corresponding to the Gait
long GaitRotY[CNT_LEGS]; //Array containing Relative Y rotation corresponding to the Gait

//boolean           GaitLegInAir[CNT_LEGS];     // True if leg is in the air
//byte          GaitNextLeg;                // The next leg which will be lifted

bool fWalking;            //  True if the robot are walking
byte bExtraCycle; // Forcing some extra timed cycles for avoiding "end of gait bug"
#define         cGPlimit 2           // GP=GaitPos testing different limits

bool g_fRobotUpsideDown;    // Is the robot upside down?
bool fRobotUpsideDownPrev;
//=============================================================================
// Define our default standard Gaits
//=============================================================================
#ifndef DEFAULT_GAIT_SPEED
#define DEFAULT_GAIT_SPEED 25
#define DEFAULT_SLOW_GAIT 50
#endif

//cRR=0, cRF, cLR, cLF, CNT_LEGS};

#ifndef OVERWRITE_GAITS
#ifdef HEXMODE
// Hex Gaits
//  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground,
//     Quad extra: COGAngleStart, COGAngleStep, CogRadius, COGCCW
//                      { RR, <RM> RF, LR, <LM>, LF}
#ifdef DISPLAY_GAIT_NAMES
extern "C" {
  // Move the Gait Names to program space...
  const char s_szGN1[]  = "Ripple 12";
  const char s_szGN2[]  = "Tripod 8";
  const char s_szGN3[]  = "Tripple 12";
  const char s_szGN4[]  = "Tripple 16";
  const char s_szGN5[]  = "Wave 24";
  const char s_szGN6[]  = "Tripod 6";
};
#endif

PHOENIXGAIT APG[] = {
    {DEFAULT_SLOW_GAIT, 12, 3, 2, 2, 8, 3, {7, 11, 3, 1, 5, 9} GATENAME(s_szGN1)},        // Ripple 12
    {DEFAULT_SLOW_GAIT, 8, 3, 2, 2, 4, 3, {1, 5, 1, 5, 1, 5} GATENAME(s_szGN2)},           //Tripod 8 steps
    {DEFAULT_GAIT_SPEED, 12, 3, 2, 2, 8, 3, {5, 10, 3, 11, 4, 9} GATENAME(s_szGN3) },      //Triple Tripod 12 step
    {DEFAULT_GAIT_SPEED, 16, 5, 3, 4, 10, 1, {6, 13, 4, 14, 5, 12} GATENAME(s_szGN4)},    // Triple Tripod 16 steps, use 5 lifted positions
    {DEFAULT_SLOW_GAIT, 24, 3, 2, 2, 20, 3, {13, 17, 21, 1, 5, 9} GATENAME(s_szGN5)},     //Wave 24 steps
    {DEFAULT_GAIT_SPEED, 6, 2, 1, 2, 4, 1, {1, 4, 1, 4, 1, 4} GATENAME(s_szGN6)}          //Tripod 6 steps
};

#elif defined(OCTOMODE)
// Octopod Gaits

//  Speed, Steps, Lifted, Front Down, Lifted Factor, Half Height, On Ground,
//                      { RR, RMR, RMF, RF, LR, LMR, LMF, LF}
#ifdef DISPLAY_GAIT_NAMES
extern "C" {
  // Move the Gait Names to program space...
  const char s_szGN1[]  = "Tripod 6";
};
#endif

PHOENIXGAIT APG[] = {
    {DEFAULT_GAIT_SPEED, 6, 2, 1, 2, 4, 1, {1, 4, 1, 4, 4, 1, 4, 1} GATENAME(s_szGN1)}          //Tripod 6 steps
};

#else
// Quad Gaits
#ifdef DISPLAY_GAIT_NAMES
extern "C" {
  // Move the Gait Names to program space...
  const char s_szGN1[]  = "Ripple 12";
  const char s_szGN2[]  = "Tripod 8";
}
#endif
PHOENIXGAIT APG[] = {
    {DEFAULT_GAIT_SPEED, 16, 3, 2, 2, 12, 3, 2250, 3600/16, 30, true, {5, 9, 1, 13} GATENAME(s_szGN1)},            // Wave 16
    {1, 28, 3, 2, 2, 24, 3, 2250, 3600/28, 30, true, {8, 15, 1, 22} GATENAME(s_szGN2)}                             // Wave 28?
};

#endif
#endif
//--------------------------------------------------------------------

#ifdef ADD_GAITS
byte NUM_GAITS = sizeof(APG)/sizeof(APG[0]) + sizeof(APG_EXTRA)/sizeof(APG_EXTRA[0]);
#else
byte NUM_GAITS = sizeof(APG)/sizeof(APG[0]);
#endif

//=============================================================================
// Function prototypes
//=============================================================================
extern void GaitSelect(void);
extern void WriteOutputs(void);
extern void SingleLegControl(void);
extern void GaitSeq(void);
extern void BalanceBody(void);
extern void CheckAngles();

//extern void  GaitGetNextLeg(byte GaitStep);
extern void BalCalcOneLeg(long PosX, long PosZ, long PosY, byte BalLegNr);
extern void BodyFK(short PosX, short PosZ, short PosY, short RotationY,
		byte BodyIKLeg);
extern void LegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ,
		byte LegIKLegNr);
extern void Gait(byte GaitCurrentLegNr);
extern void GetSinCos(short AngleDeg1);
extern short GetATan2(short AtanX, short AtanY);
extern unsigned long isqrt32(unsigned long n);

extern void StartUpdateServos(void);

extern void setupPhoenix(void);
extern void readSensors(void);


int main(void) {

	// Must include this line in all programs using "cm530.h/.c"
	SysInit();
	setupPhoenix();
	//BioloidControllerEx();

	while (1){
		while (1) {
			//Start time
			unsigned long lTimeWaitEnd;
			lTimerStart = getMillis_TIM2();
#ifdef MILLIS
			PrintString("Barebones lTimerStart ");
			Printu32d(lTimerStart);
			PrintString("\n");
#endif
#ifdef SENSOR
			TxD_Dec_U16(ReadIR(EPORT1A));
			PrintString(" ");
			mDelay(2);
			TxD_Dec_U16(ReadIR(EPORT2A));
			PrintString(" ");
			mDelay(2);
			TxD_Dec_U16(ReadAnalog(EPORT3A));
		    PrintString(" ");
			mDelay(2);
			TxD_Dec_U16(ReadAnalog(EPORT4A));
		    PrintString(" ");
			mDelay(2);
			TxD_Dec_U16(ReadAnalog(EPORT5A));
		    PrintString("\n");
#endif
			DoBackgroundProcess();
			//Read input
			CheckVoltage();        // check our voltages...
			if (!g_fLowVoltageShutdown) {
				CommanderInputController_ControlInput();
			}
			WriteOutputs();        // Write Outputs
			//Single leg control
			SingleLegControl();
			DoBackgroundProcess();

			//Gait
			GaitSeq();

			DoBackgroundProcess();

			//Balance calculations
			TotalTransX = 0;     //reset values used for calculation of balance
			TotalTransZ = 0;
			TotalTransY = 0;
			TotalXBal1 = 0;
			TotalYBal1 = 0;
			TotalZBal1 = 0;

			if (g_InControlState.BalanceMode) {

				for (LegIndex = 0; LegIndex < (CNT_LEGS / 2); LegIndex++) { // balance calculations for all Right legs

					DoBackgroundProcess();
					BalCalcOneLeg(-LegPosX[LegIndex] + GaitPosX[LegIndex],
							LegPosZ[LegIndex] + GaitPosZ[LegIndex],
							(LegPosY[LegIndex]
									 - (short) pgm_read_word(&cInitPosY[LegIndex]))
									 + GaitPosY[LegIndex], LegIndex);
				}

				for (LegIndex = (CNT_LEGS / 2); LegIndex < CNT_LEGS; LegIndex++) { // balance calculations for all Right legs
					DoBackgroundProcess();
					BalCalcOneLeg(LegPosX[LegIndex] + GaitPosX[LegIndex],
							LegPosZ[LegIndex] + GaitPosZ[LegIndex],
							(LegPosY[LegIndex]
									 - (short) pgm_read_word(&cInitPosY[LegIndex]))
									 + GaitPosY[LegIndex], LegIndex);
				}
				BalanceBody();
			}

			//Reset IKsolution indicators
			IKSolution = 0;
			IKSolutionWarning = 0;
			IKSolutionError = 0;

			//Do IK for all Right legs

		for (LegIndex = 0; LegIndex < (CNT_LEGS / 2); LegIndex++) {
				DoBackgroundProcess();
				BodyFK(
						-LegPosX[LegIndex] + g_InControlState.BodyPos.x
						+ GaitPosX[LegIndex] - TotalTransX,
						LegPosZ[LegIndex] + g_InControlState.BodyPos.z
						+ GaitPosZ[LegIndex] - TotalTransZ,
						LegPosY[LegIndex] + g_InControlState.BodyPos.y
						+ GaitPosY[LegIndex] - TotalTransY,
						GaitRotY[LegIndex], LegIndex);


				LegIK(
						LegPosX[LegIndex] - g_InControlState.BodyPos.x + BodyFKPosX
						- (GaitPosX[LegIndex] - TotalTransX),
						LegPosY[LegIndex] + g_InControlState.BodyPos.y - BodyFKPosY
						+ GaitPosY[LegIndex] - TotalTransY,
						LegPosZ[LegIndex] + g_InControlState.BodyPos.z - BodyFKPosZ
						+ GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
			}

			//Do IK for all Left legs
			for (LegIndex = (CNT_LEGS / 2); LegIndex < CNT_LEGS; LegIndex++) {
				DoBackgroundProcess();
				BodyFK(
						LegPosX[LegIndex] - g_InControlState.BodyPos.x
						+ GaitPosX[LegIndex] - TotalTransX,
						LegPosZ[LegIndex] + g_InControlState.BodyPos.z
						+ GaitPosZ[LegIndex] - TotalTransZ,
						LegPosY[LegIndex] + g_InControlState.BodyPos.y
						+ GaitPosY[LegIndex] - TotalTransY,
						GaitRotY[LegIndex], LegIndex);

				LegIK(
						LegPosX[LegIndex] + g_InControlState.BodyPos.x - BodyFKPosX
						+ GaitPosX[LegIndex] - TotalTransX,
						LegPosY[LegIndex] + g_InControlState.BodyPos.y - BodyFKPosY
						+ GaitPosY[LegIndex] - TotalTransY,
						LegPosZ[LegIndex] + g_InControlState.BodyPos.z - BodyFKPosZ
						+ GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
			}



			//Check mechanical limits
			CheckAngles();

			//Drive Servos
			if (g_InControlState.fRobotOn) {
				if (g_InControlState.fRobotOn && !g_InControlState.fPrev_RobotOn) {
					// MSound(3, 60, 2000, 80, 2250, 100, 2500);
					Buzzed(30, 2000);
					Buzzed(80, 2250);
					Buzzed(100, 2500);

				}

				//Calculate Servo Move time
				if ((abs(g_InControlState.TravelLength.x) > cTravelDeadZone)
						|| (abs(g_InControlState.TravelLength.z) > cTravelDeadZone)
						|| (abs(g_InControlState.TravelLength.y*2) > cTravelDeadZone)) {
					ServoMoveTime = g_InControlState.gaitCur.NomGaitSpeed
							+ (g_InControlState.InputTimeDelay * 2)
							+ g_InControlState.SpeedControl;

					//Add aditional delay when Balance mode is on
					if (g_InControlState.BalanceMode)
						ServoMoveTime = ServoMoveTime + BALANCE_DELAY;
				} else
					//Movement speed excl. Walking
					ServoMoveTime = 200 + g_InControlState.SpeedControl;

				// note we broke up the servo driver into start/commit that way we can output all of the servo information
				// before we wait and only have the termination information to output after the wait.  That way we hopefully
				// be more accurate with our timings...
				DoBackgroundProcess();
				StartUpdateServos();

				// See if we need to sync our processor with the servo driver while walking to ensure the prev is completed
				//before sending the next one

				// Finding any incident of GaitPos/Rot <>0:
				for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
					if ((GaitPosX[LegIndex] > cGPlimit)
							|| (GaitPosX[LegIndex] < -cGPlimit)
							|| (GaitPosZ[LegIndex] > cGPlimit)
							|| (GaitPosZ[LegIndex] < -cGPlimit)
							|| (GaitRotY[LegIndex] > cGPlimit)
							|| (GaitRotY[LegIndex] < -cGPlimit)) {

						bExtraCycle = g_InControlState.gaitCur.NrLiftedPos + 1;	//For making sure that we are using timed move until all legs are down
						break;
					}
				}
				if (bExtraCycle > 0) {
					bExtraCycle--;
					fWalking = !(bExtraCycle == 0);

					//Get endtime and calculate wait time
					lTimeWaitEnd = lTimerStart + PrevServoMoveTime;
#ifdef MILLIS
					PrintString("Barebones lTimeWaitEnd ");
					Printu32d(lTimeWaitEnd);
					PrintString("\n");
#endif
					//DebugWrite(A1, HIGH);
					do {
						// Wait the appropriate time, call any background process while waiting...
						DoBackgroundProcess();
					} while (getMillis_TIM2() < lTimeWaitEnd);
					//DebugWrite(A1, LOW);
#ifdef MILLIS
					PrintString("Barebones lTimeWaitEnd-Millis  ");
					Printu32d(getMillis_TIM2);
					PrintString("\n");
#endif

				}
				// Only do commit if we are actually doing something...
				CommitServoDriver(ServoMoveTime);
			} else {
				//Turn the bot off - May need to add ajust here...
				if (g_InControlState.fPrev_RobotOn || (AllDown == 0)) {
					ServoMoveTime = 600;
					StartUpdateServos();
					CommitServoDriver(ServoMoveTime);
					//MSound(3, 100, 2500, 80, 2250, 60, 2000);
					Buzzed(100, 2500);
					Buzzed(80, 2250);
					Buzzed(60, 2000);

					//lTimeWaitEnd = getMillis_TIM2() + 600; // setup to process background stuff while we wait...
					lTimeWaitEnd = getMillis_TIM2() + 600; // setup to process background stuff while we wait...
#ifdef MILLIS
					PrintString("Barebones lTimeWaitEnd ");
					Printu32d(lTimeWaitEnd);
					PrintString("\n");
#endif
					do {
						// Wait the appropriate time, call any background process while waiting...
						DoBackgroundProcess();
					} while (getMillis_TIM2() < lTimeWaitEnd);
					//delay(600);
#ifdef MILLIS
					PrintString("Barebones lTimeWaitEnd-Millis  ");
					Printu32d(getMillis_TIM2);
					PrintString("\n");
#endif
				} else {
					//FreeServos();
				}

				// Allow the Servo driver to do stuff durint our idle time
				IdleTime();
				TerminalMonitor();
				mDelay(20); // give a pause between times we call if nothing is happening
			}
			PrevServoMoveTime = ServoMoveTime;

			//Store previous g_InControlState.fRobotOn State
			if (g_InControlState.fRobotOn)
				g_InControlState.fPrev_RobotOn = 1;
			else
				g_InControlState.fPrev_RobotOn = 0;
		}

	}

	return 0;
}

/*************************************************
begin RAM
	_sdata
					global variables
	_enddata
					heap
	_sbrk::heap_end
						.
						.
						.
						.
						.
	sp register
					stack
	_estack
End RAM
*************************************************/


/*##########  INT MEM  #########
Estimated global data size:     009124
Starting HEAP info: start-> 536880036  current-> 536880036
Start STACK info: end-> 536932352  current-> 536936200



##########  MEM USAGE  #########
HEAP ptr -> 536880036  Usage->    000000
STACK Max->120  Usage->4294963592
Estimated UNUSED MEM->    056020*/

register u8 * stack_ptr asm("sp");
extern u8 end asm("_end");


#ifndef NULL
#define NULL   ((void *) 0)
#endif



void *_sbrk(int incr){
	//static	unsigned char *heap = NULL;
	static  u8 *heap_end = NULL;
	static  u8 *prev_heap_end;

	if (heap_end == NULL){
		//heap = (unsigned char *)&_end;
		heap_end = &end;
	}
	prev_heap_end = heap_end;

	if (heap_end + incr > stack_ptr){
		return (void *)-1;
	}

	heap_end +=incr;

	return (void *)prev_heap_end;

}


u32 g_end_stack_pointer;
u32 g_start_heap_pointer;

void initMemoryUsageTest()
{
  // Guess on start of stack. // probably using less than 100 bytes of stack space...
  g_end_stack_pointer = ((u32)stack_ptr + 100) & 0x2000FFFF;


  // get the start of the heap ponter
  g_start_heap_pointer = (u32)&end;

  // Print out some memory information
  zgb_PrintString("Estimated global data size: ");
  zgb_TxD_Dec_U32(g_start_heap_pointer & 0xffff);
  zgb_PrintString("\n");
  zgb_PrintString("Starting HEAP info: start->");
  zgb_TxD_Dec_U32(g_start_heap_pointer);
  zgb_PrintString("  current(_end)->");
  zgb_TxD_Dec_U32((u32)_sbrk(0));
  zgb_PrintString("\n");
  zgb_PrintString("Start STACK info: endRam(start stack)->");
  zgb_TxD_Dec_U32(g_end_stack_pointer);
  zgb_PrintString("  current(sp)->");
  zgb_TxD_Dec_U32((u32)stack_ptr);
  zgb_PrintString("\n");

  zgb_PrintString("Try to init memory\n");
  u8 *sp_minus = stack_ptr - 10;  // leave a little slop
  u8 *p = (u8*)_sbrk(0);
  for (;p < sp_minus; p++) *p = 0xff; // init to ff
  zgb_PrintString("After init memory\n\n");

}


void printMemoryUsage()
{
  u8 *current_heap_ptr = (u8*)_sbrk(0);
  zgb_PrintString("HEAP ptr ->");
  zgb_TxD_Dec_U32((u32)current_heap_ptr);
  zgb_PrintString("  Usage->");
  zgb_TxD_Dec_U32((u32)current_heap_ptr - g_start_heap_pointer);
  zgb_PrintString("\n");

  // stack info
  u8 *sp_minus = stack_ptr - 10;  // leave a little slop
  u8 *p = current_heap_ptr;

    // try to find out how far the stack has been used
  while ((p < sp_minus) && (*p == 0xff))  p++;
  zgb_PrintString("STACK ptr -> ");
  zgb_Printu32d((u32)p);
  zgb_PrintString("  Usage->");
  zgb_TxD_Dec_U32(g_end_stack_pointer - (u32)p);
  zgb_PrintString("\n");
  zgb_PrintString("Estimated UNUSED MEM(Stack(down)+Heap(up))->");
  zgb_TxD_Dec_U32(p - current_heap_ptr);
  zgb_PrintString("\n\n");

}



//--------------------------------------------------------------------
// UpdateGaitCmd
//--------------------------------------------------------------------
#ifdef OPT_DYNAMIC_ADJUST_LEGS
void UpdateInitialPosAndAngCmd(byte *pszCmdLine) {
  // If no other parameters, show current state
  if (1) {  // Need to get past the command letter first...
	zgb_PrintString("Len: ");
	zgb_TxD_Dec_U16(GetLegsXZLength());
	zgb_PrintString(" Angs: ");
    for(int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
    	if (LegIndex==0 || LegIndex==3 ){
    		zgb_TxD_Dec_S16(g_InControlState.aCoxaInitAngle1[LegIndex]);
    	}else{
    		zgb_TxD_Dec_U16(g_InControlState.aCoxaInitAngle1[LegIndex]);
    	}

    	zgb_PrintString(" ");
    }
    zgb_PrintString("\n");
  }
  else {
	// Get the new leg positions
    word wNewLegsXZPos = 0;//GetCmdLineNum(&pszCmdLine);
    if (*pszCmdLine) {
      int  iDeltaAngle = 0;//GetCmdLineNum(&pszCmdLine);
      RotateLegInitAngles(iDeltaAngle);
    }
    AdjustLegPositions(wNewLegsXZPos);

  }
}
#endif

char *gaits_Name[]  = {"Ripple 12","Tripod 8","Tripple 12","Tripple 16","Wave 24","Tripple 6"};

void PrintGaitsTable(void){
	PHOENIXGAIT gait;
	u8 id =0;

	for(;id<sizeof(APG)/sizeof(APG[0]);id++){
		gait = APG[id];
		zgb_PrintString(gaits_Name[id]);
		zgb_PrintString("-->");
		zgb_PrintString("NomGaitSpeed:");
		zgb_TxD_Dec_U8(gait.NomGaitSpeed);
		zgb_PrintString("|");
		zgb_PrintString("StepsInGait:");
		zgb_TxD_Dec_U8(gait.StepsInGait);
		zgb_PrintString("|");
		zgb_PrintString("NrLiftedPos:");
		zgb_TxD_Dec_U8(gait.NrLiftedPos);
		zgb_PrintString("|");
		zgb_PrintString("FrontDownPos:");
		zgb_TxD_Dec_U8(gait.FrontDownPos);
		zgb_PrintString("|");
		zgb_PrintString("LiftDivFactor:");
		zgb_TxD_Dec_U8(gait.LiftDivFactor);
		zgb_PrintString("|");
		zgb_PrintString("TLDivFactor:");
		zgb_TxD_Dec_U8(gait.TLDivFactor);
		zgb_PrintString("|");
		zgb_PrintString("HalfLiftHeight:");
		zgb_TxD_Dec_U8(gait.HalfLiftHeight);
		zgb_PrintString("\n");
	}
}

void TerminalMonitor(void)
{
	  //byte szCmdLine[20];  // currently pretty simple command lines...
	  byte ich;
	  // See if we need to output a prompt.
	  if (g_InControlState.showPromptTerminal) {
		  zgb_PrintString("############################################################\n");
		  mDelay(100);
		  zgb_PrintString("M - PRINT Menu\n");
		  mDelay(100);
		  zgb_PrintString("P - PRINT Memory Usage\n");
		  mDelay(100);
		  zgb_PrintString("C - PRINT AX-12A Control Table (run as: C#12#(e|r|a)#)\n");
		  mDelay(100);
		  zgb_PrintString("A - PRINT AX-12A Address Table\n");
		  mDelay(100);
		  zgb_PrintString("G - PRINT GAITs Table\n");
		  mDelay(100);
		  zgb_PrintString("R - RESET PHANTOMX\n");
		  mDelay(100);
#ifdef OPT_DYNAMIC_ADJUST_LEGS
		  zgb_PrintString("L - DYNAMIC_ADJUST_LEGS pos ang\n");
#endif
		  zgb_PrintString("############################################################\n\n");
		  mDelay(100);

	#ifdef QUADMODE
	//	DBGSerial.println("B <percent>");
	    DBGSerial.println("G ST NL RR RF LR LF");
	#endif
	#ifdef OPT_TERMINAL_MONITOR_IC    // Allow the input controller to define stuff as well
	    g_InputController.ShowTerminalCommandList();
	#endif

	    // Let the Servo driver show it's own set of commands...
	    g_InControlState.showPromptTerminal = 0;
	  }

	 if ( zgb_hal_rx((u8*)&gbRcvPacket[0], 1) == 1)
		 {
		    ich = gbRcvPacket[0];

			if (ich == 'r' || ich == 'R') {
				//VECTRESET (= core reset) NVIC_GenerateCoreReset resets only the Cortex core, without any other on-chip devices (like UART etc.).
				//SYSRESETREQ (= system reset)NVIC_GenerateSystemReset is an outgoing signal from the core to the other on-chip-logic.
				NVIC_GenerateSystemReset();
				//NVIC_GenerateCoreReset();
			}

			if (ich == 'm' || ich == 'M') {
				g_InControlState.showPromptTerminal = 1;
			}
			if (ich == 'g' || ich == 'G') {
				zgb_PrintString("\n########## GaitsTable ##################\n");
				PrintGaitsTable();
			}
#ifdef OPT_DYNAMIC_ADJUST_LEGS
			if (ich == 'l' || ich == 'L') {
				zgb_PrintString("\n########## DYNAMIC_ADJUST_LEGS ##################\n");
				UpdateInitialPosAndAngCmd(szCmdLine);
			}
#endif
			if (ich == 'p' || ich == 'P') {
				printMemoryUsage();
			}
			if (ich == 'a' || ich == 'A') {
				zgb_PrintString("X - RW from AX-12A address table\n");
				mDelay(100);
				zgb_PrintString("?: X#id#R(read)|W(write)#1(byte)|2(word)#operation#value# | X#12#R#1#I# | X#12#W#1#I#23#" );
				mDelay(100);
				zgb_PrintString("\n########## AX-12a AddressTable ##################\n");
				mDelay(100);
				BioloidControllerEx_AX_AddressTable();
			}
			if (ich == 'x' || ich == 'X') {
				int idrw = 0;
				bool units = FALSE;
				zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
				if (gbRcvPacket[0] == '#'){
					while (1){
						zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
						if (gbRcvPacket[0] != '#'){
							if (!units){
								idrw = idrw +(int) gbRcvPacket[0] - 48;
								units = TRUE;
							}else{
								idrw =  (idrw * 10) + (int) gbRcvPacket[0] - 48;
								//OJO
								zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
								break;
							}
						}else{
							break;
						}
					}
				}
				if (gbRcvPacket[0] == '#'){
					zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
					byte rw = gbRcvPacket[0];
					zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
					if (gbRcvPacket[0] == '#'){
						zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
						byte size = gbRcvPacket[0];
						zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
						if (gbRcvPacket[0] == '#'){
							zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
							byte address = gbRcvPacket[0];
							zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
							if (gbRcvPacket[0] == '#'){
								zgb_PrintString("\n########## RW ##################\n");
								BioloidControllerEx_AX_RW(idrw,rw,size,address);
							}
						}
					}

				}
			}
			if (ich == 'c' || ich == 'C') {
				u8 idc = 0;
				bool units = FALSE;
				zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
				if (gbRcvPacket[0] == '#'){
					while (1){
						zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
						if (gbRcvPacket[0] != '#'){
							if (!units){
								idc = idc +(u8) gbRcvPacket[0] - 48;
								units = TRUE;
							}else{
								idc =  (idc * 10) + (u8) gbRcvPacket[0] - 48;
								//OJO
								zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
								break;
							}
						}else{
							break;
						}
					}
					if (gbRcvPacket[0] == '#'){
						zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
						byte section = gbRcvPacket[0];
						zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
						if (gbRcvPacket[0] == '#'){
							zgb_PrintString("\n########## ControlTable ##################\n");
							BioloidControllerEx_AX_ReadControlTable(idc,section);
						}

					}
				}
			}
		}
}
void setupPhoenix() {

	while (getMillis_TIM2() < 6000);

	//if (!g_InControlState.from_TerminalMonitor){
		zgb_PrintString("##################################\n");
		zgb_PrintString("#        Program Start           #\n");
		zgb_PrintString("##################################\n\n");
	g_fShowDebugPrompt = TRUE;
	g_fDebugOutput = FALSE;
	//init BioloidEx
		zgb_PrintString("##########  INT MEM  #########\n");
		initMemoryUsageTest();
	BioloidControllerEx();

	// Init our ServoDriver
	Servo_Init();

	mDelay(10);

	// Setup Init Positions
	for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
		LegPosX[LegIndex] = (short) pgm_read_word(&cInitPosX[LegIndex]); //Set start positions for each leg
		LegPosY[LegIndex] = (short) pgm_read_word(&cInitPosY[LegIndex]);
		LegPosZ[LegIndex] = (short) pgm_read_word(&cInitPosZ[LegIndex]);
	}

	ResetLegInitAngles();

	//Single leg control. Make sure no leg is selected
#ifdef OPT_SINGLELEG
	g_InControlState.SelectedLeg = 255; // No Leg selected
	g_InControlState.PrevSelectedLeg = 255;
#endif
	//Body Positions
	g_InControlState.BodyPos.x = 0;
	g_InControlState.BodyPos.y = 0;
	g_InControlState.BodyPos.z = 0;

//Body Rotations
	g_InControlState.BodyRot1.x = 0;
	g_InControlState.BodyRot1.y = 0;
	g_InControlState.BodyRot1.z = 0;
	g_InControlState.BodyRotOffset.x = 0;
	g_InControlState.BodyRotOffset.y = 0; //Input Y offset value to adjust centerpoint of rotation
	g_InControlState.BodyRotOffset.z = 0;

	//Gait
	g_InControlState.GaitType = 0;
	g_InControlState.BalanceMode = 0;
	g_InControlState.LegLiftHeight = 50;
	g_InControlState.ForceGaitStepCnt = 0; // added to try to adjust starting positions depending on height...
	g_InControlState.GaitStep = 1;
	GaitSelect();

		zgb_PrintString("##########  GAIT  #########\n");
		PrintGaitsTable();
		zgb_PrintString("\n");
	CommanderInputController_Init();
	zgb_PrintString("\n");

	// Servo Driver
	ServoMoveTime = 150;
	g_InControlState.fRobotOn = 0;
	g_fLowVoltageShutdown = FALSE;
		zgb_PrintString("##########  MEM USAGE  #########\n");
		printMemoryUsage();
	//}

}

void StartUpdateServos() {
	byte LegIndex;

	// First call off to the init...
	BeginServoUpdate();    // Start the update

	u8 joint_state[70];
	int pos = 0;
    int signo = 0;
	joint_state[pos++]=0xFF;
	std_putchar(0xFF);
	joint_state[pos++]=0xFF;
	std_putchar(0xFF);

	for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {


		joint_state[pos++]=LegIndex;
		pcu_put_byte(LegIndex);

		if (CoxaAngle1[LegIndex] < 0){
			joint_state[pos++]=0;
			pcu_put_byte(0);
			signo = -1;
		} else {
			joint_state[pos++]=1;
			pcu_put_byte(1);
			signo = 1;
		}
		joint_state[pos++]=dxl_get_lowbyte(signo * (word)CoxaAngle1[LegIndex]);
		pcu_put_byte(dxl_get_lowbyte(signo * (word)CoxaAngle1[LegIndex]));
		joint_state[pos++]=dxl_get_highbyte(signo * (word)CoxaAngle1[LegIndex]);
		pcu_put_byte(dxl_get_highbyte(signo * (word)CoxaAngle1[LegIndex]));

		if (FemurAngle1[LegIndex] < 0){
			joint_state[pos++]=0;
			pcu_put_byte(0);
			signo = -1;
		} else {
			joint_state[pos++]=1;
			pcu_put_byte(1);
			signo = 1;
		}
		joint_state[pos++]=dxl_get_lowbyte(signo * (word)FemurAngle1[LegIndex]);
		pcu_put_byte(dxl_get_lowbyte(signo * (word)FemurAngle1[LegIndex]));
		joint_state[pos++]=dxl_get_highbyte(signo * (word)FemurAngle1[LegIndex]);
		pcu_put_byte(dxl_get_highbyte(signo * (word)FemurAngle1[LegIndex]));

		if (TibiaAngle1[LegIndex] < 0){
			joint_state[pos++]=0;
			pcu_put_byte(0);
			signo = -1;
		} else {
			joint_state[pos++]=1;
			pcu_put_byte(1);
			signo = 1;
		}
		joint_state[pos++]=dxl_get_lowbyte(signo * (word)TibiaAngle1[LegIndex]);
		pcu_put_byte(dxl_get_lowbyte( signo * (word)TibiaAngle1[LegIndex]));
		joint_state[pos++]=dxl_get_highbyte( signo *(word)TibiaAngle1[LegIndex]);
		pcu_put_byte(dxl_get_highbyte( signo * (word)TibiaAngle1[LegIndex]));


		OutputServoInfoForLeg(LegIndex,
				cCoxaInv[LegIndex] ?
						-CoxaAngle1[LegIndex] : CoxaAngle1[LegIndex],
				cFemurInv[LegIndex] ?
						-FemurAngle1[LegIndex] : FemurAngle1[LegIndex],
				cTibiaInv[LegIndex] ?
						-TibiaAngle1[LegIndex] : TibiaAngle1[LegIndex]);

		//OutputServoInfoForLeg(LegIndex,CoxaAngle1[LegIndex],FemurAngle1[LegIndex], TibiaAngle1[LegIndex]);
	}

	byte checksum = checkSumatory(joint_state, pos);
	pcu_put_byte(checksum);
	joint_state[pos++]= checksum;
	//mDelay(pos);

}

//--------------------------------------------------------------------
//[WriteOutputs] Updates the state of the leds
//--------------------------------------------------------------------
void WriteOutputs(void) {

}
//--------------------------------------------------------------------
//[CHECK VOLTAGE]
//Reads the input voltage and shuts down the bot when the power drops
byte s_bLVBeepCnt;
bool CheckVoltage() {
#ifdef cTurnOffVol
	// Moved to Servo Driver - BUGBUG: Need to do when I merge back...
	//    Voltage = analogRead(cVoltagePin); // Battery voltage
	//    Voltage = ((long)Voltage*1955)/1000;
	Voltage = ReadAnalog(VBUS) >> 4;

	// BUGBUG:: if voltage is 0 it failed to retrieve don't hang program...
	//    if (!Voltage)
	//      return;

	if (!g_fLowVoltageShutdown) {
		/**PrintString("Voltage : ");
		Printu32d(Voltage);
		PrintString("\n");*/
		if ((Voltage < cTurnOffVol) || (Voltage >= 1999)) {


			//Turn off
			g_InControlState.BodyPos.x = 0;
			g_InControlState.BodyPos.y = 0;
			g_InControlState.BodyPos.z = 0;
			g_InControlState.BodyRot1.x = 0;
			g_InControlState.BodyRot1.y = 0;
			g_InControlState.BodyRot1.z = 0;
			g_InControlState.TravelLength.x = 0;
			g_InControlState.TravelLength.z = 0;

#ifdef OPT_SINGLELEG
			g_InControlState.TravelLength.y = 0;
			g_InControlState.SelectedLeg = 255;
#endif
			g_fLowVoltageShutdown = 1;
			s_bLVBeepCnt = 0;    // how many times we beeped...
			g_InControlState.fRobotOn = FALSE;
		}
#ifdef cTurnOnVol
	} else if ((Voltage > cTurnOnVol) && (Voltage < 1999)) {

		g_fLowVoltageShutdown = 0;

#endif
	} else {
		if (s_bLVBeepCnt < 5) {
			s_bLVBeepCnt++;

			//MSound( 1, 45, 2000);
			Buzzed(45, 2000);
		}
		mDelay(2000);
	}
#endif

	return g_fLowVoltageShutdown;
}

//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
void SingleLegControl(void) {
#ifdef OPT_SINGLELEG



	//Check if all legs are down
	AllDown = (LegPosY[cRF] == (short) pgm_read_word(&cInitPosY[cRF]))
			&& (LegPosY[cRR] == (short) pgm_read_word(&cInitPosY[cRR]))
			&& (LegPosY[cLR] == (short) pgm_read_word(&cInitPosY[cLR])) &&
#ifdef HEXMODE
			(LegPosY[cRM] == (short) pgm_read_word(&cInitPosY[cRM]))
			&& (LegPosY[cLM] == (short) pgm_read_word(&cInitPosY[cLM])) &&

#endif
			(LegPosY[cLF] == (short) pgm_read_word(&cInitPosY[cLF]));


	if (g_InControlState.SelectedLeg <= (CNT_LEGS - 1)) {
		if (g_InControlState.SelectedLeg != g_InControlState.PrevSelectedLeg) {
			if (AllDown) { //Lift leg a bit when it got selected
				LegPosY[g_InControlState.SelectedLeg] = (short) pgm_read_word(
						&cInitPosY[g_InControlState.SelectedLeg]) - 20;

				//Store current status
				g_InControlState.PrevSelectedLeg = g_InControlState.SelectedLeg;
			} else { //Return prev leg back to the init position
				LegPosX[g_InControlState.PrevSelectedLeg] =
						(short) pgm_read_word(
								&cInitPosX[g_InControlState.PrevSelectedLeg]);
				LegPosY[g_InControlState.PrevSelectedLeg] =
						(short) pgm_read_word(
								&cInitPosY[g_InControlState.PrevSelectedLeg]);
				LegPosZ[g_InControlState.PrevSelectedLeg] =
						(short) pgm_read_word(
								&cInitPosZ[g_InControlState.PrevSelectedLeg]);
			}
		} else if (!g_InControlState.fSLHold) {
			//LegPosY[g_InControlState.SelectedLeg] = LegPosY[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.y;
			LegPosY[g_InControlState.SelectedLeg] = (short) pgm_read_word(
					&cInitPosY[g_InControlState.SelectedLeg])
					+ g_InControlState.SLLeg.y; // Using DIY remote Zenta prefer it this way
			LegPosX[g_InControlState.SelectedLeg] = (short) pgm_read_word(
					&cInitPosX[g_InControlState.SelectedLeg])
					+ g_InControlState.SLLeg.x;
			LegPosZ[g_InControlState.SelectedLeg] = (short) pgm_read_word(
					&cInitPosZ[g_InControlState.SelectedLeg])
					+ g_InControlState.SLLeg.z;
		}
	} else { //All legs to init position
		if (!AllDown) {
			for (LegIndex = 0; LegIndex <= (CNT_LEGS - 1); LegIndex++) {
				LegPosX[LegIndex] = (short) pgm_read_word(&cInitPosX[LegIndex]);
				LegPosY[LegIndex] = (short) pgm_read_word(&cInitPosY[LegIndex]);
				LegPosZ[LegIndex] = (short) pgm_read_word(&cInitPosZ[LegIndex]);
			}
		}
		if (g_InControlState.PrevSelectedLeg != 255)
			g_InControlState.PrevSelectedLeg = 255;
	}
#endif
}

void GaitSelect(void) {
	//Gait selector
	// First pass simply use defined table, next up will allow robots to add or relace set...
	if (g_InControlState.GaitType < NUM_GAITS) {

		g_InControlState.gaitCur = APG[g_InControlState.GaitType];

	}

}

//--------------------------------------------------------------------
//[GAIT Sequence]
void GaitSeq(void) {
	//Check if the Gait is in motion - If not if we are going to start a motion try to align our Gaitstep to start with a good foot
	// for the direction we are about to go...

	if (fWalking || (g_InControlState.ForceGaitStepCnt != 0))
		TravelRequest = TRUE;   // Is walking or was walking...
	else {
		TravelRequest = (abs(g_InControlState.TravelLength.x) > cTravelDeadZone)
				|| (abs(g_InControlState.TravelLength.z) > cTravelDeadZone)
				|| (abs(g_InControlState.TravelLength.y) > cTravelDeadZone);

		if (TravelRequest) {

		} else {    //Clear values under the cTravelDeadZone
			g_InControlState.TravelLength.x = 0;
			g_InControlState.TravelLength.z = 0;
			g_InControlState.TravelLength.y = 0; //Gait NOT in motion, return to home position
		}
	}

	//Calculate Gait sequence
	for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) { // for all legs
		Gait(LegIndex);
	}    // next leg

		 //Advance to the next step
	g_InControlState.GaitStep++;
	if (g_InControlState.GaitStep > g_InControlState.gaitCur.StepsInGait)
		g_InControlState.GaitStep = 1;

	// If we have a force count decrement it now...
	if (g_InControlState.ForceGaitStepCnt)
		g_InControlState.ForceGaitStepCnt--;
}

//--------------------------------------------------------------------
//[GAIT]
void Gait(byte GaitCurrentLegNr) {

	// Try to reduce the number of time we look at GaitLegnr and Gaitstep
	short int LegStep = g_InControlState.GaitStep
			- g_InControlState.gaitCur.GaitLegNr[GaitCurrentLegNr];

	//Leg middle up position OK
	//Gait in motion
	// For Lifted pos = 1, 3, 5
	if ((TravelRequest && (g_InControlState.gaitCur.NrLiftedPos & 1)
			&& LegStep == 0)
			|| (!TravelRequest && LegStep == 0
					&& ((abs(GaitPosX[GaitCurrentLegNr]) > 2)
							|| (abs(GaitPosZ[GaitCurrentLegNr]) > 2)
							|| (abs(GaitRotY[GaitCurrentLegNr]) > 2)))) { //Up
		GaitPosX[GaitCurrentLegNr] = 0;
		GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight;
		GaitPosZ[GaitCurrentLegNr] = 0;
		GaitRotY[GaitCurrentLegNr] = 0;
	}
	//Optional Half heigth Rear (2, 3, 5 lifted positions)
	else if (((g_InControlState.gaitCur.NrLiftedPos == 2 && LegStep == 0)
			|| (g_InControlState.gaitCur.NrLiftedPos >= 3
					&& (LegStep == -1
							|| LegStep
									== (g_InControlState.gaitCur.StepsInGait - 1))))
			&& TravelRequest) {
		GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x
				/ g_InControlState.gaitCur.LiftDivFactor;
		GaitPosY[GaitCurrentLegNr] = -3 * g_InControlState.LegLiftHeight
				/ (3 + g_InControlState.gaitCur.HalfLiftHeight); //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
		GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z
				/ g_InControlState.gaitCur.LiftDivFactor;
		GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y
				/ g_InControlState.gaitCur.LiftDivFactor;
	}
	// _A_
	// Optional Half heigth front (2, 3, 5 lifted positions)
	else if ((g_InControlState.gaitCur.NrLiftedPos >= 2)
			&& (LegStep == 1
					|| LegStep == -(g_InControlState.gaitCur.StepsInGait - 1))
			&& TravelRequest) {
		GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x
				/ g_InControlState.gaitCur.LiftDivFactor;
		GaitPosY[GaitCurrentLegNr] = -3 * g_InControlState.LegLiftHeight
				/ (3 + g_InControlState.gaitCur.HalfLiftHeight); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
		GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z
				/ g_InControlState.gaitCur.LiftDivFactor;
		GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y
				/ g_InControlState.gaitCur.LiftDivFactor;
	}

	//Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
	else if (((g_InControlState.gaitCur.NrLiftedPos == 5 && (LegStep == -2)))
			&& TravelRequest) {
		GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x / 2;
		GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight / 2;
		GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z / 2;
		GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y / 2;
	}

	//Optional Half heigth Front 5 LiftedPos (5 lifted positions)
	else if ((g_InControlState.gaitCur.NrLiftedPos == 5)
			&& (LegStep == 2
					|| LegStep == -(g_InControlState.gaitCur.StepsInGait - 2))
			&& TravelRequest) {
		GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x / 2;
		GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight / 2;
		GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z / 2;
		GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y / 2;
	}
	//_B_
	//Leg front down position //bug here?  From _A_ to _B_ there should only be one gaitstep, not 2!
	//For example, where is the case of LegStep==0+2 executed when NRLiftedPos=3?
	else if ((LegStep == g_InControlState.gaitCur.FrontDownPos
			|| LegStep
					== -(g_InControlState.gaitCur.StepsInGait
							- g_InControlState.gaitCur.FrontDownPos))
			&& GaitPosY[GaitCurrentLegNr] < 0) {
		GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x / 2;
		GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z / 2;
		GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y / 2;
		GaitPosY[GaitCurrentLegNr] = 0;
	}

	//Move body forward
	else {
		GaitPosX[GaitCurrentLegNr] = GaitPosX[GaitCurrentLegNr]
				- (g_InControlState.TravelLength.x
						/ (short) g_InControlState.gaitCur.TLDivFactor);
		GaitPosY[GaitCurrentLegNr] = 0;
		GaitPosZ[GaitCurrentLegNr] = GaitPosZ[GaitCurrentLegNr]
				- (g_InControlState.TravelLength.z
						/ (short) g_InControlState.gaitCur.TLDivFactor);
		GaitRotY[GaitCurrentLegNr] = GaitRotY[GaitCurrentLegNr]
				- (g_InControlState.TravelLength.y
						/ (short) g_InControlState.gaitCur.TLDivFactor);
	}

}

//--------------------------------------------------------------------
//[BalCalcOneLeg]
void BalCalcOneLeg(long PosX, long PosZ, long PosY, byte BalLegNr) {
	long CPR_X;            //Final X value for centerpoint of rotation
	long CPR_Y;            //Final Y value for centerpoint of rotation
	long CPR_Z;            //Final Z value for centerpoint of rotation

	long lAtan;
	//Calculating totals from center of the body to the feet
	CPR_Z = (short) pgm_read_word(&cOffsetZ[BalLegNr]) + PosZ;
	CPR_X = (short) pgm_read_word(&cOffsetX[BalLegNr]) + PosX;
	CPR_Y = 150 + PosY; // using the value 150 to lower the centerpoint of rotation 'g_InControlState.BodyPos.y +

	TotalTransY += (long) PosY;
	TotalTransZ += (long) CPR_Z;
	TotalTransX += (long) CPR_X;

	lAtan = GetATan2(CPR_X, CPR_Z);
	TotalYBal1 += (lAtan * 1800) / 31415;

	lAtan = GetATan2(CPR_X, CPR_Y);
	TotalZBal1 += ((lAtan * 1800) / 31415) - 900; //Rotate balance circle 90 deg

	lAtan = GetATan2(CPR_Z, CPR_Y);
	TotalXBal1 += ((lAtan * 1800) / 31415) - 900; //Rotate balance circle 90 deg

}
//--------------------------------------------------------------------
//[BalanceBody]
void BalanceBody(void) {

	TotalTransZ = TotalTransZ / BalanceDivFactor;
	TotalTransX = TotalTransX / BalanceDivFactor;
	TotalTransY = TotalTransY / BalanceDivFactor;

#ifndef QUADMODE // ??? on PhantomX Hex at no movment YBal1 = 1800, on Quad = 0...  Need to experiment
	if (TotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
		TotalYBal1 -= 1800;
	else
		TotalYBal1 += 1800;
#endif

	if (TotalZBal1 < -1800) //Compensate for extreme balance positions that causes overflow
		TotalZBal1 += 3600;

	if (TotalXBal1 < -1800) //Compensate for extreme balance positions that causes overflow
		TotalXBal1 += 3600;

	//Balance rotation
	TotalYBal1 = -TotalYBal1 / BalanceDivFactor;
	TotalXBal1 = -TotalXBal1 / BalanceDivFactor;
	TotalZBal1 = TotalZBal1 / BalanceDivFactor;

}
//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1     - Input Angle in degrees
//sin4        - Output Sinus of AngleDeg
//cos4          - Output Cosinus of AngleDeg
void GetSinCos(short AngleDeg1) {
	short ABSAngleDeg1;   //Absolute value of the Angle in Degrees, decimals = 1
	//Get the absolute value of AngleDeg
	if (AngleDeg1 < 0)
		ABSAngleDeg1 = AngleDeg1 * -1;
	else
		ABSAngleDeg1 = AngleDeg1;

	//Shift rotation to a full circle of 360 deg -> AngleDeg // 360
	if (AngleDeg1 < 0)    //Negative values
		AngleDeg1 = 3600 - (ABSAngleDeg1 - (3600 * (ABSAngleDeg1 / 3600)));
	else
		//Positive values
		AngleDeg1 = ABSAngleDeg1 - (3600 * (ABSAngleDeg1 / 3600));

	if (AngleDeg1 >= 0 && AngleDeg1 <= 900)     // 0 to 90 deg
			{
		sin4 = pgm_read_word(&GetSin[AngleDeg1 / 5]); // 5 is the presision (0.5) of the table
		cos4 = pgm_read_word(&GetSin[(900 - (AngleDeg1)) / 5]);
	}

	else if (AngleDeg1 > 900 && AngleDeg1 <= 1800)     // 90 to 180 deg
			{
		sin4 = pgm_read_word(&GetSin[(900 - (AngleDeg1 - 900)) / 5]); // 5 is the presision (0.5) of the table
		cos4 = -pgm_read_word(&GetSin[(AngleDeg1 - 900) / 5]);
	} else if (AngleDeg1 > 1800 && AngleDeg1 <= 2700) // 180 to 270 deg
			{
		sin4 = -pgm_read_word(&GetSin[(AngleDeg1 - 1800) / 5]); // 5 is the presision (0.5) of the table
		cos4 = -pgm_read_word(&GetSin[(2700 - AngleDeg1) / 5]);
	}

	else if (AngleDeg1 > 2700 && AngleDeg1 <= 3600) // 270 to 360 deg
			{
		sin4 = -pgm_read_word(&GetSin[(3600 - AngleDeg1) / 5]); // 5 is the presision (0.5) of the table
		cos4 = pgm_read_word(&GetSin[(AngleDeg1 - 2700) / 5]);
	}
}

//--------------------------------------------------------------------
//(GETARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//cos4        - Input Cosinus
//AngleRad4     - Output Angle in AngleRad4
long GetArcCos(short cos4) {
	bool NegativeValue/*:1*/;    //If the the value is Negative
	//Check for negative value
	if (cos4 < 0) {
		cos4 = -cos4;
		NegativeValue = 1;
	} else
		NegativeValue = 0;

	//Limit cos4 to his maximal value
	cos4 = min(cos4, c4DEC);

	if ((cos4 >= 0) && (cos4 < 9000)) {
		AngleRad4 = (byte) pgm_read_byte(&GetACos[cos4 / 79]);
		AngleRad4 = ((long) AngleRad4 * 616) / c1DEC; //616=acos resolution (pi/2/255) ;
	} else if ((cos4 >= 9000) && (cos4 < 9900)) {
		AngleRad4 = (byte) pgm_read_byte(&GetACos[(cos4 - 9000) / 8 + 114]);
		AngleRad4 = (long) ((long) AngleRad4 * 616) / c1DEC; //616=acos resolution (pi/2/255)
	} else if ((cos4 >= 9900) && (cos4 <= 10000)) {
		AngleRad4 = (byte) pgm_read_byte(&GetACos[(cos4 - 9900) / 2 + 227]);
		AngleRad4 = (long) ((long) AngleRad4 * 616) / c1DEC; //616=acos resolution (pi/2/255)
	}

	//Add negative sign
	if (NegativeValue)
		AngleRad4 = 31416 - AngleRad4;

	return AngleRad4;
}

unsigned long isqrt32(unsigned long n) //
{
	unsigned long root;
	unsigned long remainder;
	unsigned long place;

	root = 0;
	remainder = n;
	place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

	while (place > remainder)
		place = place >> 2;
	while (place) {
		if (remainder >= root + place) {
			remainder = remainder - root - place;
			root = root + (place << 1);
		}
		root = root >> 1;
		place = place >> 2;
	}
	return root;
}

//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4          - Output ARCTAN2(X/Y)
//XYhyp2            - Output presenting Hypotenuse of X and Y
short GetATan2(short AtanX, short AtanY) {
	XYhyp2 = isqrt32(
			((long) AtanX * AtanX * c4DEC) + ((long) AtanY * AtanY * c4DEC));
	GetArcCos(((long) AtanX * (long) c6DEC) / (long) XYhyp2);

	if (AtanY < 0) // removed overhead... Atan4 = AngleRad4 * (AtanY/abs(AtanY));
		Atan4 = -AngleRad4;
	else
		Atan4 = AngleRad4;
	return Atan4;
}

//--------------------------------------------------------------------
//(BODY INVERSE KINEMATICS)
//BodyRotX         - Global Input pitch of the body
//BodyRotY         - Global Input rotation of the body
//BodyRotZ         - Global Input roll of the body
//RotationY         - Input Rotation for the gait
//PosX            - Input position of the feet X
//PosZ            - Input position of the feet Z
//SinB                  - Sin buffer for BodyRotX
//CosB               - Cos buffer for BodyRotX
//SinG                  - Sin buffer for BodyRotZ
//CosG               - Cos buffer for BodyRotZ
//BodyFKPosX         - Output Position X of feet with Rotation
//BodyFKPosY         - Output Position Y of feet with Rotation
//BodyFKPosZ         - Output Position Z of feet with Rotation
void BodyFK(short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg) {
	short SinA4;          //Sin buffer for BodyRotX calculations
	short CosA4;          //Cos buffer for BodyRotX calculations
	short SinB4;          //Sin buffer for BodyRotX calculations
	short CosB4;          //Cos buffer for BodyRotX calculations
	short SinG4;          //Sin buffer for BodyRotZ calculations
	short CosG4;          //Cos buffer for BodyRotZ calculations
	short CPR_X;            //Final X value for centerpoint of rotation
	short CPR_Y;            //Final Y value for centerpoint of rotation
	short CPR_Z;            //Final Z value for centerpoint of rotation

	//Calculating totals from center of the body to the feet
	CPR_X = (short) pgm_read_word(&cOffsetX[BodyIKLeg]) + PosX
			+ g_InControlState.BodyRotOffset.x;
	CPR_Y = PosY + g_InControlState.BodyRotOffset.y; //Define centerpoint for rotation along the Y-axis
	CPR_Z = (short) pgm_read_word(&cOffsetZ[BodyIKLeg]) + PosZ
			+ g_InControlState.BodyRotOffset.z;

	//Successive global rotation matrix:
	//Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate
	//Sinus Alfa = SinA, cosinus Alfa = cosA. and so on...

	//First calculate sinus and cosinus for each rotation:
	GetSinCos(g_InControlState.BodyRot1.x + TotalXBal1);
	SinG4 = sin4;
	CosG4 = cos4;

	GetSinCos(g_InControlState.BodyRot1.z + TotalZBal1);
	SinB4 = sin4;
	CosB4 = cos4;

	GetSinCos(g_InControlState.BodyRot1.y + (RotationY * c1DEC) + TotalYBal1);

	SinA4 = sin4;
	CosA4 = cos4;

	//Calcualtion of rotation matrix:
	BodyFKPosX = ((long) CPR_X * c2DEC
			- ((long) CPR_X * c2DEC * CosA4 / c4DEC * CosB4 / c4DEC
					- (long) CPR_Z * c2DEC * CosB4 / c4DEC * SinA4 / c4DEC
					+ (long) CPR_Y * c2DEC * SinB4 / c4DEC)) / c2DEC;
	BodyFKPosZ = ((long) CPR_Z * c2DEC
			- ((long) CPR_X * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC
					+ (long) CPR_X * c2DEC * CosA4 / c4DEC * SinB4 / c4DEC
							* SinG4 / c4DEC
					+ (long) CPR_Z * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC
					- (long) CPR_Z * c2DEC * SinA4 / c4DEC * SinB4 / c4DEC
							* SinG4 / c4DEC
					- (long) CPR_Y * c2DEC * CosB4 / c4DEC * SinG4 / c4DEC))
			/ c2DEC;
	BodyFKPosY = ((long) CPR_Y * c2DEC
			- ((long) CPR_X * c2DEC * SinA4 / c4DEC * SinG4 / c4DEC
					- (long) CPR_X * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC
							* SinB4 / c4DEC
					+ (long) CPR_Z * c2DEC * CosA4 / c4DEC * SinG4 / c4DEC
					+ (long) CPR_Z * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC
							* SinB4 / c4DEC
					+ (long) CPR_Y * c2DEC * CosB4 / c4DEC * CosG4 / c4DEC))
			/ c2DEC;
}

//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError    - Output true if the solution is NOT possible
//FemurAngle1           - Output Angle of Femur in degrees
//TibiaAngle1           - Output Angle of Tibia in degrees
//CoxaAngle1            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
void LegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ,
		byte LegIKLegNr) {
	unsigned long IKSW2;      //Length between Shoulder and Wrist, decimals = 2
	unsigned long IKA14; //Angle of the line S>W with respect to the ground in radians, decimals = 4
	unsigned long IKA24; //Angle of the line S>W with respect to the femur in radians, decimals = 4
	short IKFeetPosXZ;    //Diagonal direction from Input X and Z

#define TarsOffsetXZ 0      // Vector value
#define TarsOffsetY  0      //Vector value / The 2 DOF IK calcs (femur and tibia) are based upon these vectors

	long Temp1;
	long Temp2;
	long T3;



	//Calculate IKCoxaAngle and IKFeetPosXZ
	GetATan2(IKFeetPosX, IKFeetPosZ);
	CoxaAngle1[LegIKLegNr] = (((long) Atan4 * 180) / 3141)
			+ (short) pgm_read_word(&cCoxaAngle1[LegIKLegNr]);

	CoxaAngle_Odroid[LegIKLegNr] = (((long) Atan4 * 180) / 3141)
					+ (short) pgm_read_word(&cCoxaAngle1[LegIKLegNr]);

	//Length between the Coxa and tars [foot]
	IKFeetPosXZ = XYhyp2 / c2DEC;

	//Using GetAtan2 for solving IKA1 and IKSW
	//IKA14 - Angle between SW line and the ground in radians
	IKA14 = GetATan2(IKFeetPosY - TarsOffsetY,
			IKFeetPosXZ
					- (byte) pgm_read_byte(
							&cCoxaLength[LegIKLegNr])-TarsOffsetXZ);

	//IKSW2 - Length between femur axis and tars
	IKSW2 = XYhyp2;

	//IKA2 - Angle of the line S>W with respect to the femur in radians
	Temp1 = ((((long) (byte) pgm_read_byte(&cFemurLength[LegIKLegNr])
			* (byte) pgm_read_byte(&cFemurLength[LegIKLegNr]))
			- ((long) (byte) pgm_read_byte(&cTibiaLength[LegIKLegNr])
					* (byte) pgm_read_byte(&cTibiaLength[LegIKLegNr]))) * c4DEC
			+ ((long) IKSW2 * IKSW2));
	Temp2 = (long) (2 * (byte) pgm_read_byte(&cFemurLength[LegIKLegNr])) * c2DEC
			* (unsigned long) IKSW2;
	T3 = Temp1 / (Temp2 / c4DEC);
	IKA24 = GetArcCos(T3);



	FemurAngle1[LegIKLegNr] = -(long) (IKA14 + IKA24) * 180
			/ 3141+ 900 + CFEMURHORNOFFSET1(LegIKLegNr);  //Normal

	FemurAngle_Odroid[LegIKLegNr] =  -(long) (IKA14 + IKA24) * 180
			/ 3141+ 900 + CFEMURHORNOFFSET1(LegIKLegNr);  //Normal

	//IKTibiaAngle
	Temp1 = ((((long) (byte) pgm_read_byte(&cFemurLength[LegIKLegNr])
			* (byte) pgm_read_byte(&cFemurLength[LegIKLegNr]))
			+ ((long) (byte) pgm_read_byte(&cTibiaLength[LegIKLegNr])
					* (byte) pgm_read_byte(&cTibiaLength[LegIKLegNr]))) * c4DEC
			- ((long) IKSW2 * IKSW2));
	Temp2 = 2 * ((long) ((byte) pgm_read_byte(&cFemurLength[LegIKLegNr])))
			* (long) ((byte) pgm_read_byte(&cTibiaLength[LegIKLegNr]));
	GetArcCos(Temp1 / Temp2);

/*
	TibiaAngle1[LegIKLegNr] = -(1450 - (long) AngleRad4 * 180 / 3141
			+ CTIBIAHORNOFFSET1(LegIKLegNr)); //!!!!!!!!!!!!145 instead of 1800
*/
	TibiaAngle1[LegIKLegNr] = -(1350 - (long) AngleRad4 * 180 / 3141
			+ CTIBIAHORNOFFSET1(LegIKLegNr)); //!!!!!!!!!!!!145 instead of 1800

	TibiaAngle_Odroid[LegIKLegNr] = -(1350 - (long) AngleRad4 * 180 / 3141
			+ CTIBIAHORNOFFSET1(LegIKLegNr)); //!!!!!!!!!!!!145 instead of 1800

	//Set the Solution quality
	if (IKSW2
			< ((word) ((byte) pgm_read_byte(&cFemurLength[LegIKLegNr])
					+ (byte) pgm_read_byte(&cTibiaLength[LegIKLegNr]) - 30)
					* c2DEC))
		IKSolution = 1;
	else {
		if (IKSW2
				< ((word) ((byte) pgm_read_byte(&cFemurLength[LegIKLegNr])
						+ (byte) pgm_read_byte(&cTibiaLength[LegIKLegNr]))
						* c2DEC))
			IKSolutionWarning = 1;
		else
			IKSolutionError = 1;
	}

}

//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
short CheckServoAngleBounds(short sID, short sVal, const short *sMin,
		const short *sMax) {

	// Pull into simple function as so I can report errors on debug
	// Note ID is bogus, but something to let me know which one.
	short s = (short) pgm_read_word(sMin);
	if (sVal < s) {
		return s;
	}

	s = (short) pgm_read_word(sMax);
	if (sVal > s) {

		return s;
	}
	return sVal;

}

//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void) {
/**#ifndef SERVOS_DO_MINMAX
	short s = 0;  // BUGBUG just some index so we can get a hint who errored out
	for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
		CoxaAngle1[LegIndex] = CheckServoAngleBounds(s++, CoxaAngle1[LegIndex],
				&cCoxaMin1[LegIndex], &cCoxaMax1[LegIndex]);
		FemurAngle1[LegIndex] = CheckServoAngleBounds(s++,
				FemurAngle1[LegIndex], &cFemurMin1[LegIndex],
				&cFemurMax1[LegIndex]);
		TibiaAngle1[LegIndex] = CheckServoAngleBounds(s++,
				TibiaAngle1[LegIndex], &cTibiaMin1[LegIndex],
				&cTibiaMax1[LegIndex]);

	}
#endif*/
	  for (LegIndex = 0; LegIndex <=5; LegIndex++)
	  {
	    CoxaAngle1[LegIndex]  = min(max(CoxaAngle1[LegIndex], (short)pgm_read_word(&cCoxaMin1[LegIndex])),
	    (short)pgm_read_word(&cCoxaMax1[LegIndex]));
	    FemurAngle1[LegIndex] = min(max(FemurAngle1[LegIndex], (short)pgm_read_word(&cFemurMin1[LegIndex])),
	    (short)pgm_read_word(&cFemurMax1[LegIndex]));
	    TibiaAngle1[LegIndex] = min(max(TibiaAngle1[LegIndex], (short)pgm_read_word(&cTibiaMin1[LegIndex])),
	    (short)pgm_read_word(&cTibiaMax1[LegIndex]));
	  }
}

//--------------------------------------------------------------------
// SmoothControl (From Zenta) -  This function makes the body
//            rotation and translation much smoother
//--------------------------------------------------------------------
short SmoothControl(short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider) {

	if (CtrlMoveOut < (CtrlMoveInp - 4))
		return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp) / CtrlDivider);
	else if (CtrlMoveOut > (CtrlMoveInp + 4))
		return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp) / CtrlDivider);

	return CtrlMoveInp;
}

//--------------------------------------------------------------------
// GetLegsXZLength -
//--------------------------------------------------------------------
word g_wLegsXZLength = 0xffff;
word GetLegsXZLength(void) {
	// Could save away or could do a little math on one leg...
	if (g_wLegsXZLength != 0xffff)
		return g_wLegsXZLength;

	return isqrt32((LegPosX[0] * LegPosX[0]) + (LegPosZ[0] * LegPosZ[0]));
}

//--------------------------------------------------------------------
// AdjustLegPositions() - Will adjust the init leg positions to the
//      width passed in.
//--------------------------------------------------------------------
#ifndef MIN_XZ_LEG_ADJUST
#define MIN_XZ_LEG_ADJUST (cCoxaLength[0])      // don't go inside coxa...
#endif

#ifndef MAX_XZ_LEG_ADJUST
#define MAX_XZ_LEG_ADJUST   (word)(cCoxaLength[0]+cTibiaLength[0] + cFemurLength[0]/4)
#endif

void AdjustLegPositions(word XZLength1) {
	//now lets see what happens when we change the leg positions...
	if (XZLength1 > MAX_XZ_LEG_ADJUST)
		XZLength1 = MAX_XZ_LEG_ADJUST;
	if (XZLength1 < MIN_XZ_LEG_ADJUST)
		XZLength1 = MIN_XZ_LEG_ADJUST;

	// see if same length as when we came in
	if (XZLength1 == g_wLegsXZLength)
		return;

	g_wLegsXZLength = XZLength1;

	u8 LegIndex;
	for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {

		GetSinCos((short) pgm_read_word(&cCoxaAngle1[LegIndex]));

		LegPosX[LegIndex] = ((long) ((long) cos4 * XZLength1)) / c4DEC; //Set start positions for each leg
		LegPosZ[LegIndex] = -((long) ((long) sin4 * XZLength1)) / c4DEC;
	}

	// Make sure we cycle through one gait to have the legs all move into their new locations...
	g_InControlState.ForceGaitStepCnt = g_InControlState.gaitCur.StepsInGait;
}

//--------------------------------------------------------------------
// ResetLegInitAngles - This is used when we allow the user to
// adjust the leg position angles.  This resets to what it was when the
// the program was started.
//--------------------------------------------------------------------
void ResetLegInitAngles(void) {
#ifdef OPT_DYNAMIC_ADJUST_LEGS
	for (int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
#ifdef cRRInitCoxaAngle1    // We can set different angles for the legs than just where they servo horns are set...
		g_InControlState.aCoxaInitAngle1[LegIndex] = (short)pgm_read_word(&cCoxaInitAngle1[LegIndex]);
#else
		g_InControlState.aCoxaInitAngle1[LegIndex] = (short)pgm_read_word(&cCoxaAngle1[LegIndex]);
#endif
	}
	g_wLegsXZLength = 0xffff;
#endif
}

//--------------------------------------------------------------------
// ResetLegInitAngles - This is used when we allow the user to
//--------------------------------------------------------------------
void RotateLegInitAngles(int iDeltaAngle) {
#ifdef OPT_DYNAMIC_ADJUST_LEGS
	for (int LegIndex=0; LegIndex < CNT_LEGS; LegIndex++) {
		// We will use the cCoxaAngle1 array to know which direction the legs logically are
		// If the initial angle is 0 don't mess with.  Hex middle legs...
		if ((short)pgm_read_word(&cCoxaAngle1[LegIndex]) > 0)
		g_InControlState.aCoxaInitAngle1[LegIndex] += iDeltaAngle;
		else if ((short)pgm_read_word(&cCoxaAngle1[LegIndex]) < 0)
		g_InControlState.aCoxaInitAngle1[LegIndex] -= iDeltaAngle;

		// Make sure we don't exceed some min/max angles.
		// Right now hard coded to +-70 degrees... Should probably load
		if (g_InControlState.aCoxaInitAngle1[LegIndex] > 700)
		g_InControlState.aCoxaInitAngle1[LegIndex] = 700;
		else if (g_InControlState.aCoxaInitAngle1[LegIndex] < -700)
		g_InControlState.aCoxaInitAngle1[LegIndex] = -700;
	}
	g_wLegsXZLength = 0xffff;
#endif
}

//--------------------------------------------------------------------
// AdjustLegPositionsToBodyHeight() - Will try to adjust the position of the legs
//     to be appropriate for the current y location of the body...
//--------------------------------------------------------------------

u8 g_iLegInitIndex = 0x00; // remember which index we are currently using...

void AdjustLegPositionsToBodyHeight() {
#ifdef CNT_HEX_INITS
	// Lets see which of our units we should use...
	// Note: We will also limit our body height here...
	if (g_InControlState.BodyPos.y
			> (short) pgm_read_byte(&g_abHexMaxBodyY[CNT_HEX_INITS-1]))
		g_InControlState.BodyPos.y = (short) pgm_read_byte(
				&g_abHexMaxBodyY[CNT_HEX_INITS-1]);

	u8 i;
	word XZLength1 = pgm_read_byte(&g_abHexIntXZ[CNT_HEX_INITS-1]);
	for (i = 0; i < (CNT_HEX_INITS - 1); i++) { // Don't need to look at last entry as we already init to assume this one...
		if (g_InControlState.BodyPos.y
				<= (short) pgm_read_byte(&g_abHexMaxBodyY[i])) {
			XZLength1 = pgm_read_byte(&g_abHexIntXZ[i]);
			break;
		}
	}
	if (i != g_iLegInitIndex) {
		g_iLegInitIndex = i;  // remember the current index...
		AdjustLegPositions(XZLength1);
	}
#endif // CNT_HEX_INITS
}


