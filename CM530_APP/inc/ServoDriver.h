/*
 * ServoDriver.h
 *
 *  Created on: 13/02/2016
 *      Author: xottam
 */

#ifndef SERVODRIVER_H_
#define SERVODRIVER_H_

void Servo_Init(void);

word GetBatteryVoltage(void);

void BeginServoUpdate(void);    // Start the update

void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1,
		short sTibiaAngle1);

void CommitServoDriver(word wMoveTime);
void FreeServos(void);

void IdleTime(void);       // called when the main loop when the robot is not on

// Allow for background process to happen...
#ifdef OPT_BACKGROUND_PROCESS
void BackgroundProcess(void);
#endif    

#endif /* SERVODRIVER_H_ */
