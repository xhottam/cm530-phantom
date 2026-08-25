/*
 * BioloidEx.h
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_APP_INC_BIOLOIDEX_H_
#define CM530_APP_INC_BIOLOIDEX_H_

#include "stm32f10x_type.h"

//#define DEBUG_BIOLOIDEX


//WAIT_SLOP_FACTOR BioloidEx0
//BIOLOID_FRAME_LENGTH BioloidEx

//ZIGBEE_BUFFER_LENGTH cm530

//DEFAULT_GAIT_SPEED 50  Hex_Cfg
//DEFAULT_SLOW_GAIT  50  Hex_Cfg

//BALANCE_DELAY barebones
//ServoMoveTime barebones
//lTimeWaitEnd barebones

//cTibiaHornOffset1


/* pose engine runs at 30Hz (33ms between frames)
 recommended values for interpolateSetup are of the form X*BIOLOID_FRAME_LENGTH - 1 */
//#define BIOLOID_FRAME_LENGTH      33
#define BIOLOID_FRAME_LENGTH      33
/* we need some extra resolution, use 13 bits, rather than 10, during interpolation */
//#define BIOLOID_SHIFT             3

#define AX12_MAX_SERVOS           18

/** a structure to hold transitions **/
typedef struct {
	unsigned int * pose;    // addr of pose to transition to
	int time;               // time for transition
} transition_t;


void BioloidControllerEx();

/* New-style constructor/setup */

void Bioloid_Setup(int servo_cnt);

/* Pose Manipulation */
void BioloidControllerEx_loadPose(const unsigned int * addr); // load a named pose from FLASH
void BioloidControllerEx_readPose();           // read a pose in from the servos
void BioloidControllerEx_writePose();          // write a pose out to the servos
int BioloidControllerEx_getCurPose(int id); // get a servo value in the current pose
int BioloidControllerEx_getNextPose(int id); // get a servo value in the next pose
void BioloidControllerEx_setNextPose(int id, int pos); // set a servo value in the next pose
void BioloidControllerEx_setNextPoseByIndex(int index, int pos); // set a servo value by index for next pose
void BioloidControllerEx_setId(int index, int id); // set the id of a particular storage index
int BioloidControllerEx_getId(int index); // get the id of a particular storage index

/* Pose Engine */
void BioloidControllerEx_interpolateSetup(int time); // calculate speeds for smooth transition
int BioloidControllerEx_interpolateStep(bool fWait); // move forward one step in current interpolation
extern unsigned char interpolating;          // are we in an interpolation? 0=No, 1=Yes
extern unsigned char runningSeq;              // are we running a sequence? 0=No, 1=Yes
extern int poseSize;                // how many servos are in this pose, used by Sync()

// Kurt's Hacks
extern u8 frameLength;                 // Allow variable frame lengths, to test...

/* to interpolate:
 *  bioloid.loadPose(myPose);
 *  bioloid.interpolateSetup(67);
 *  while(bioloid.interpolating > 0){
 *      bioloid.interpolateStep();
 *      delay(1);
 *  }
 */

/*unsigned int * pose_;                       // the current pose, updated by Step(), set out by Sync()
 unsigned int * nextpose_;                   // the destination pose, where we put on load
 int * speed_;                               // speeds for interpolation
 unsigned char * id_;   */                     // servo id for this index

extern unsigned int pose_[] ;
extern unsigned int nextpose_[];
extern int speed_[];
extern unsigned char id_[];

//    unsigned long lastframe_;                   // time last frame was sent out
extern unsigned long nextframe_;                   //
extern transition_t * sequence;                    // sequence we are running
extern int transitions;                    // how many transitions we have left to load

#endif /* CM530_APP_INC_BIOLOIDEX_H_ */
