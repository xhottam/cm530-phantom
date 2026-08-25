/*
 * BioloidEx.c
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */


#include "BioloidEx.h"
#include "dynamixel_address_tables.h"
#include "dynamixel.h"
#include "serial.h"
#include "wiring.h"
#include "system_func.h"
#include "zigbee.h"
#include "Phoenix.h"

unsigned char interpolating;          // are we in an interpolation? 0=No, 1=Yes
unsigned char runningSeq;              // are we running a sequence? 0=No, 1=Yes
int poseSize;                // how many servos are in this pose, used by Sync()
u8 frameLength;                 // Allow variable frame lengths, to test...
unsigned int pose_[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned int nextpose_[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0 };
int speed_[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char id_[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//    unsigned long lastframe_;                   // time last frame was sent out
unsigned long nextframe_;                   //
transition_t * sequence;                    // sequence we are running
int transitions;                    // how many transitions we have left to load


//  initializes serial1 transmit at baud, 8-N-1
void BioloidControllerEx() {
	int i;
	// setup storage
	/**id_ = (unsigned char *) malloc(AX12_MAX_SERVOS * sizeof(unsigned char));
	 pose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
	 nextpose_ = (unsigned int *) malloc(AX12_MAX_SERVOS * sizeof(unsigned int));
	 speed_ = (int *) malloc(AX12_MAX_SERVOS * sizeof(int));*/
	// initialize
	bool wakeup = 0;
		BioloidControllerEx_findServo(0,wakeup);
	for (i = 0; i < AX12_MAX_SERVOS; i++) {
		BioloidControllerEx_findServo(i+1,1);
		id_[i] = i + 1;
		pose_[i] = 512;
		nextpose_[i] = 512;
	}
	frameLength = BIOLOID_FRAME_LENGTH;
	interpolating = 0;
	nextframe_ = getMillis_TIM2();
#ifdef MILLIS
	PrintString("BioloidEX nextframe_ ");
	Printu32d(nextframe_);
	PrintString("\n");
#endif
}

/* new-style setup */
void Bioloid_Setup(int servo_cnt) {
	int i;
	// setup storage
	/**id_ = (unsigned char *) malloc(servo_cnt * sizeof(unsigned char));
	 pose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
	 nextpose_ = (unsigned int *) malloc(servo_cnt * sizeof(unsigned int));
	 speed_ = (int *) malloc(servo_cnt * sizeof(int));*/
	// initialize
	poseSize = servo_cnt;
	for (i = 0; i < poseSize; i++) {
		id_[i] = i + 1;
		pose_[i] = 512;
		nextpose_[i] = 512;
	}
	interpolating = 0;
	nextframe_ = getMillis_TIM2();
#ifdef MILLIS
	PrintString("BioloidEX nextframe_ ");
	Printu32d(nextframe_);
	PrintString("\n");
#endif
}
void BioloidControllerEx_setId(int index, int id) {
	id_[index] = id;
}
int BioloidControllerEx_getId(int index) {
	return id_[index];
}

/* load a named pose from FLASH into nextpose. */
void BioloidControllerEx_loadPose(const unsigned int * addr) {
	int i;
	poseSize = pgm_read_word_near(addr); // number of servos in this pose
	for (i = 0; i < poseSize; i++)
		//nextpose_[i] = pgm_read_word_near(addr+1+i) << BIOLOID_SHIFT;
		nextpose_[i] = pgm_read_word_near(addr + 1 + i);
}
/* read in current servo positions to the pose. */
void BioloidControllerEx_readPose() {
	int i;

#ifdef DEBUG_BIOLOIDEX
	PrintString("Bioloid controller --- READ POSE ---\n");
	PrintString("ID - POSICION\n");
#endif
	for (i = 0; i < poseSize; i++) {
		//pose_[i] = dxl_read_word(id_[i],AXM_PRESENT_POSITION_L)<<BIOLOID_SHIFT;
		pose_[i] = dxl_read_word(id_[i], AXM_PRESENT_POSITION_L);

#ifdef DEBUG_BIOLOIDEX
		PrintChar(id_[i]);
		PrintString(" - ");
		TxD_Dec_U16(pose_[i]);
		PrintString("\n");
#endif
		mDelay(25);
	}
}
/* write pose out to servos using sync write. */
void BioloidControllerEx_writePose() {
	int temp;
	dxl_set_txpacket_id (BROADCAST_ID);
	dxl_set_txpacket_instruction (INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, AXM_GOAL_POSITION_L);
	dxl_set_txpacket_parameter(1, 2);
	int i;
	for (i = 0; i < poseSize; i++) {
		//temp = pose_[i] >> BIOLOID_SHIFT;
		temp = pose_[i];
		dxl_set_txpacket_parameter(2 + 3 * i, id_[i]);
		dxl_set_txpacket_parameter(2 + 3 * i + 1, dxl_get_lowbyte(temp));
		dxl_set_txpacket_parameter(2 + 3 * i + 2, dxl_get_highbyte(temp));
	}
	dxl_set_txpacket_length((2 + 1) * poseSize + 4);
	dxl_txrx_packet();
	u16 CommStatus = dxl_get_result();
/**#ifdef USING_PC_UART
	if (CommStatus == DXL_RXSUCCESS)
		PrintErrorCode();
	else
		PrintCommStatus(CommStatus);
#endif*/
}

/* set up for an interpolation from pose to nextpose over TIME
 milliseconds by setting servo speeds. */
void BioloidControllerEx_interpolateSetup(int time) {
	int i;
	int frames = (time/frameLength) + 1;
	nextframe_ = getMillis_TIM2() + frameLength;
#ifdef MILLIS
	PrintString("BioloidEX BioloidControllerEx_interpolateSetup frames ");
	Printu32d(frames);
	PrintString("\n");
	PrintString("BioloidEX BioloidControllerEx_interpolateSetup nextframe_ ");
	Printu32d(nextframe_);
	PrintString("\n");
#endif
	// set speed each servo...
	for (i = 0; i < poseSize; i++) {
		if (nextpose_[i] > pose_[i]) {
			speed_[i] = (nextpose_[i] - pose_[i]) / frames + 1;
		} else {
			speed_[i] = (pose_[i] - nextpose_[i]) / frames + 1;
		}
#ifdef DEBUG_BIOLOIDEX
		PrintString("Bioloid controller --- INTERPOLATE SETUP ---\n");
		PrintString("ID - SPEED\n");
		PrintChar(id_[i]);
		PrintString(" - ");
		TxD_Dec_U16(speed_[i]);
		PrintString("\n");
#endif
	}
	interpolating = 1;
}
/* interpolate our pose, this should be called at about 30Hz. */
#define WAIT_SLOP_FACTOR 10
int BioloidControllerEx_interpolateStep(bool fWait) {

	if (interpolating == 0)
		return 0;
	int i;
	int complete = poseSize;
	if (!fWait) {
		if (getMillis_TIM2() < (nextframe_ - WAIT_SLOP_FACTOR)) {
			return (getMillis_TIM2() - nextframe_); // We still have some time to do something...
		}
	}
#ifdef MILLIS
	PrintString("BioloidEX  BioloidControllerEx_interpolateStep nextframe_ ");
	Printu32d(nextframe_);
	PrintString("\n");
#endif
	while (getMillis_TIM2() < nextframe_);
#ifdef MILLIS
	PrintString("BioloidEX  BioloidControllerEx_interpolateStep nextframe_-Millis ");
	Printu32d(getMillis_TIM2());
	PrintString("\n");
#endif

	nextframe_ = getMillis_TIM2() + frameLength;
#ifdef MILLIS
	PrintString("BioloidEX  BioloidControllerEx_interpolateStep nextframe_ (Millis + framelength) ");
	Printu32d(nextframe_);
	PrintString("\n");
#endif
	// update each servo
	for (i = 0; i < poseSize; i++) {
		int diff = nextpose_[i] - pose_[i];
		if (diff == 0) {
			complete--;
		} else {
			if (diff > 0) {
				if (diff < speed_[i]) {
					pose_[i] = nextpose_[i];
					complete--;
				} else
					pose_[i] += speed_[i];
			} else {
				if ((-diff) < speed_[i]) {
					pose_[i] = nextpose_[i];
					complete--;
				} else
					pose_[i] -= speed_[i];
			}
		}

#ifdef DEBUG_BIOLOIDEX
		PrintString("Bioloid controller --- INTERPOLATE STEP ---\n");
		PrintString("ID - POSE\n");
		PrintChar(id_[i]);
		PrintString(" - ");
		TxD_Dec_U16(pose_[i]);
		PrintString("\n");
#endif

	}
	if (complete <= 0){
		interpolating = 0;
	}
	BioloidControllerEx_writePose();
	return 0;
}

/* get a servo value in the current pose */
int BioloidControllerEx_getCurPose(int id) {
	int i;
	for (i = 0; i < poseSize; i++) {
		if (id_[i] == id)
			//return ((pose_[i]) >> BIOLOID_SHIFT);
			return ((pose_[i]));
	}
	return -1;
}
/* get a servo value in the next pose */
int BioloidControllerEx_getNextPose(int id) {
	int i;
	for (i = 0; i < poseSize; i++) {
		if (id_[i] == id)
			//return ((nextpose_[i]) >> BIOLOID_SHIFT);
			return ((nextpose_[i]));
	}
	return -1;
}
/* set a servo value in the next pose */
void BioloidControllerEx_setNextPose(int id, int pos) {
	int i;
	for (i = 0; i < poseSize; i++) {
		if (id_[i] == id) {
			//nextpose_[i] = (pos << BIOLOID_SHIFT);
			nextpose_[i] = (pos);
			return;
		}
	}
}

/* Added by Kurt */
void BioloidControllerEx_setNextPoseByIndex(int index, int pos) { // set a servo value by index for next pose
	if (index < poseSize) {
		//nextpose_[index] = (pos << BIOLOID_SHIFT);
		nextpose_[index] = (pos);
	}
}

// if found returns , if not 0xff
void BioloidControllerEx_findServo(int id,bool wakeup){


	u16 wdata;
	u16 error = 0;
	wdata = (dxl_read_byte(id, P_ID) & 0x00FF);
	if (wakeup){
		if (wdata == id) {
			wdata = 0;
			zgb_PrintString("{");
			zgb_Printu32d(id);
			zgb_PrintString(", ");

			wdata = dxl_read_word(id, P_MODEL_NUMBER_L);
			error = dxl_get_result();
			if (!(error & DXL_RXSUCCESS))
				zgb_PrintCommStatus(error);
			zgb_Printu32d(wdata);
			if (wdata == MODEL_AX12) {
				zgb_PrintString(" (ax-12)");
			} else if (wdata == MODEL_AX18) {
				zgb_PrintString(" (AX-18)");
			} else if (wdata == MODEL_AXS1) {
				zgb_PrintString(" (AX-S1)");
			} else if (wdata == MODEL_AXS20) {
				zgb_PrintString(" (AX-S20)");
			} else if (wdata == MODEL_JHFPS) {
				zgb_PrintString(" (JH-FPS)");
			} else if (wdata == MODEL_MX28) {
				zgb_PrintString(" (MX-28)");
			} else if (wdata == MODEL_HaViMo2) {
				zgb_PrintString(" (HaViMo2)");
			}

			zgb_PrintString(", ");
			zgb_Printu32d(dxl_read_byte(id, P_FIRMWARE_VERSION));
			zgb_PrintString("} \n");
		}else {
			zgb_PrintString("{ servo :");
			zgb_Printu32d(id);
			zgb_PrintString(", ");
			zgb_PrintString("not_found");
			zgb_PrintString("} \n");

		}
	}
}

void BioloidControllerEx_AX_ReadControlTable(int id,byte section){

	dxl_set_txpacket_id(id);
	dxl_set_txpacket_instruction(INST_READ_DATA);
	dxl_set_txpacket_parameter(0,0);
	dxl_set_txpacket_parameter(1,49);
	dxl_set_txpacket_length(4);

	dxl_txrx_packet();

	u16 CommStatus = dxl_get_result();
	bool checkStatus = zgb_PrintCommStatus(CommStatus);

	if ( (section == 'e' || section == 'E' || section == 'a' || section == 'A') && checkStatus ){
		zgb_PrintString("EEPROM ** ");
		zgb_PrintString("|model:");
		zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_MODEL_NUMBER_L],gbStatusPacket[DXL_PKT_PARA + AXM_MODEL_NUMBER_H]));
		zgb_PrintString(" |firmware:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_FIRMWARE_VERSION ]);
		zgb_PrintString(" |id:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_ID ]);
		zgb_PrintString(" |baud:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_BAUD_RATE ]);
		zgb_PrintString(" |returnDelay:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_RETURN_DELAY_TIME ]);
		zgb_PrintString(" |CWAngleLimit:");
		zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_CW_ANGLE_LIMIT_L],gbStatusPacket[DXL_PKT_PARA + AXM_CW_ANGLE_LIMIT_H]));
		zgb_PrintString(" |CCWAngleLimit:");
		zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_CCW_ANGLE_LIMIT_L],gbStatusPacket[DXL_PKT_PARA + AXM_CCW_ANGLE_LIMIT_H]));
		zgb_PrintString(" |HLimitTemp:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_HIGHEST_LIMIT_TEMPERATURE ]);
		zgb_PrintString(" |HLimitVolt:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_HIGHEST_LIMIT_VOLTAGE ]);
		zgb_PrintString(" |LLimitVolt:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_LOWEST_LIMIT_VOLTAGE ]);
		zgb_PrintString(" |MaxTorque:");
		zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_MAX_TORQUE_L],gbStatusPacket[DXL_PKT_PARA + AXM_MAX_TORQUE_H]));
		zgb_PrintString(" |StatusReturnLevel:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_STATUS_RETURN_LEVEL ]);
		zgb_PrintString(" |AlarmLed:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_ALARM_LED ]);
		zgb_PrintString(" |AlarmShutdown:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_ALARM_SHUTDOWN ]);
		zgb_PrintString("**\n");
	}
	if ((section == 'r' || section == 'R' || section == 'a' || section == 'A') && checkStatus){
		zgb_PrintString("RAM ** ");
		zgb_PrintString(" |TorqueEnable:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_TORQUE_ENABLE ]);
		zgb_PrintString(" |Led:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_LED ]);
		zgb_PrintString(" |CWMargin:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_CW_COMPLIANCE_MARGIN   ]);
		zgb_PrintString(" |CCWMargin:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_CCW_COMPLIANCE_MARGIN  ]);
		zgb_PrintString(" |CWSlope:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_CW_COMPLIANCE_SLOPE ]);
		zgb_PrintString(" |CCWSlope:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_CCW_COMPLIANCE_SLOPE  ]);
		zgb_PrintString(" |PositionGoal:");
	    zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_GOAL_POSITION_L ],gbStatusPacket[DXL_PKT_PARA + AXM_GOAL_POSITION_H ]));
	    zgb_PrintString(" |MovingSpeed:");
	    zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_MOVING_SPEED_L ],gbStatusPacket[DXL_PKT_PARA + AXM_MOVING_SPEED_H ]));
	    zgb_PrintString(" |TorqueLimit:");
	    zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_TORQUE_LIMIT_L ],gbStatusPacket[DXL_PKT_PARA + AXM_TORQUE_LIMIT_H ]));
	    zgb_PrintString(" |PresentPosition:");
	    zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_PRESENT_POSITION_L ],gbStatusPacket[DXL_PKT_PARA + AXM_PRESENT_POSITION_H ]));
	    zgb_PrintString(" |PresentSpeed:");
	    zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_PRESENT_SPEED_L ],gbStatusPacket[DXL_PKT_PARA + AXM_PRESENT_SPEED_H ]));
	    zgb_PrintString(" |PresentLoad:");
	    zgb_TxD_Dec_U16(dxl_makeword(gbStatusPacket[DXL_PKT_PARA + AXM_PRESENT_LOAD_L ],gbStatusPacket[DXL_PKT_PARA + AXM_PRESENT_LOAD_H ]));
	    zgb_PrintString(" |Voltage:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_PRESENT_VOLTAGE   ]);
		zgb_PrintString(" |Temp:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_PRESENT_TEMPERATURE   ]);
		zgb_PrintString(" |RegisteredInstruc:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_REGISTERED_INSTRUCTION   ]);
		zgb_PrintString(" |Moving:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_MOVING   ]);
		zgb_PrintString(" |Lock:");
		zgb_TxD_Dec_U8(gbStatusPacket[DXL_PKT_PARA + AXM_LOCK   ]);
		zgb_PrintString("**\n");
	}
}
void BioloidControllerEx_AX_AddressTable(){

	zgb_PrintString("(I)ID{3}\n");
	zgb_PrintString("(T)PRESENT_TEMPERATURE [0-70]\n");
	zgb_PrintString("(R)RETURN_DELAY_TIME [0-254]\n");
	zgb_PrintString("(S)STATUS_RETURN_LEVEL [0-1-2]\n");
	zgb_PrintString("(A)ALARM_LED [?]\n");
	zgb_PrintString("(B)ALARM_SHUTDOWN [?]\n");
	zgb_PrintString("(G)POSITION GOAL [0-1023]\n");
	zgb_PrintString("(M)MOVING_SPEED [0-1023]\n");
	zgb_PrintString("(Q)TORQUE_ENABLE [0-1]\n");
	zgb_PrintString("(O)MAX_TORQUE [0-1023]\n");
	zgb_PrintString("(W)CW_COMPLIANCE_MARGIN [0-255]\n");
	zgb_PrintString("(X)CCW_COMPLIANCE_MARGIN [0-255]\n");
	zgb_PrintString("(Y)CW_COMPLIANCE_SLOPE [2-4-8-16-32-64-128]\n");
	zgb_PrintString("(Z)CCW_COMPLIANCE_SLOPE [2-4-8-16-32-64-128]\n");
	zgb_PrintString("(P)PUNCH [32-1023]\n\n");
}

bool sizeControl = FALSE;
u16 zgb_Pick_Word_from_PC(){
	u16 value = 0;
	bool unidades = FALSE;
	bool decenas = FALSE;
	bool centenas = FALSE;
	bool millar = FALSE;
	sizeControl = FALSE;
	while (1){
		zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
		if (gbRcvPacket[0] != '#'){
			if (!unidades){
				value = value + (u16) gbRcvPacket[0] - 48;
				unidades = TRUE;
			}else{
				if (!decenas){
					value =  (value * 10) + (u16) gbRcvPacket[0] - 48;
					decenas = TRUE;
				}else{
					if (!centenas){
						value =  (value * 10) + (u16) gbRcvPacket[0] - 48;
						centenas = TRUE;
					}else{
						if (!millar){
							value =  (value * 10) + (u16) gbRcvPacket[0] - 48;
							millar = TRUE;
						}
					}
				}
			}
			}else{
				sizeControl = TRUE;
				break;
			}
		}
	return value;
}

u8 zgb_Pick_Byte_from_PC(){

	u8 value = 0;
	bool unidades = FALSE;
	bool decenas = FALSE;
	bool centenas = FALSE;
	sizeControl = FALSE;
	while (1){
		zgb_hal_rx((u8*)&gbRcvPacket[0], 1);
		if (gbRcvPacket[0] != '#'){
			if (!unidades){
				value = value +(u8) gbRcvPacket[0] - 48;
				unidades = TRUE;
			}else{
				if (!decenas){
					value =  (value * 10) + (u8) gbRcvPacket[0] - 48;
					decenas = TRUE;
				}else{
					if (!centenas){
						value =  (value * 10) + (u8) gbRcvPacket[0] - 48;
						centenas = TRUE;
					}
				}
			}
			}else{
				sizeControl = TRUE;
				break;
			}
		}
	return value;
}

void BioloidControllerEx_AX_RW(u8 idw, byte rw, byte size, byte address){


	if (rw == 'R') {
		if (size == '1'){
			if (address == 'I'){
				zgb_PrintString("ID->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_ID));
				zgb_PrintString("\n");
			}
			if (address == 'T'){
				zgb_PrintString("PRESENT_TEMPERATURE->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_PRESENT_TEMPERATURE));
				zgb_PrintString("\n");
			}
			if (address == 'R'){
				zgb_PrintString("RETURN_DELAY_TIME->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_RETURN_DELAY_TIME));
				zgb_PrintString("\n");
			}
			if (address == 'S'){
				zgb_PrintString("STATUS_RETURN_LEVEL->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_STATUS_RETURN_LEVEL));
				zgb_PrintString("\n");
			}
			if (address == 'Q'){
				zgb_PrintString("TORQUE_ENABLE->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_TORQUE_ENABLE));
				zgb_PrintString("\n");
			}
			if (address == 'W'){
				zgb_PrintString("CW_COMPLIANCE_MARGIN->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_CW_COMPLIANCE_MARGIN));
				zgb_PrintString("\n");
			}
			if (address == 'X'){
				zgb_PrintString("CCW_COMPLIANCE_MARGIN->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_CCW_COMPLIANCE_MARGIN));
				zgb_PrintString("\n");
			}
			if (address == 'Y'){
				zgb_PrintString("CW_COMPLIANCE_SLOPE->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_CW_COMPLIANCE_SLOPE));
				zgb_PrintString("\n");
			}
			if (address == 'Z'){
				zgb_PrintString("CCW_COMPLIANCE_SLOPE->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_CCW_COMPLIANCE_SLOPE));
				zgb_PrintString("\n");
			}
			if (address == 'A'){
				zgb_PrintString("ALARM_LED->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_ALARM_LED));
				zgb_PrintString("\n");
			}
			if (address == 'B'){
				zgb_PrintString("ALARM_SHUTDOWN->");
				zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_ALARM_SHUTDOWN));
				zgb_PrintString("\n");
			}
		}
		if (size == '2'){
			if (address == 'M'){
				zgb_PrintString("MOVING_SPEED->");
				zgb_TxD_Dec_U16(dxl_read_word(idw,AXM_MOVING_SPEED_L));
				zgb_PrintString("\n");
			}
			if (address == 'Q'){
				zgb_PrintString("MAX_TORQUE->");
				zgb_TxD_Dec_U16(dxl_read_word(idw,AXM_MAX_TORQUE_L));
				zgb_PrintString("\n");
			}
			if (address == 'P'){
				zgb_PrintString("PUNCH->");
				zgb_TxD_Dec_U16(dxl_read_word(idw,AXM_PUNCH_L));
				zgb_PrintString("\n");
			}
			if (address == 'G'){
				zgb_PrintString("POSITION_GOAL->");
				zgb_TxD_Dec_U16(dxl_read_word(idw,AXM_GOAL_POSITION_L));
				zgb_PrintString("\n");
			}
		}

	}
	if (rw == 'W') {
		if (size == '1'){
			if (address == 'I'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_ID,value);
					zgb_PrintString("ID->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_ID));
					zgb_PrintString("\n");
				}
			}
			if (address == 'T'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_HIGHEST_LIMIT_TEMPERATURE,value);
					zgb_PrintString("HIGHEST_LIMIT_TEMPERATURE->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_HIGHEST_LIMIT_TEMPERATURE));
					zgb_PrintString("\n");
				}
			}
			if (address == 'R'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_RETURN_DELAY_TIME,value);
					zgb_PrintString("RETURN_DELAY_TIME->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_RETURN_DELAY_TIME));
					zgb_PrintString("\n");
				}
			}
			if (address == 'S'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_STATUS_RETURN_LEVEL,value);
					zgb_PrintString("STATUS_RETURN_LEVEL->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_STATUS_RETURN_LEVEL));
					zgb_PrintString("\n");
				}
			}
			if (address == 'Q'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_TORQUE_ENABLE,value);
					zgb_PrintString("TORQUE_ENABLE->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_TORQUE_ENABLE));
					zgb_PrintString("\n");
				}
			}
			if (address == 'W'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_CW_COMPLIANCE_MARGIN,value);
					zgb_PrintString("CW_COMPLIANCE_MARGIN->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_CW_COMPLIANCE_MARGIN));
					zgb_PrintString("\n");
				}
			}
			if (address == 'X'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_CCW_COMPLIANCE_MARGIN,value);
					zgb_PrintString("CCW_COMPLIANCE_MARGIN->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_CCW_COMPLIANCE_MARGIN));
					zgb_PrintString("\n");
				}
			}
			if (address == 'Y'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_CW_COMPLIANCE_SLOPE,value);
					zgb_PrintString("CW_COMPLIANCE_SLOPE->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_CW_COMPLIANCE_SLOPE));
					zgb_PrintString("\n");
				}
			}
			if (address == 'Z'){
				int value = zgb_Pick_Byte_from_PC();
				if (sizeControl){
					dxl_write_byte(idw,AXM_CCW_COMPLIANCE_SLOPE,value);
					zgb_PrintString("CCW_COMPLIANCE_SLOPE->");
					zgb_TxD_Dec_U8(dxl_read_byte(idw,AXM_CCW_COMPLIANCE_SLOPE));
					zgb_PrintString("\n");
				}
			}
		}
		if (size == '2'){
			if (address == 'M'){
				u16 value = zgb_Pick_Word_from_PC();
				if (sizeControl){
					dxl_write_word(idw,AXM_MOVING_SPEED_L,value);
					zgb_PrintString("MOVING_SPEED->");
					zgb_TxD_Dec_U16(dxl_read_word(idw,AXM_MOVING_SPEED_L));
					zgb_PrintString("\n");
				}
			}
			if (address == 'Q'){
				u16 value = zgb_Pick_Word_from_PC();
				if (sizeControl){
					dxl_write_word(idw,AXM_MAX_TORQUE_L,value);
					zgb_PrintString("MAX_TORQUE->");
					zgb_TxD_Dec_U16(dxl_read_word(idw,AXM_MAX_TORQUE_L));
					zgb_PrintString("\n");
				}
			}
			if (address == 'P'){
				u16 value = zgb_Pick_Word_from_PC();
				if (sizeControl){
					dxl_write_word(idw,AXM_PUNCH_L,value);
					zgb_PrintString("PUNCH->");
					zgb_TxD_Dec_U16(dxl_read_word(idw,AXM_PUNCH_L));
					zgb_PrintString("\n");
				}
			}
			if (address == 'G'){
				u16 value = zgb_Pick_Word_from_PC();
				if (sizeControl){
					dxl_write_word(idw,AXM_GOAL_POSITION_L,value);
					zgb_PrintString("GOAL_POSITION->");
					zgb_TxD_Dec_U16(dxl_read_word(idw,AXM_GOAL_POSITION_L));
					zgb_PrintString("\n");
				}
			}
		}
	}

}


