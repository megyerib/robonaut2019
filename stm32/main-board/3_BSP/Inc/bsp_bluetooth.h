////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bcm_BluetoothCommunication.h
//!  \brief    
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "handler_common.h"
#include "FreeRTOS.h"
#include "semphr.h"


// Defines -------------------------------------------------------------------------------------------------------------

//!	Defines for the (byte) length of the individual trace objects. Example: 123 ->  Length = 3.
#define BT_LOG_LEN_NAVI_N				/* 1 */		5	//!<
#define BT_LOG_LEN_NAVI_E							5	//!<
#define BT_LOG_LEN_NAVI_PSI							5   //!<
#define BT_LOG_LEN_ENC_V							4   //!<
#define BT_LOG_LEN_DIST_TOF_1						4	//!<
#define BT_LOG_LEN_DIST_TOF_2						4	//!<
#define BT_LOG_LEN_DIST_TOF_3						4	//!<
#define BT_LOG_LEN_DIST_SHARP_1						4	//!<
#define BT_LOG_LEN_INERT_ACCEL_X					6	//!<
#define BT_LOG_LEN_INERT_ACCEL_Y		/* 10 */	6	//!<
#define BT_LOG_LEN_INERT_ACCEL_Z					6	//!<
#define BT_LOG_LEN_INERT_ANG_VEL_X					6	//!<
#define BT_LOG_LEN_INERT_ANG_VEL_Y					6   //!<
#define BT_LOG_LEN_INERT_ANG_VEL_Z					6	//!<
#define BT_LOG_LEN_STEER_WHEEL_ANGLE				5   //!<
#define BT_LOG_LEN_SERVO_ANGLE			/* 16 */	5   //!<

#define BT_LOG_LEN_MTR_MAIN_BAT_VOLT	/* 17 */	5   //!<
#define BT_LOG_LEN_MTR_SEC_BAT_VOLT					5   //!<
#define BT_LOG_LEN_MTR_MOTOR_CURR					9   //!<
#define BT_LOG_LEN_MTR_SYS_CURR						4   //!<
#define BT_LOG_LEN_MTR_SERVO_CURR		/* 21 */	4	//!<

#define BT_LOG_LEN_LINE_LINE_NBR		/* 22 */	2   //!<
#define BT_LOG_LEN_LINE_MAIN_LINE_POS				6   //!<
#define BT_LOG_LEN_LINE_SEC_LINE_POS	/* 24 */	6   //!<

#define BT_LOG_LEN_MAZE_MAIN_SM			/* 25 */	1
#define BT_LOG_LEN_MAZE_GET_KP						5
#define BT_LOG_LEN_MAZE_GET_KD						5
#define BT_LOG_LEN_MAZE_GET_SPEED					2
#define BT_LOG_LEN_MAZE_SEGENTS						12
#define BT_LOG_LEN_MAZE_ACT_STATE		/* 30 */	1
#define BT_LOG_LEN_MAZE_ACT_KP						5
#define BT_LOG_LEN_MAZE_ACT_KD						5
#define BT_LOG_LEN_MAZE_ACT_SPEED					2
#define BT_LOG_LEN_MAZE_INCLIN_SEGMENT	/* 34 */	2

#define BT_LOG_LEN_SRUN_MAIN_SM			/* 35 */	2
#define BT_LOG_LEN_SRUN_ACT_STATE					2
#define BT_LOG_LEN_SRUN_ACT_P						6
#define BT_LOG_LEN_SRUN_ACT_KP						5
#define BT_LOG_LEN_SRUN_ACT_KD						5
#define BT_LOG_LEN_SRUN_ACT_SPEED		/* 40 */	2
#define BT_LOG_LEN_SRUN_GET_P						6
#define BT_LOG_LEN_SRUN_GET_KP						5
#define BT_LOG_LEN_SRUN_GET_KD						5
#define BT_LOG_LEN_SRUN_GET_SPEED		/* 44 */	2

//! Calculate the size of the buffer that can hold all of the data.
#define BCM_LOG_SIZE								(	BT_LOG_LEN_NAVI_N +						\
														BT_LOG_LEN_NAVI_E +						\
														BT_LOG_LEN_NAVI_PSI +					\
														BT_LOG_LEN_ENC_V +						\
														BT_LOG_LEN_DIST_TOF_1 +					\
														BT_LOG_LEN_DIST_TOF_2 +					\
														BT_LOG_LEN_DIST_TOF_3 +					\
														BT_LOG_LEN_DIST_SHARP_1 +				\
														BT_LOG_LEN_INERT_ACCEL_X +				\
														BT_LOG_LEN_INERT_ACCEL_Y +				\
														BT_LOG_LEN_INERT_ACCEL_Z +				\
														BT_LOG_LEN_INERT_ANG_VEL_X +			\
														BT_LOG_LEN_INERT_ANG_VEL_Y +			\
														BT_LOG_LEN_INERT_ANG_VEL_Z +			\
														BT_LOG_LEN_STEER_WHEEL_ANGLE +			\
														BT_LOG_LEN_SERVO_ANGLE +				\
														BT_LOG_LEN_MTR_MAIN_BAT_VOLT +			\
														BT_LOG_LEN_MTR_SEC_BAT_VOLT +			\
														BT_LOG_LEN_MTR_MOTOR_CURR +				\
														BT_LOG_LEN_MTR_SYS_CURR +				\
														BT_LOG_LEN_MTR_SERVO_CURR +				\
														BT_LOG_LEN_LINE_LINE_NBR +				\
														BT_LOG_LEN_LINE_MAIN_LINE_POS +			\
														BT_LOG_LEN_LINE_SEC_LINE_POS +			\
														BT_LOG_LEN_MAZE_MAIN_SM +				\
														BT_LOG_LEN_MAZE_GET_KP +				\
														BT_LOG_LEN_MAZE_GET_KD +				\
														BT_LOG_LEN_MAZE_GET_SPEED +				\
														BT_LOG_LEN_MAZE_SEGENTS +				\
														BT_LOG_LEN_MAZE_ACT_STATE +				\
														BT_LOG_LEN_MAZE_ACT_KP +				\
														BT_LOG_LEN_MAZE_ACT_KD +				\
														BT_LOG_LEN_MAZE_ACT_SPEED +				\
														BT_LOG_LEN_MAZE_INCLIN_SEGMENT +		\
														BT_LOG_LEN_SRUN_MAIN_SM +				\
														BT_LOG_LEN_SRUN_ACT_STATE +				\
														BT_LOG_LEN_SRUN_ACT_P +					\
														BT_LOG_LEN_SRUN_ACT_KP +				\
														BT_LOG_LEN_SRUN_ACT_KD					\
														BT_LOG_LEN_SRUN_ACT_SPEED +				\
														BT_LOG_LEN_SRUN_GET_P +					\
														BT_LOG_LEN_SRUN_GET_KP +				\
														BT_LOG_LEN_SRUN_GET_KD +				\
														BT_LOG_LEN_SRUN_GET_SPEED				\
													)

// Typedefs ------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!	@brief	Enum for the bluetooth trace data by the sources.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum
{
	BT_LOG_NAVI_N 			= 0,	// 1
	BT_LOG_NAVI_E,				//!<
	BT_LOG_NAVI_PSI,            //!<
	BT_LOG_ENC_V,          //!<
	BT_LOG_DIST_TOF_1,          //!<
	BT_LOG_DIST_TOF_2,          //!<
	BT_LOG_DIST_TOF_3,        //!<
	BT_LOG_DIST_SHARP_1,        //!<
	BT_LOG_INERT_ACCEL_X,        //!<
	BT_LOG_INERT_ACCEL_Y,							// 10
	BT_LOG_INERT_ACCEL_Z,                 //!<
	BT_LOG_INERT_ANG_VEL_X,             //!<
	BT_LOG_INERT_ANG_VEL_Y,                //!<
	BT_LOG_INERT_ANG_VEL_Z,         //!<
	BT_LOG_STEER_WHEEL_ANGLE,         //!<
	BT_LOG_SERVO_ANGLE,         //!<

	BT_LOG_MTR_MAIN_BAT_VOLT,      //!<
	BT_LOG_MTR__SEC_BAT_VOLT,       //!<
	BT_LOG_MTR_MOTOR_CURR,               //!<
	BT_LOG_MTR__SYS_CURR,					// 20
	BT_LOG_MTR__SREVO_CURR,           //!<

	BT_LOG_LINE_LINE_NBR,    //!<
	BT_LOG_LINE_MAIN_LINE_POS,           //!<
	BT_LOG_LINE_SEC_LINE_POS,                 //!<

	BT_LOG_MAZE_MAIN_SM,
	BT_LOG_MAZE_GET_KP,
	BT_LOG_MAZE_GET_KD,
	BT_LOG_MAZE_GET_SPEED,
	BT_LOG_MAZE_SEGMENTS,
	BT_LOG_MAZE_ACT_STATE,
	BT_LOG_MAZE_ACT_KP,
	BT_LOG_MAZE_ACT_KD,
	BT_LOG_MAZE_ACT_SPEED,
	BT_LOG_MAZE_INCLIN_SEGMENT,

	BT_LOG_SRUN_

} eBluetoothLogMember;

//!	@brief	Structure that hold all of the data of the log trace that needs to be sent out.
typedef struct
{
	uint8_t naviN[BT_LOG_LEN_NAVI_N];										//!<
	uint8_t naviE[BT_LOG_LEN_NAVI_E];										//!<
	uint8_t naviPsi[BT_LOG_LEN_NAVI_PSI];									//!<
	uint8_t encV[BT_LOG_LEN_ENC_V];											//!<
	uint8_t distToF1[BT_LOG_LEN_DIST_TOF_1];								//!<
	uint8_t distToF2[BT_LOG_LEN_DIST_TOF_2];								//!<
	uint8_t distToF3[BT_LOG_LEN_DIST_TOF_3];	                   			//!<
	uint8_t distSharp1[BT_LOG_LEN_DIST_SHARP_1];                    	    //!<
	uint8_t inertAccX[BT_LOG_LEN_INERT_ACCEL_X];                       		//!<
	uint8_t inertAccY[BT_LOG_LEN_INERT_ACCEL_Y];							//!<
	uint8_t inertAccZ[BT_LOG_LEN_INERT_ACCEL_Z];							//!<
	uint8_t inertAngVelX[BT_LOG_LEN_INERT_ANG_VEL_X];						//!<
	uint8_t inertAngVelY[BT_LOG_LEN_INERT_ANG_VEL_Y];						//!<
	uint8_t inertAngVelZ[BT_LOG_LEN_INERT_ANG_VEL_Z];                       //!<
	uint8_t steerWheelAngle[BT_LOG_LEN_STEER_WHEEL_ANGLE];                  //!<
	uint8_t servoAngle[BT_LOG_LEN_SERVO_ANGLE];                        		//!<

	uint8_t mtrMainBatVolt[BT_LOG_LEN_MTR_MAIN_BAT_VOLT];					//!<
	uint8_t mtrSecBatVolt[BT_LOG_LEN_MTR_SEC_BAT_VOLT];						//!<
	uint8_t mtrCurr[BT_LOG_LEN_MTR_MOTOR_CURR];								//!<
	uint8_t mtrSysCurr[BT_LOG_LEN_MTR_SYS_CURR];							//!<
	uint8_t mtrSrvCurr[BT_LOG_LEN_MTR_SERVO_CURR];							//!<

	uint8_t lineLineNbr[BT_LOG_LEN_LINE_LINE_NBR];							//!<
	uint8_t lineMainLinePos[BT_LOG_LEN_LINE_MAIN_LINE_POS];					//!<
	uint8_t lineSecLinePos[BT_LOG_LEN_LINE_SEC_LINE_POS];					//!<

	uint8_t mazeMainSM[BT_LOG_LEN_MAZE_MAIN_SM];							//!<
	uint8_t mazeGetKp[BT_LOG_LEN_MAZE_GET_KP];								//!<
	uint8_t mazeGetKd[BT_LOG_LEN_MAZE_GET_KD];								//!<
	uint8_t mazeGetSpeed[BT_LOG_LEN_MAZE_GET_SPEED];						//!<
	uint8_t mazeSegments[BT_LOG_LEN_MAZE_SEGENTS];							//!<
	uint8_t mazeActState[BT_LOG_LEN_MAZE_ACT_STATE];						//!<
	uint8_t mazeActKp[BT_LOG_LEN_MAZE_ACT_KP];								//!<
	uint8_t mazeActKd[BT_LOG_LEN_MAZE_ACT_KD];								//!<
	uint8_t mazeActSpeed[BT_LOG_LEN_MAZE_ACT_SPEED];						//!<
	uint8_t mazeInclinSegment[BT_LOG_LEN_MAZE_INCLIN_SEGMENT];				//!<

	uint8_t sRunMainSm[BT_LOG_LEN_SRUN_MAIN_SM];							//!<
	uint8_t sRunActState[BT_LOG_LEN_SRUN_ACT_STATE];						//!<
	uint8_t sRunActP[BT_LOG_LEN_SRUN_ACT_P];								//!<
	uint8_t sRunActKp[BT_LOG_LEN_SRUN_ACT_KP];								//!<
	uint8_t sRunActKd[BT_LOG_LEN_SRUN_ACT_KD];								//!<
	uint8_t sRunActSpeed[BT_LOG_LEN_SRUN_ACT_SPEED];						//!<
	uint8_t sRunGetP[BT_LOG_LEN_SRUN_GET_P];								//!<
	uint8_t sRunGetKp[BT_LOG_LEN_SRUN_GET_KP];								//!<
	uint8_t sRunGetKd[BT_LOG_LEN_SRUN_GET_KD];								//!<
	uint8_t sRunGetSpeed[BT_LOG_LEN_SRUN_GET_SPEED];						//!<
} cBluetoothLog;

//TODO IDEA: trace the connection informations
typedef struct
{
	uint8_t mac[12];
	char name[100];
} cBluetoothDevice;
//END_IDEA

// Variables -----------------------------------------------------------------------------------------------------------

//TODO IDEA: trace the connection informations
cBluetoothDevice remoteDevice;
//END_IDEA

// flag
bool connected;

// Function prototypes -------------------------------------------------------------------------------------------------

//! Initializes the bluetooth communication module.
void bspBluetoothInit (void);

//! Checks the bluetooth connection.
//!
//! @return			True if connected, False if not connected
bool bspBluetoothConnected (void);

//!	Resets the bluetooth module.
void bspResetBluetooth (void);

//! Builds up a connection with a given bluetooth device with this (car) board.
//!
//! @return			True if successfully connected, False if could not connet
bool bspTryConnectToCar (void);

//!	Send out the log structure through Bluetooth UART.
void bspBtBufferFlush (void);

//!	Stores a given uint8_t array to the bluetotth log structure.
//!
//! @param member	enum of a given log slot
//! @param array	array that holds the characters that has to be stored
//! @param len		how many characters has to be saved
//! @return			True if successfully stored, False if could not store
bool bspLogMemberUpdate (const eBluetoothLogMember member, uint8_t* const array, const uint32_t len);

//! Sends out a uint8_t array through Bluetooth with interrupt.
//!
//! @param txBuffer	characters will be sent out
//! @param length	how many characters will be sent out
void bspBtSend (uint8_t* const txBuffer, const uint16_t length);

//! Receives a given number of characters and stores in an array.
//!
//! @param rxBuffer	array that will store the characters
//! @param length	how many characters will be received
void bspBtReceive (uint8_t* const rxBuffer, const uint16_t length);

