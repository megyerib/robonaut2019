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
#define BCM_LOG_LENGHT_SHARP_DISTANCE				5	//!< 1.
#define BCM_LOG_LENGHT_SHARP_COLLISION_WARNING		1   //!<
#define BCM_LOG_LENGHT_SERVO_ANGLE					5   //!<
#define BCM_LOG_LENGHT_INERT_ACCEL_X				4	//!< can be negative
#define BCM_LOG_LENGHT_INERT_ACCEL_Y				4	//!< can be negative
#define BCM_LOG_LENGHT_INERT_ACCEL_Z				4	//!< can be negative
#define BCM_LOG_LENGHT_INERT_ANG_VEL_X				4	//!< can be negative
#define BCM_LOG_LENGHT_INERT_ANG_VEL_Y				4	//!< can be negative
#define BCM_LOG_LENGHT_INERT_ANG_VEL_Z				4	//!< can be negative
#define BCM_LOG_LENGHT_NAVI_N						5	//!< 10. can be negative
#define BCM_LOG_LENGHT_NAVI_E						5	//!< can be negative
#define BCM_LOG_LENGHT_NAVI_THETA					5   //!<
#define BCM_LOG_LENGHT_ENC_VEL						5	//!< can be negative
#define BCM_LOG_LENGHT_TOF_1_DISTANCE				5   //!<
#define BCM_LOG_LENGHT_TOF_2_DISTANCE				5   //!<
#define BCM_LOG_LENGHT_TOF_3_DISTANCE				5   //!<
#define BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT			5   //!<
#define BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT				5   //!<
#define BCM_LOG_LENGHT_MTR_CURR						9   //!<
#define BCM_LOG_LENGHT_MTR_SYS_CURR					4	//!< 20.
#define BCM_LOG_LENGHT_MTR_SRV_CURR					4   //!<
#define BCM_LOG_LENGHT_MTR_CMD_STOP_ENGINE			1   //!<
#define BCM_LOG_LENGHT_CTRL_MTR_CURR				6   //!<
#define BCM_LOG_LENGHT_LINE_D						3   //!<
#define BCM_LOG_LENGHT_LINE_THETA					3   //!<



//! Calculate the size of the buffer that can hold all of the data.
#define BCM_LOG_SIZE								(	BCM_LOG_LENGHT_SHARP_DISTANCE +				\
														BCM_LOG_LENGHT_SHARP_COLLISION_WARNING +	\
														BCM_LOG_LENGHT_SERVO_ANGLE +				\
														BCM_LOG_LENGHT_INERT_ACCEL_X +				\
														BCM_LOG_LENGHT_INERT_ACCEL_Y +				\
														BCM_LOG_LENGHT_INERT_ACCEL_Z +				\
														BCM_LOG_LENGHT_INERT_ANG_VEL_X +			\
														BCM_LOG_LENGHT_INERT_ANG_VEL_Y +			\
														BCM_LOG_LENGHT_INERT_ANG_VEL_Z +			\
														BCM_LOG_LENGHT_NAVI_N +						\
														BCM_LOG_LENGHT_NAVI_E +						\
														BCM_LOG_LENGHT_NAVI_THETA +					\
														BCM_LOG_LENGHT_ENC_VEL +					\
														BCM_LOG_LENGHT_TOF_1_DISTANCE +				\
														BCM_LOG_LENGHT_TOF_2_DISTANCE +				\
														BCM_LOG_LENGHT_TOF_3_DISTANCE +				\
														BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT +			\
														BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT +			\
														BCM_LOG_LENGHT_MTR_CURR +					\
														BCM_LOG_LENGHT_MTR_SYS_CURR +				\
														BCM_LOG_LENGHT_MTR_SRV_CURR +				\
														BCM_LOG_LENGHT_MTR_CMD_STOP_ENGINE +		\
														BCM_LOG_LENGHT_CTRL_MTR_CURR +				\
														BCM_LOG_LENGHT_LINE_D +						\
														BCM_LOG_LENGHT_LINE_THETA					\
													)

// Typedefs ------------------------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!	@brief	Enum for the bluetooth trace data by the sources.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum
{
	BCM_LOG_SHARP_DISTANCE 			= 0,	// 1
	BCM_LOG_SHARP_COLLISION_WARNING,
	BCM_LOG_SERVO_ANGLE,
	BCM_LOG_INERT_ACCEL_X,
	BCM_LOG_INERT_ACCEL_Y,
	BCM_LOG_INERT_ACCEL_Z,
	BCM_LOG_INERT_ANG_VEL_X,
	BCM_LOG_INERT_ANG_VEL_Y,
	BCM_LOG_INERT_ANG_VEL_Z,
	BCM_LOG_NAVI_N,							// 10
	BCM_LOG_NAVI_E,
	BCM_LOG_NAVI_THETA,
	BCM_LOG_ENC_VEL,
	BCM_LOG_TOF_1_DISTANCE,
	BCM_LOG_TOF_2_DISTANCE,
	BCM_LOG_TOF_3_DISTANCE,
	BCM_LOG_MTR_MAIN_BAT_VOLT,
	BCM_LOG_MTR_SEC_BAT_VOLT,
	BCM_LOG_MTR_CURR,
	BCM_LOG_MTR_SYS_CURR,					// 20
	BCM_LOG_MTR_SRV_CURR,
	BCM_LOG_MTR_CMD_STOP_ENGINE,
	BCM_LOG_CTR_MTR_CURR,
	BCM_LOG_LINE_D,
	BCM_LOG_LINE_THETA
} eBluetoothLogMember;

//!	@brief	Structure that hold all of the data of the log trace that needs to be sent out.
typedef struct
{
	uint8_t sharpDistant[BCM_LOG_LENGHT_SHARP_DISTANCE];						//!< distance in cm
	uint8_t sharpCollisionWarning[BCM_LOG_LENGHT_SHARP_COLLISION_WARNING];		//!< status flag
	uint8_t servoAngle[BCM_LOG_LENGHT_SERVO_ANGLE];								//!< angle in rad
	uint8_t inertAccelX[BCM_LOG_LENGHT_INERT_ACCEL_X];							//!< acceleration
	uint8_t inertAccelY[BCM_LOG_LENGHT_INERT_ACCEL_Y];							//!< acceleration
	uint8_t inertAccelZ[BCM_LOG_LENGHT_INERT_ACCEL_Z];							//!< acceleration
	uint8_t inertAngVelX[BCM_LOG_LENGHT_INERT_ANG_VEL_X];	                    //!< angular acceleration
	uint8_t inertAngVelY[BCM_LOG_LENGHT_INERT_ANG_VEL_Y];                       //!< angular acceleration
	uint8_t inertAngVelZ[BCM_LOG_LENGHT_INERT_ANG_VEL_Z];                       //!< angular acceleration
	uint8_t naviN[BCM_LOG_LENGHT_NAVI_E];										//!< coordinate
	uint8_t naviE[BCM_LOG_LENGHT_NAVI_E];										//!< coordinate
	uint8_t naviTheta[BCM_LOG_LENGHT_NAVI_THETA];								//!< angle
	uint8_t encVel[BCM_LOG_LENGHT_ENC_VEL];										//!< velocity
	uint8_t tof1Distance[BCM_LOG_LENGHT_TOF_1_DISTANCE];                        //!< distance
	uint8_t tof2Distance[BCM_LOG_LENGHT_TOF_2_DISTANCE];                        //!< distance
	uint8_t tof3Distance[BCM_LOG_LENGHT_TOF_3_DISTANCE];                        //!< distance
	uint8_t mtrMainBatVolt[BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT];					//!< voltage in V
	uint8_t mtrSecBatVolt[BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT];						//!< voltage in V
	uint8_t mtrCurr[BCM_LOG_LENGHT_MTR_CURR];									//!< current
	uint8_t mtrSysCurr[BCM_LOG_LENGHT_MTR_SYS_CURR];							//!< current in mA
	uint8_t mtrSrvCurr[BCM_LOG_LENGHT_MTR_SRV_CURR];							//!< current in mA
	uint8_t mtrCmdStopEngine[BCM_LOG_LENGHT_MTR_CMD_STOP_ENGINE];				//!< command flag
	uint8_t ctrMtrCurr[BCM_LOG_LENGHT_CTRL_MTR_CURR];							//!< current
	uint8_t lineD[BCM_LOG_LENGHT_LINE_D];										//!< ? in mm
	uint8_t lineTheta[BCM_LOG_LENGHT_LINE_THETA];								//!< angle in deg
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

