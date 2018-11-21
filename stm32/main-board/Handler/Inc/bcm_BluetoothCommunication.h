////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      bcm_BluetoothCommunication.h
//!  \brief    
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Includes ------------------------------------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "semphr.h"

#include "hndlCommon.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define BCM_LOG_LENGHT_SHARP_DISTANCE				5
#define BCM_LOG_LENGHT_SHARP_COLLISION_WARNING		1
#define BCM_LOG_LENGHT_SERVO_ANGLE					5
#define BCM_LOG_LENGHT_INERT_ACCEL_X				3
#define BCM_LOG_LENGHT_INERT_ACCEL_Y				3
#define BCM_LOG_LENGHT_INERT_ACCEL_Z				3
#define BCM_LOG_LENGHT_INERT_ANG_VEL_X				3
#define BCM_LOG_LENGHT_INERT_ANG_VEL_Y				3
#define BCM_LOG_LENGHT_INERT_ANG_VEL_Z				3
#define BCM_LOG_LENGHT_NAVI_N						7
#define BCM_LOG_LENGHT_NAVI_E						7
#define BCM_LOG_LENGHT_NAVI_THETA					6
#define BCM_LOG_LENGHT_ENC_VEL						4
#define BCM_LOG_LENGHT_TOF_1_DISTANCE				5
#define BCM_LOG_LENGHT_TOF_2_DISTANCE				5
#define BCM_LOG_LENGHT_TOF_3_DISTANCE				5
#define BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT			3
#define BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT				3
#define BCM_LOG_LENGHT_MTR_CURR						4
#define BCM_LOG_LENGHT_MTR_SYS_CURR					4
#define BCM_LOG_LENGHT_MTR_SRV_CURR					4
#define BCM_LOG_LENGHT_CTRL_MTR_CURR				4
#define BCM_LOG_LENGHT_CMD_STOP_ENGINE				1



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
														BCM_LOG_LENGHT_CTRL_MTR_CURR +				\
														BCM_LOG_LENGHT_CMD_STOP_ENGINE				\
													)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	BCM_LOG_SHARP_DISTANCE 			= 0,
	BCM_LOG_SHARP_COLLISION_WARNING,
	BCM_LOG_SERVO_ANGLE,
	BCM_LOG_INERT_ACCEL_X,
	BCM_LOG_INERT_ACCEL_Y,
	BCM_LOG_INERT_ACCEL_Z,
	BCM_LOG_INERT_ANG_VEL_X,
	BCM_LOG_INERT_ANG_VEL_Y,
	BCM_LOG_INERT_ANG_VEL_Z,
	BCM_LOG_NAVI_N,
	BCM_LOG_NAVI_E,
	BCM_LOG_NAVI_THETA,
	BCM_LOG_ENC_VEL,
	BCM_LOG_TOF_1_DISTANCE,
	BCM_LOG_TOF_2_DISTANCE,
	BCM_LOG_TOF_3_DISTANCE,
	BCM_LOG_MTR_MAIN_BAT_VOLT,
	BCM_LOG_MTR_SEC_BAT_VOLT,
	BCM_LOG_MTR_CURR,
	BCM_LOG_MTR_SYS_CURR,
	BCM_LOG_MTR_SRV_CURR,
	BCM_LOG_CTR_MTR_CURR,
	BCM_LOG_CMD_STOP_ENGINE
} eBluetoothLogMember;

typedef struct
{
	uint8_t sharpDistant[BCM_LOG_LENGHT_SHARP_DISTANCE];						//! distance in mm
	uint8_t sharpCollisionWarning[BCM_LOG_LENGHT_SHARP_COLLISION_WARNING];		//! y or n
	uint8_t servoAngle[BCM_LOG_LENGHT_SERVO_ANGLE];								//! rad
	uint8_t inertAccelX[BCM_LOG_LENGHT_INERT_ACCEL_X];
	uint8_t inertAccelY[BCM_LOG_LENGHT_INERT_ACCEL_Y];
	uint8_t inertAccelZ[BCM_LOG_LENGHT_INERT_ACCEL_Z];
	uint8_t inertAngVelX[BCM_LOG_LENGHT_INERT_ANG_VEL_X];
	uint8_t inertAngVelY[BCM_LOG_LENGHT_INERT_ANG_VEL_Y];
	uint8_t inertAngVelZ[BCM_LOG_LENGHT_INERT_ANG_VEL_Z];
	uint8_t naviN[BCM_LOG_LENGHT_NAVI_E];
	uint8_t naviE[BCM_LOG_LENGHT_NAVI_E];
	uint8_t naviTheta[BCM_LOG_LENGHT_NAVI_THETA];
	uint8_t encVel[BCM_LOG_LENGHT_ENC_VEL];
	uint8_t tof1Distance[BCM_LOG_LENGHT_TOF_1_DISTANCE];
	uint8_t tof2Distance[BCM_LOG_LENGHT_TOF_2_DISTANCE];
	uint8_t tof3Distance[BCM_LOG_LENGHT_TOF_3_DISTANCE];
	uint8_t mtrMainBatVolt[BCM_LOG_LENGHT_MTR_MAIN_BAT_VOLT];
	uint8_t mtrSecBatVolt[BCM_LOG_LENGHT_MTR_SEC_BAT_VOLT];
	uint8_t mtrCurr[BCM_LOG_LENGHT_MTR_CURR];
	uint8_t mtrSysCurr[BCM_LOG_LENGHT_MTR_SYS_CURR];
	uint8_t mtrSrvCurr[BCM_LOG_LENGHT_MTR_SRV_CURR];
	uint8_t ctrMtrCurr[BCM_LOG_LENGHT_CTRL_MTR_CURR];
	uint8_t cmdStopEngine[BCM_LOG_LENGHT_CMD_STOP_ENGINE];

} cBluetoothLog;

typedef struct
{
	uint8_t mac[12];
	char name[100];
} cBluetoothDevice;

// Variables -----------------------------------------------------------------------------------------------------------

extern SemaphoreHandle_t semBcm;

cBluetoothDevice remoteDevice;

bool connected;

// Function prototypes -------------------------------------------------------------------------------------------------

void bcmInit (void);

void bcmResetBluetooth (void);

bool bcmTryConnectToCar (void);

void bcmBtBufferFlush (void);

bool bcmLogMemberUpdate (const eBluetoothLogMember member, uint8_t* const array, const uint32_t len);

void bcmSend (uint8_t* const txBuffer, const uint16_t length);

void bcmReceive (uint8_t* const rxBuffer, const uint16_t length);

