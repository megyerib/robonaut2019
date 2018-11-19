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

#include <stdbool.h>

// Defines -------------------------------------------------------------------------------------------------------------

#define BCM_LOG_LENGHT_SHARP_DISTANCE				5
#define BCM_LOG_LENGHT_SHARP_COLLISION_WARNING		1
#define BCM_LOG_LENGHT_SERVO_ANGLE					6


#define BCM_LOG_SIZE								(	BCM_LOG_LENGHT_SHARP_DISTANCE +				\
														BCM_LOG_LENGHT_SHARP_COLLISION_WARNING +	\
														BCM_LOG_LENGHT_SERVO_ANGLE)


// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	BCM_LOG_SHARP_DISTANCE 			= 0,
	BCM_LOG_SHARP_COLLISION_WARNING,
	BCM_LOG_SERVO_ANGLE
} eBluetoothLogMember;

typedef struct
{
	uint8_t sharpDistant[BCM_LOG_LENGHT_SHARP_DISTANCE];						//! distance in mm
	uint8_t sharpCollisionVarning[BCM_LOG_LENGHT_SHARP_COLLISION_WARNING];		//! y or n
	uint8_t servoAngle[BCM_LOG_LENGHT_SERVO_ANGLE];								//! rad
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

void bcmSend (uint8_t* const txBuffer, const uint16_t length);

void bcmReceive (uint8_t* const rxBuffer, const uint16_t length);

bool bcmTryConnectToCar (void);

void bcmResetBluetooth (void);

bool bcmTraceSharpDistance (uint32_t dist);

bool bcmTraceSharpCollisionWarning (bool collision);

bool bcmTraceServoAngle (double theta);

void bcmBtBufferFlush (void);
