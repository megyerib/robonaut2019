////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_speedRun_StateMachines.c
//!  \brief		This is a submodule that hold the state machines and the controller of the speed run app.
//!  \details	See in app_speedRun_StateMachine.h
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_speedRun.h"
#include "app_speedRun_StateMachines.h"
#include "bsp_sharp.h"
#include "bsp_servo.h"
#include "motor.h"
#include "line.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

//! State of the speed run main state machine.
eSTATE_MAIN smMainState;
//! The lap is divided into a given number of segments, and this many states has a lap state machine.
uint8_t actLapSegment;
//! Actual state of the overtake state machine.
static eSTATE_OVERTAKE overtakeState;

//! Control parameters of the actual state.
cPD_CONTROLLER_PARAMS actualParams;
//! Contain all of the control parameters.
cSRUN_PD_CONTROL_PARAM_LIST paramList;

//! During the Parade lap the car is allowed to try to overtake the safety car.
bool tryToOvertake;
//! Flag that indicates that the car is behind the safety car.
static bool behindSafetyCar;
//! Flag that indicates if the car went through the start gate, which means a new lap is started.
static bool startGateFound;
//! Counter for the individual timing functionalities.
static uint32_t timeCounter;

//! Line position in the actual task run.
static float line_pos;
//! Line position in the previous task run.
static float line_prevPos;
//! Measured distance value in front of the car in the actual task run.
static uint32_t dist_front;
//! Measured distance value in front of the car in the previous task run.
static uint32_t dist_frontPrev;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void sRunCntrKeepDistance (void);

// Global function definitions -----------------------------------------------------------------------------------------

//! Function: sRunInitStateMachines
void sRunInitStateMachines (void)
{
	smMainState = eSTATE_MAIN_READY;
	actLapSegment = 0;
	overtakeState = eSTATE_OVERTAKE_LEAVE_LINE;

	actualParams.P		= 0;
	actualParams.Kp		= 0;
	actualParams.Kd		= 0;
	actualParams.Speed	= 0;

	tryToOvertake 	= false;
	behindSafetyCar = true;
	startGateFound  = false;
	timeCounter 	= 0;

	line_prevPos   = 0;
	line_pos 	   = 0;
	dist_front 	   = 0;
	dist_frontPrev = 0;
}

//! Function: sRunMainStateMachine
void sRunMainStateMachine (void)
{
	switch (smMainState)
	{
		case eSTATE_MAIN_READY:
		{
			// Check the distance change of the safety car.
			dist_frontPrev = dist_front;
			dist_front = sharpGetMeasurement().Distance;

			if (dist_front >= SRUN_FOLLOW_DISTANCE)
			{
				// Trigger: safety car starts.
				smMainState = eSTATE_MAIN_PARADE_LAP;
			}
			break;
		}
		case eSTATE_MAIN_PARADE_LAP:
		{
			sRunParadeLapAlgorithm();
			break;
		}
		case eSTATE_MAIN_OVERTAKING:
		{
			sRunOvertakeStateMachine();
			break;
		}
		case eSTATE_MAIN_LAP_1:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			if (startGateFound == true)
			{
				smMainState = eSTATE_MAIN_LAP_2;
			}
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			if (startGateFound == true)
			{
				smMainState = eSTATE_MAIN_LAP_3;
			}
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			// Drive state machine.
			sRunDriveStateMachine();

			if (startGateFound == true)
			{
				smMainState = eSTATE_MAIN_STOP;
			}
			break;
		}
		case eSTATE_MAIN_STOP:
		{
			// Race is complete, stop the car.
			motorSetDutyCycle(0);
			break;
		}
		default:
		{
			// NOP
			break;
		}
	}
}

//! Function: sRunDriveStateMachine
void sRunDriveStateMachine (void)	// TODO implementation
{
	// Load in control parameters.
	switch (actLapSegment)
	{
		case eSTATE_MAIN_LAP_1:
		{
			actualParams.P 		= paramList.lap1[actLapSegment].P;
			actualParams.Kp 	= paramList.lap1[actLapSegment].Kp;
			actualParams.Kd 	= paramList.lap1[actLapSegment].Kd;
			actualParams.Speed 	= paramList.lap1[actLapSegment].Speed;
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			actualParams.P 		= paramList.lap2[actLapSegment].P;
			actualParams.Kp 	= paramList.lap2[actLapSegment].Kp;
			actualParams.Kd 	= paramList.lap2[actLapSegment].Kd;
			actualParams.Speed 	= paramList.lap2[actLapSegment].Speed;
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			actualParams.P 		= paramList.lap3[actLapSegment].P;
			actualParams.Kp 	= paramList.lap3[actLapSegment].Kp;
			actualParams.Kd 	= paramList.lap3[actLapSegment].Kd;
			actualParams.Speed 	= paramList.lap3[actLapSegment].Speed;
			break;
		}
		default:
		{
			break;
		}
	}

	// Actuate the car
	sRunCntrLineFollow();

	// Search for the end of the segment: 3 continuous lines (slow down segment).
	switch (actLapSegment)
	{
		case 0:
		{
			// Find slowing segment (3 continuous lines).
			if (false)
			{

				actLapSegment = 1;
			}
			break;
		}
		case 1:
		{
			// Find main track (1 line).
			if (false)
			{

				actLapSegment = 2;
			}
			break;
		}
		case 2:
		{
			// Find acceleration segment (3 discrete line segments).
			if (false)
			{

				actLapSegment = 3;
			}
			break;
		}
		case 3:
		{
			// Find main track (1 line).
			if (false)
			{

				actLapSegment = 4;
			}
			break;
		}
		case 4:
		{
			// Find slowing segment (3 continuous lines).
			if (false)
			{

				actLapSegment = 5;
			}
			break;
		}
		case 5:
		{
			// Find main track (1 line).
			if (false)
			{

				actLapSegment = 6;
			}
			break;
		}
		case 6:
		{
			// Find acceleration segment (3 discrete line segments).
			if (false)
			{

				actLapSegment = 7;
			}
			break;
		}
		case 7:
		{
			// Find main track (1 line).
			if (false)
			{

				actLapSegment = 8;
			}
			break;
		}
		case 8:
		{
			// Find slowing segment (3 continuous lines).
			if (false)
			{

				actLapSegment = 9;
			}
			break;
		}
		case 9:
		{
			// Find main track (1 line).
			if (false)
			{

				actLapSegment = 10;
			}
			break;
		}
		case 10:
		{
			// Find acceleration segment (3 discrete line segments).
			if (false)
			{

				actLapSegment = 11;
			}
			break;
		}
		case 11:
		{
			// Find main track (1 line).
			if (false)
			{

				actLapSegment = 12;
			}
			break;
		}
		case 12:
		{
			// Find slowing segment (3 continuous lines).
			if (false)
			{

				actLapSegment = 13;
			}
			break;
		}
		case 13:
		{
			// Find main track (1 line).
			if (false)
			{

				actLapSegment = 14;
			}
			break;
		}
		case 14:
		{
			// Find acceleration segment (3 discrete line segments).
			if (false)
			{

				actLapSegment = 15;
			}
			break;
		}
		case 15:
		{
			// Find start gate.
			if (false)
			{

				// The lap is complete, new one is starting.
				actLapSegment = 0;
			}
			break;
		}
		default:
		{
			// NOP
			break;
		}
	}
}

//! Function: sRunOvertakeStateMachine
void sRunOvertakeStateMachine (void)
{
	// Maneuver
	switch (overtakeState)
	{
		case eSTATE_OVERTAKE_LEAVE_LINE:
		{
			// Slow down.
			motorSetDutyCycle(SRUN_OVERTAKE_SPEED_SLOW);

			// Turn left for a bit
			servoSetAngle(SRUN_OVERTAKE_SERVO_ANGLE);

			// Time is ticking.
			if (timeCounter >= SRUN_OVERTAKE_TURN_TIME)
			{
				// Reset the time counter.
				timeCounter = 0;

				// Get back to parallel.
				overtakeState = eSTATE_OVERTAKE_GET_PARALLEL;
			}
			else
			{
				timeCounter++;
			}
			break;
		}
		case eSTATE_OVERTAKE_GET_PARALLEL:
		{
			// Turn right to get parallel with the line.
			servoSetAngle(-SRUN_OVERTAKE_SERVO_ANGLE);

			// Time is ticking.
			if (timeCounter >= SRUN_OVERTAKE_TURN_TIME)
			{
				// Reset the time counter.
				timeCounter = 0;

				// Pass the safety car.
				overtakeState = eSTATE_OVERTAKE_PASS_SAFETY_CAR;
			}
			else
			{
				timeCounter++;
			}
			break;
		}
		case eSTATE_OVERTAKE_PASS_SAFETY_CAR:
		{
			// Speed up.
			motorSetDutyCycle(paramList.overtaking.Speed);

			// After a given time slow down.
			// Time is ticking.
			if (timeCounter >= SRUN_OVERTAKE_PASS_TIME)
			{
				// Reset the time counter.
				timeCounter = 0;

				// Get back to the track
				overtakeState = eSTATE_OVERTAKE_FIND_LINE;
			}
			else
			{
				timeCounter++;
			}
			break;
		}
		case eSTATE_OVERTAKE_FIND_LINE:
		{
			// Slow down.
			motorSetDutyCycle(SRUN_OVERTAKE_SPEED_SLOW);

			// Turn right.
			servoSetAngle(-SRUN_OVERTAKE_SERVO_ANGLE);

			// Search for the line.
			// Time is ticking.
			if (timeCounter >= SRUN_OVERTAKE_FIND_TIME)
			{
				// Could not find the line, the car is lost.
				overtakeState = eSTATE_OVERTAKE_FAILED;
			}
			else
			{
				timeCounter++;

				// If the line is back, then it was successful. If not stop after a time (no collision).
				if (lineGetRawFront().cnt != 0)
				{
					// The car is back on track.
					overtakeState = eSTATE_OVERTAKE_SUCCESS;
				}
			}
			break;
		}
		case eSTATE_OVERTAKE_SUCCESS:
		{
			// Overtake is finished
			tryToOvertake = false;
			behindSafetyCar = false;

			// Resume the Parade lap.
			smMainState = eSTATE_MAIN_PARADE_LAP;
		}
		case eSTATE_OVERTAKE_FAILED:
		{
			// Stop the car.
			motorSetDutyCycle(0);

			if (lineGetRawFront().cnt != 0)
			{
				// If the line is back continue the Parade Lap.
				// Overtake is finished. The car is still behind the safety car.
				tryToOvertake = false;
				behindSafetyCar = true;

				// Resume the Parade lap.
				smMainState = eSTATE_MAIN_PARADE_LAP;
			}
			break;
		}
		default:
		{
			// NOP
			break;
		}
	}
}

//! Function: sRunParadeLapAlgorithm
void sRunParadeLapAlgorithm (void)
{
	if (behindSafetyCar == true)
	{
		// Load in control parameters.
		actualParams.P 		= paramList.lapParade.P;
		actualParams.Kp 	= paramList.lapParade.Kp;
		actualParams.Kd 	= paramList.lapParade.Kd;
		actualParams.Speed	= paramList.lapParade.Speed;

		// Follow the safety car. WARNING: Keep distance calculates the speed, line follow set the speed.
		sRunCntrKeepDistance();
		sRunCntrLineFollow();

		// In the right place check if we can try overtaking.

		// Try to overtake if it is enabled.
		if (tryToOvertake == true && actLapSegment == SRUN_OVERTAKE_SEGMENT)
		{
			smMainState = eSTATE_MAIN_OVERTAKING;

			// Reset the time counter for the maneuver.
			timeCounter = 0;
		}
		else
		{
			// Follow the safety car until the last corner.

			// Start gate means a new lap.
			if (startGateFound == true)
			{
				smMainState = eSTATE_MAIN_LAP_1;
			}
		}
	}
	else
	{
		// We overtook the safety car!!! :D
		// Speed up. Load in control parameters.
		actualParams.P = paramList.lapParade.P;
		actualParams.Kp = paramList.lapParade.Kp;
		actualParams.Kd = paramList.lapParade.Kd;
		actualParams.Speed = paramList.lapParade.Speed;

		// Follow the track.
		sRunCntrLineFollow();

		// Start gate means a new lap.
		if (startGateFound == true)
		{
			smMainState = eSTATE_MAIN_LAP_1;
		}
	}
}

//! Function: sRunCntrLineFollow
void sRunCntrLineFollow (void)
{
	float line_diff;
	float servo_angle;
	float P_modifier;
	float D_modifier;

	// Detect line.
	line_prevPos = line_pos;
	line_pos = lineGetSingle() / 1000; // m -> mm
	line_diff = line_pos - line_prevPos;

	// Control the servo.
	P_modifier = line_pos  * actualParams.Kp;
	D_modifier = line_diff * actualParams.Kd;
	servo_angle = -0.75f * (P_modifier + D_modifier);

	// Actuate.
	motorSetDutyCycle(actualParams.Speed);
	servoSetAngle(servo_angle);
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//**********************************************************************************************************************
//! Distance P controller.
//!
//! @return -
//**********************************************************************************************************************
static void sRunCntrKeepDistance (void)
{
	float dist_front;
	float dist_diff;

	dist_front = sharpGetMeasurement().Distance;
	dist_diff = SRUN_FOLLOW_DISTANCE - dist_front;

	actualParams.Speed = actualParams.P * dist_diff;
}
