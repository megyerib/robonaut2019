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
#include "speed.h"
#include "app_controllers.h"
#include "app_race_roadsignal.h"
#include "app_common.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define SRUN_SEG_MIN_LEN			(20U)		//!< mm
#define SRUN_SPEED_SUB_SEG_LEN_MAX 	(100U)  	//!< mm

#define SRUN_DIST_KEEP_SPEED_MIN	(0.0f)		//!< m/s
#define SRUN_DIST_KEEP_SPEED_MAX	(3.0f)  	//!< m/s
#define SRUN_DIST_KEEP_KP			(0.03f)
#define SRUN_DIST_SETPOINT			(60u)		//!< cm

// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

extern QueueHandle_t qNaviPSI_f;

eSTATE_MAIN smMainStateSRun;	//!< State of the speed run main state machine.
uint8_t actLapSegment;			//!< The lap is divided into a given number of segments, and this many states has a lap state machine.

cPD_CNTRL_PARAMS sRunActualParams;			//!< Control parameters of the actual state.
cSRUN_PD_CONTROL_PARAM_LIST paramListSRun;	//!< Contain all of the control parameters.

bool tryToOvertake;				//!< During the Parade lap the car is allowed to try to overtake the safety car.

float sRunServoAngle;
float sRunActSpeedDist;

uint32_t sRunActFrontDist;		//! Measured distance value in front of the car in the actual task run.
uint32_t sRunPrevFrontDist;		//! Measured distance value in front of the car in the previous task run.


static bool behindSafetyCar;	//!< Flag that indicates that the car is behind the safety car.
static bool startGateFound;		//!< Flag that indicates if the car went through the start gate, which means a new lap is started.
static uint32_t timeCounter;	//!< Counter for the individual timing functionalities.

static eSTATE_OVERTAKE overtakeState;	//!< Actual state of the overtake state machine.
static bool actLapIsFinished;
static float overtakeStartPoint;
static float overtakeEndPoint;
static float overtakeSpeed;
static float overtakePsi;

static uint8_t segmentTypeCounter;
static uint32_t lineStart;
static uint32_t lineLenght;
static uint32_t lineTimeCounter;
static uint32_t lineTimePoint;
static uint8_t prevSegmentType;
static bool lineNewLine;

static bool lapFinished;

static uint32_t lapSegStart;
static uint32_t lapSegEnd;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static eSEGMENT_TYPE sRunGetSegmentType (void);
static void sRunLoadInParamsToRun (void);
static void sRunSpeedUpParadeLap (void);
static float sRunConvertAngleTo180 (const float angle);

// Global function definitions -----------------------------------------------------------------------------------------

//! Function: sRunInitStateMachines
void sRunInitStateMachines (void)
{
	smMainStateSRun = eSTATE_MAIN_WAIT_BEHIND;
	actLapSegment = 0;
	overtakeState = eSTATE_OVERTAKE_START;
	actLapIsFinished = false;

	sRunActualParams.P		= 0.05;
	sRunActualParams.Kp		= 0.025;
	sRunActualParams.Kd		= 3.5;
	sRunActualParams.Speed	= 18;

	paramListSRun.lapParade.P = 0.05;
	paramListSRun.lapParade.Kp = 0.025;
	paramListSRun.lapParade.Kd = 3.5;
	paramListSRun.lapParade.Speed = 17;

	paramListSRun.overtaking.P = 0;
	paramListSRun.overtaking.Kp = 0.01;
	paramListSRun.overtaking.Kd = 0.5;
	paramListSRun.overtaking.Speed = 30;

	tryToOvertake 	= true;
	behindSafetyCar = true;
	startGateFound  = false;
	timeCounter 	= 0;

	sRunActFrontDist  = 0;
	sRunPrevFrontDist = 0;

	segmentTypeCounter = 0;
	lineTimeCounter = 0;
	prevSegmentType = eSEG_LOST_TRACK;

	sRunCntrlLineFollow_ON();

	lapFinished = false;

	setRaceRs(InitFast);
}

//! Function: sRunMainStateMachine
void sRunMainStateMachine (void)
{
	switch (smMainStateSRun)
	{
		case eSTATE_MAIN_WAIT_BEHIND:
		{
			// Check the distance change of the safety car.
			sRunPrevFrontDist = sRunActFrontDist;
			sRunActFrontDist = sharpGetMeasurement().Distance;

			if (sRunActFrontDist >= SRUN_DIST_SETPOINT)
			{
				// Trigger: safety car starts.
				smMainStateSRun = eSTATE_MAIN_PARADE_LAP;
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
			sRunLoadInParamsToRun();

			// Drive state machine.
			actLapIsFinished = sRunDriveStateMachine();

			if (startGateFound == true || actLapIsFinished)
			{
				smMainStateSRun = eSTATE_MAIN_LAP_2;
			}
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			sRunLoadInParamsToRun();

			// Drive state machine.
			actLapIsFinished = sRunDriveStateMachine();

			if (startGateFound == true || actLapIsFinished)
			{
				smMainStateSRun = eSTATE_MAIN_LAP_3;
			}
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			sRunLoadInParamsToRun();

			// Drive state machine.
			sRunDriveStateMachine();

			if (startGateFound == true || lapFinished)
			{
				smMainStateSRun = eSTATE_MAIN_STOP;
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
bool sRunDriveStateMachine (void)
{
	bool robotInLastSegment = false;
	RACE_RS roadSign;
	float lenght;

	roadSign = getRaceRs();
	lapSegEnd = speedGetDistance();
	lenght = lapSegEnd - lapSegStart;

	switch (actLapSegment)
	{
		case 0:
		{
			lapFinished = false;

			if (roadSign != Fast)
			{
				actLapSegment = 1;
				lapSegStart = speedGetDistance();
			}
			break;
		}
		case 1:
		{
			if (lenght > 1.0f)
			{
				actLapSegment = 2;
			}
			break;
		}
		case 2:
		{
			if (roadSign == Fast)
			{
				actLapSegment = 3;
			}
			break;
		}
		case 3:
		{
			if (roadSign != Fast)
			{
				actLapSegment = 4;
				lapSegStart = speedGetDistance();
			}
			break;
		}
		case 4:
		{
			if (lenght > 2.0f)
			{
				actLapSegment = 5;
				lapSegStart = speedGetDistance();
			}
			break;
		}
		case 5:
		{
			if (lenght > 2.5f)
			{
				actLapSegment = 6;
			}
			break;
		}
		case 6:
		{
			if (roadSign == Fast)
			{
				actLapSegment = 7;
			}
			break;
		}
		case 7:
		{
			if (roadSign != Fast)
			{
				actLapSegment = 8;
				lapSegStart = speedGetDistance();
			}
			break;
		}
		case 8:
		{
			if (lenght > 0.8f)
			{
				actLapSegment = 9;
				lapSegStart = speedGetDistance();
			}
			break;
		}
		case 9:
		{
			if (lenght > 4.5f)
			{
				actLapSegment = 10;
			}
			break;
		}
		case 10:
		{
			if (roadSign == Fast)
			{
				actLapSegment = 11;
			}
			break;
		}
		case 11:
		{
			if (roadSign != Fast)
			{
				actLapSegment = 12;
			}
			break;
		}
		case 12:
		{
			if (roadSign == Fast)
			{
				actLapSegment = 13;
				robotInLastSegment = true;
			}
			break;
		}
		case 13:
		{
			if (roadSign != Fast)
			{
				actLapSegment = 0;
				lapFinished = true;
			}
			break;
		}
		default:
		{
			break;
		}
	}

	return robotInLastSegment;
}

//! Function: sRunOvertakeStateMachine
void sRunOvertakeStateMachine (void)
{
	float lenght;
	float actPsi;

	sRunActualParams.P 		= paramListSRun.overtaking.P;
	sRunActualParams.Kp 	= paramListSRun.overtaking.Kp;
	sRunActualParams.Kd 	= paramListSRun.overtaking.Kd;
	sRunActualParams.Speed	= paramListSRun.overtaking.Speed;

	// Actual distance from start.
	overtakeEndPoint = speedGetDistance();
	// Distance form the start position.
	lenght = overtakeEndPoint - overtakeStartPoint;

	// Maneuver
	switch (overtakeState)
	{
		case eSTATE_OVERTAKE_START:
		{
			overtakeStartPoint = speedGetDistance();
			overtakeSpeed = 0.6f;

			overtakeState = eSTATE_OVERTAKE_DELAY;
			break;
		}
		case eSTATE_OVERTAKE_DELAY:
		{
			if (lenght > 1.0f)
			{
				overtakeStartPoint = speedGetDistance();
				xQueuePeek(qNaviPSI_f, &overtakePsi, 0);

				overtakeState = eSTATE_OVERTAKE_LEAVE_LINE;
			}
			break;
		}
		case eSTATE_OVERTAKE_LEAVE_LINE:
		{
			// Slow down.
			overtakeSpeed = SRUN_OVERTAKE_SPEED_SLOW;
			// Turn off line follow.
			sRunCntrlLineFollow_OFF();

			if (lenght < SRUN_OVERTAKE_DIST_TURN)
			{
				// Turn left for a bit
				sRunServoAngle = SRUN_OVERTAKE_SERVO_ANGLE;
			}
			else
			{
				// Steer back to straight.
				sRunServoAngle = SERVO_MIDDLE_RAD;

				if (lenght > SRUN_OVERTAKE_DIST_FROM_LINE)
				{
					overtakeState = eSTATE_OVERTAKE_GET_PARALLEL;
					overtakeStartPoint = speedGetDistance();
				}
			}
			break;
		}
		case eSTATE_OVERTAKE_GET_PARALLEL:
		{
			// Slow down.
			overtakeSpeed = SRUN_OVERTAKE_SPEED_SLOW;

			if (lenght < SRUN_OVERTAKE_DIST_TURN + 0.05f)
			{
				// Turn right to get parallel with the line.
				sRunServoAngle = -SRUN_OVERTAKE_SERVO_ANGLE;
			}
			else
			{
				sRunServoAngle = SERVO_MIDDLE_RAD;
				overtakeStartPoint = speedGetDistance();


				overtakeState = eSTATE_OVERTAKE_PASS_SAFETY_CAR;
			}
			break;
		}
		case eSTATE_OVERTAKE_PASS_SAFETY_CAR:
		{
			float actAngle;
			float wantedAngle;

			// Speed up.
			overtakeSpeed = SRUN_OVERTAKE_SPEED_FAST;

			sRunServoAngle = SERVO_MIDDLE_RAD;

			if (lenght > SRUN_OVERTAKE_DIST_STRAIGHT - 1.5f)
			{
				overtakeSpeed = 18.0f;
			}

			if (lenght > SRUN_OVERTAKE_DIST_STRAIGHT)
			{
				// Get back to the track
				overtakeState = eSTATE_OVERTAKE_FIND_LINE;

				overtakeStartPoint = speedGetDistance();
			}
			break;
		}
		case eSTATE_OVERTAKE_FIND_LINE:
		{
			// If the line is back, then it was successful. If not stop after a time (no collision).
			if (lineGetRawFront().cnt != 0)
			{
				sRunCntrlLineFollow_ON();
				actLapSegment = SRUN_OVERTAKE_SEGMENT;

				// The car is back on track.
				overtakeState = eSTATE_OVERTAKE_SUCCESS;
			}

			// Slow down.
			overtakeSpeed = SRUN_OVERTAKE_SPEED_SLOW;

			if (lenght < SRUN_OVERTAKE_DIST_TURN)
			{
				sRunServoAngle = SRUN_OVERTAKE_SERVO_RET_ANGLE;
			}
			else if (lenght > SRUN_OVERTAKE_DIST_FROM_LINE)
			{
				sRunServoAngle = SERVO_MIDDLE_RAD;
			}
			break;
		}
		case eSTATE_OVERTAKE_SUCCESS:
		{
			// Overtake is finished
			tryToOvertake = false;
			behindSafetyCar = false;

			actLapSegment = SRUN_OVERTAKE_SEGMENT;
			smMainStateSRun = eSTATE_MAIN_PARADE_LAP;
			break;
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
				smMainStateSRun = eSTATE_MAIN_PARADE_LAP;
			}
			break;
		}
		default:
		{
			// NOP
			break;
		}
	}

	paramListSRun.overtaking.Speed = overtakeSpeed * 10;
}

//! Function: sRunParadeLapAlgorithm
void sRunParadeLapAlgorithm (void)
{
	bool lastSegment = false;
	bool extraSpeed = false;

	if (behindSafetyCar == true)
	{
		// Load in control parameters.
		sRunActualParams.P 		= paramListSRun.lapParade.P;
		sRunActualParams.Kp 	= paramListSRun.lapParade.Kp;
		sRunActualParams.Kd 	= paramListSRun.lapParade.Kd;
		sRunActualParams.Speed	= paramListSRun.lapParade.Speed;

		lastSegment = sRunDriveStateMachine();

		if (actLapSegment == 3 || actLapSegment == 4 || actLapSegment == 5)
		{
			extraSpeed = true;
		}

		// Follow the safety car. WARNING: Keep distance calculates the speed, line follow set the speed.
		sRunActSpeedDist = cntrDistance(SRUN_DIST_SETPOINT,
										sRunPrevFrontDist,
										sRunActFrontDist,
										SRUN_DIST_KEEP_KP,
										SRUN_DIST_KEEP_SPEED_MIN,
										SRUN_DIST_KEEP_SPEED_MAX,
										extraSpeed);

		// Try to overtake if it is enabled.
		if (tryToOvertake == true && actLapSegment == SRUN_OVERTAKE_SEGMENT)
		{
			smMainStateSRun = eSTATE_MAIN_OVERTAKING;
		}
		else
		{
			// Follow the safety car until the last corner.
			smMainStateSRun = eSTATE_MAIN_PARADE_LAP;
		}

		if (startGateFound == true || lastSegment == true)
		{
			smMainStateSRun = eSTATE_MAIN_LAP_1;
		}
	}
	else
	{
		// We overtook the safety car!!! :D
		// Speed up. Load in control parameters.
		sRunSpeedUpParadeLap();

		lastSegment = sRunDriveStateMachine();

		// Start gate means a new lap.
		if (startGateFound == true || lastSegment == true)
		{
			smMainStateSRun = eSTATE_MAIN_LAP_1;
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

//**********************************************************************************************************************
//!
//!
//! @return -
//**********************************************************************************************************************
static eSEGMENT_TYPE sRunGetSegmentType (void)
{
	eSEGMENT_TYPE actualSegmentType;
	uint8_t lineNbr;

	// Actual line sensor data.
	lineNbr = lineGetRawFront().cnt;
	// Actual distance from encoder since start.
	lineLenght = (uint32_t)(speedGetDistance() * 1000) - lineStart;	// mm
	// Increment time.
	lineTimeCounter++;

	// Keep the previous segment type between the changing procedure.
	actualSegmentType = prevSegmentType;

	switch (segmentTypeCounter)
	{
		case 0:	// Changing segment.
		{
			// New line type.
			if (lineNbr == 1)
			{
				// One main line check.
				segmentTypeCounter = 1;
			}
			else if (lineNbr == 3)
			{
				// Slow down and speed up segment check.
				segmentTypeCounter = 2;
			}
			else if (lineNbr == 0)
			{
				// Lost track check.
				segmentTypeCounter = 3;
			}

			// Reset the timer.
			lineTimeCounter = 0;

			// Start is 0m.
			lineStart = (uint32_t)(speedGetDistance() * 1000);	// mm
			break;
		}
		case 1:	// Straight segment
		{
			// On line since a given distance or a given time.
			if (lineLenght > SRUN_SEG_MIN_LEN /*|| lineTimeCounter > 10*/)
			{
				// It is not a noise, it is a main single line.
				if (prevSegmentType != eSEG_SPEED_UP)
				{
					actualSegmentType = eSEG_CORNER;
				}
				else
				{
					actualSegmentType = eSEG_STRAIGHT;
				}

				prevSegmentType = actualSegmentType;
			}

			// Single line is lost or a noise.
			if (lineNbr != 1)
			{
				if (lineNewLine == false)
				{
					if (lineTimeCounter - lineTimePoint > 10)
					{
						// Single line is lost.
						segmentTypeCounter = 0;

						// Reset flag.
						lineNewLine = true;
					}
				}
				else
				{
					lineTimePoint = lineTimeCounter;
					lineNewLine = false;
				}
			}
			break;
		}
		case 2:	// Speed up or slow down segment.
		{
			// On line since a given distance or a given time.
			if (lineLenght > SRUN_SEG_MIN_LEN || lineTimeCounter > 10)
			{
				/*if (lineLenght > SRUN_SPEED_SUB_SEG_LEN_MAX)
				{
					actualSegmentType = eSEG_SLOW_DOWN;
				}
				else if (lineLenght > )
				{
					actualSegmentType = eSEG_SPEED_UP;
				}*/
				actualSegmentType = eSEG_SLOW_OR_SPEED;
				prevSegmentType = actualSegmentType;
			}

			// Line is found or a noise.
			if (lineNbr != 3)
			{
				if (lineNewLine == false)
				{
					if (lineTimeCounter - lineTimePoint > 10)
					{
						// Line is found
						segmentTypeCounter = 0;

						// Reset flag.
						lineNewLine = true;
					}
				}
				else
				{
					lineTimePoint = lineTimeCounter;
					lineNewLine = false;
				}
			}
			break;
		}
		case 3:
		{
			// On line since a given distance or a given time.
			if (lineLenght > SRUN_SEG_MIN_LEN || lineTimeCounter > 10)
			{
				// It is not a noise, it is a main single line.
				actualSegmentType = eSEG_LOST_TRACK;
				prevSegmentType = actualSegmentType;
			}

			// Line is found or a noise.
			if (lineNbr != 0)
			{
				if (lineNewLine == false)
				{
					if (lineTimeCounter - lineTimePoint > 10)
					{
						// Line is found
						segmentTypeCounter = 0;

						// Reset flag.
						lineNewLine = true;
					}
				}
				else
				{
					lineTimePoint = lineTimeCounter;
					lineNewLine = false;
				}
			}
			break;
		}
		default:
		{
			break;
		}
	}

	return actualSegmentType;
}

//**********************************************************************************************************************
//!
//!
//! @return -
//**********************************************************************************************************************
static void sRunLoadInParamsToRun (void)
{
	// Load in control parameters.
	switch (smMainStateSRun)
	{
		case eSTATE_MAIN_LAP_1:
		{
			sRunActualParams.P 		= paramListSRun.lap1[actLapSegment].P;
			sRunActualParams.Kp 	= paramListSRun.lap1[actLapSegment].Kp;
			sRunActualParams.Kd 	= paramListSRun.lap1[actLapSegment].Kd;
			sRunActualParams.Speed 	= paramListSRun.lap1[actLapSegment].Speed;
			break;
		}
		case eSTATE_MAIN_LAP_2:
		{
			sRunActualParams.P 		= paramListSRun.lap2[actLapSegment].P;
			sRunActualParams.Kp 	= paramListSRun.lap2[actLapSegment].Kp;
			sRunActualParams.Kd 	= paramListSRun.lap2[actLapSegment].Kd;
			sRunActualParams.Speed 	= paramListSRun.lap2[actLapSegment].Speed;
			break;
		}
		case eSTATE_MAIN_LAP_3:
		{
			sRunActualParams.P 		= paramListSRun.lap3[actLapSegment].P;
			sRunActualParams.Kp 	= paramListSRun.lap3[actLapSegment].Kp;
			sRunActualParams.Kd 	= paramListSRun.lap3[actLapSegment].Kd;
			sRunActualParams.Speed 	= paramListSRun.lap3[actLapSegment].Speed;
			break;
		}
		default:
		{
			break;
		}
	}
}

//**********************************************************************************************************************
//!
//!
//! @return -
//**********************************************************************************************************************
static void sRunSpeedUpParadeLap (void)
{
	paramListSRun.lapParade.P 		= paramListSRun.lap1[actLapSegment].P;
	paramListSRun.lapParade.Kp 		= paramListSRun.lap1[actLapSegment].Kp;
	paramListSRun.lapParade.Kd 		= paramListSRun.lap1[actLapSegment].Kd;
	paramListSRun.lapParade.Speed 	= paramListSRun.lap1[actLapSegment].Speed;

	sRunActSpeedDist = (float)(paramListSRun.lap1[actLapSegment].Speed) / 10.0f;

	if (actLapSegment == 7 || actLapSegment == 8)
	{
		sRunActSpeedDist = 1.8f;
	}
}

static float sRunConvertAngleTo180 (const float angle)
{
	float retVal;

	if (angle > PI)
	{
		retVal = angle - PI;
	}
	else
	{
		retVal = angle + PI;
	}

	return retVal;
}
