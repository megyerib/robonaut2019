////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_inertCalibration.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "app_inertCalibration.h"
#include "app_common.h"
#include "inert.h"
#include "main.h"
#include "trace.h"

// Defines -------------------------------------------------------------------------------------------------------------
// Typedefs ------------------------------------------------------------------------------------------------------------
// Local (static) & extern variables -----------------------------------------------------------------------------------

static float Xgain;
static float Xofs;
static float XtoY;
static float XtoZ;
static float Ygain;
static float Yofs;
static float YtoX;
static float YtoZ;
static float Zgain;
static float Zofs;
static float ZtoX;
static float ZtoY;

static float arrXofs[3];
static float arrYofs[3];
static float arrZofs[3];

static float arrXgain[2];
static float arrYgain[2];
static float arrZgain[2];

static float arrXtoY[2];
static float arrXtoZ[2];
static float arrYtoX[2];
static float arrYtoZ[2];
static float arrZtoX[2];
static float arrZtoY[2];

static GPIO_PinState blueBtn = GPIO_PIN_SET;
static bool pressed = false;
static bool enable  = false;

static uint8_t measurementIndex = 0;

// Accelerometer____________________________________

static ACCEL temp;

static float AccX[6];
static float AccY[6];
static float AccZ[6];

static bool accelCalibFinished;

//Gyroscope_________________________________________

static ANGVELd Wofs;
static ANGVELd w;

static float AngX[6];	// Roll
static float AngY[6];	// Pitch
static float AngZ[6];	// Yaw

static bool ofsSaved;
static bool angulCalibFinished;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static void inertCal_CalculateOffset();
static void inertCal_CalculateGain();
static void inertCal_CalculateCrossAxisGain();
static void inertCal_DebounceButton (void);
static void inertCal_Accelerometer (void);

// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_InertialCalibration(void)
{
	ofsSaved = false;
	angulCalibFinished = false;

	xTaskCreate(Task_InertialCalibration,
				"TASK_INERTCALIBRATION",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_INRT_CAL_PRIO,
				NULL);
}


void Task_InertialCalibration(void* p)
{
	(void)p;

	while (1)
	{
		inertCal_DebounceButton();

		if (accelCalibFinished)
		{
			inertCal_Accelerometer();
		}

		if (ofsSaved == false && measurementIndex == 0)
		{
			Wofs = inertGetAngVel();

			ofsSaved = true;
			measurementIndex = 0;
		}
		else if (angulCalibFinished == false && ofsSaved == true)
		{
			switch (measurementIndex)
			{
				case 1:
				{
					if (enable == true)
					{
						// Rotate the inertial sensor in [l, 0, 0]. 6-point tumble calibration 1st measurement.
						// AngX1 = -Xgain + Xofs, AngY1 = -XtoY + Yofs, AngZ1 = -XtoZ + Zofs
						w = inertGetAngVel();

						AngX[0] = w.omega_x;
						AngY[0] = w.omega_y;
						AngZ[0] = w.omega_z;

						measurementIndex = 2;
						enable = false;
					}
					break;
				}
				case 2:
				{
					if (enable == true)
					{
						// Rotate the inertial sensor in [-l, 0, 0]. 6-point tumble calibration 2nd measurement.
						// AngX2 = -Xgain + Xofs, AngY2 = -XtoY + Yofs, AngZ2 = -XtoZ + Zofs
						w = inertGetAngVel();

						AngX[1] = w.omega_x;
						AngY[1] = w.omega_y;
						AngZ[1] = w.omega_z;

						measurementIndex = 3;
						enable = false;
					}
					break;
				}
				case 3:
				{
					if (enable == true)
					{
						// Rotate the inertial sensor in [0, m, 0]. 6-point tumble calibration 3rd measurement.
						// AngX3 = -Xgain + Xofs, AngY3 = -XtoY + Yofs, AngZ3 = -XtoZ + Zofs
						w = inertGetAngVel();

						AngX[2] = w.omega_x;
						AngY[2] = w.omega_y;
						AngZ[2] = w.omega_z;

						measurementIndex = 4;
						enable = false;
					}
					break;
				}
				case 4:
				{
					if (enable == true)
					{
						// Rotate the inertial sensor in [0, -m, 0]. 6-point tumble calibration 4th measurement.
						// AngX4 = -Xgain + Xofs, AngY4 = -XtoY + Yofs, AngZ4 = -XtoZ + Zofs
						w = inertGetAngVel();

						AngX[3] = w.omega_x;
						AngY[3] = w.omega_y;
						AngZ[3] = w.omega_z;

						measurementIndex = 5;
						enable = false;
					}
					break;
				}
				case 5:
				{
					if (enable == true)
					{
						// Rotate the inertial sensor in [0, 0, n]. 6-point tumble calibration 4th measurement.
						// AngX5 = -Xgain + Xofs, AngY5 = -XtoY + Yofs, AngZ5 = -XtoZ + Zofs
						w = inertGetAngVel();

						AngX[4] = w.omega_x;
						AngY[4] = w.omega_y;
						AngZ[4] = w.omega_z;

						measurementIndex = 6;
						enable = false;
					}
					break;
				}
				case 6:
				{
					if (enable == true)
					{
						// Rotate the inertial sensor in [0, 0, -n]. 6-point tumble calibration 4th measurement.
						// AngX5 = -Xgain + Xofs, AngY5 = -XtoY + Yofs, AngZ5 = -XtoZ + Zofs
						w = inertGetAngVel();

						AngX[5] = w.omega_x;
						AngY[5] = w.omega_y;
						AngZ[5] = w.omega_z;

						measurementIndex = 7;
						enable = false;
					}
					break;
				}
				case 7:
				{
					// Calibration

					// Calculate the offsets.
					inertCal_CalculateOffset();

					// Calculate the gains.
					inertCal_CalculateGain();

					// Calculate the cross-axis gains.
					inertCal_CalculateCrossAxisGain();

					// Save the calibration result in the inert module.
					inert6PointCalibration(
												Xgain,
												Xofs,
												XtoY,
												XtoZ,
												Ygain,
												Yofs,
												YtoX,
												YtoZ,
												Zgain,
												Zofs,
												ZtoX,
												ZtoY
											);

					measurementIndex = 0;
				}
				default:
				{
					// Start calibration
					if (enable == true)
					{
						measurementIndex = 1;
					}
					break;
				}
			}
		}

		inertTriggerMeasurement();

		vTaskDelay(TASK_DELAY_16_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static void inertCal_CalculateOffset()
{
	// AccX1+AccX2 = 2 Xofs1, AccX3+AccX4 = 2 Xofs2, AccX5+AccX6 = 2 Xofs3  -> Xofs = avg(Xofs1, Xofs2, Xofs3)
	arrXofs[0] = (AccX[0] + AccX[1]) / 2.0;
	arrXofs[1] = (AccX[2] + AccX[3]) / 2.0;
	arrXofs[2] = (AccX[4] + AccX[5]) / 2.0;
	Xofs = (arrXofs[0] + arrXofs[1] + arrXofs[2]) / 3.0;

	// AccY1+AccY2 = 2 Yofs1, AccY3+AccY4 = 2 Yofs2, AccY5+AccY6 = 2 Yofs3  -> Yofs = avg(Yofs1, Yofs2, Yofs3)
	arrYofs[0] = (AccY[0] + AccY[1]) / 2.0;
	arrYofs[1] = (AccY[2] + AccY[3]) / 2.0;
	arrYofs[2] = (AccY[4] + AccY[5]) / 2.0;
	Yofs = (arrYofs[0] + arrYofs[1] + arrYofs[2]) / 3.0;

	// AccZ1+AccZ2 = 2 Zofs1, AccZ3+AccZ4 = 2 Zofs2, AccZ5+AccZ6 = 2 Zofs3  -> Zofs = avg(Zofs1, Zofs2, Zofs3)
	arrZofs[0] = (AccZ[0] + AccZ[1]) / 2.0;
	arrZofs[1] = (AccZ[2] + AccZ[3]) / 2.0;
	arrZofs[2] = (AccZ[4] + AccZ[5]) / 2.0;
	Zofs = (arrZofs[0] + arrZofs[1] + arrZofs[2]) / 3.0;
}

static void inertCal_CalculateGain()
{
	// Xgain1 = AccX1-Xofs, Xgain2 = -AccX2+Xofs  -> Xgain = avg(Xgain1, Xgain2)
	arrXgain[0] = AccX[0] - Xofs;
	arrXgain[1] = -AccX[1] + Xofs;
	Xgain = (arrXgain[0] + arrXgain[1]) /  2.0;

	// Ygain1 = AccY3-Yofs, Ygain2 = -AccY4+Yofs  -> Ygain = avg(Ygain1, Ygain2)
	arrYgain[0] = AccY[2] - Yofs;
	arrYgain[1] = -AccY[3] + Yofs;
	Ygain = (arrYgain[0] + arrYgain[1]) /  2.0;

	// Zgain1 = AccZ5-Zofs, Zgain2 = -AccZ6+Zofs  -> Zgain = avg(Zgain1, Zgain2)
	arrZgain[0] = AccZ[4] - Zofs;
	arrZgain[1] = -AccZ[5] + Zofs;
	Zgain = (arrZgain[0] + arrZgain[1]) /  2.0;
}

static void inertCal_CalculateCrossAxisGain()
{
	// XtoY1 = AccY1-Yofs, XtoY2 = -AccY2+Yofs    -> XtoY = avg(XtoY1, XtoY2)
	arrXtoY[0] = AccY[0] - Yofs;
	arrXtoY[1] = -AccY[1] + Yofs;
	XtoY = (arrXtoY[0] + arrXtoY[1]) /  2.0;

	// XtoZ1 = AccZ1-Zofs, XtoZ2 = -AccZ2+Zofs    -> XtoZ = avg(XtoZ1, XtoZ2)
	arrXtoZ[0] = AccZ[0] - Zofs;
	arrXtoZ[1] = -AccZ[1] + Zofs;
	XtoZ = (arrXtoZ[0] + arrXtoZ[1]) /  2.0;

	// YtoX1 = AccX3-Xofs, YtoX2 = -AccX4+Xofs    -> YtoX = avg(YtoX1, YtoX2)
	arrYtoX[0] = AccX[2] - Xofs;
	arrYtoX[1] = -AccX[3] + Xofs;
	YtoX = (arrYtoX[0] + arrYtoX[1]) /  2.0;

	// YtoZ1 = AccZ3-Zofs, YtoZ2 = -AccZ4+Zofs    -> YtoZ = avg(YtoZ1, YtoZ2)
	arrYtoZ[0] = AccZ[2] - Zofs;
	arrYtoZ[1] = -AccZ[3] + Zofs;
	YtoZ = (arrYtoZ[0] + arrYtoZ[1]) /  2.0;

	// ZtoX1 = AccX5-Xofs, ZtoX2 = -AccX6+Xofs    -> ZtoX = avg(ZtoX1, ZtoX2)
	arrZtoX[0] = AccX[4] - Xofs;
	arrZtoX[1] = -AccX[5] + Xofs;
	ZtoX = (arrZtoX[0] + arrZtoX[1]) /  2.0;

	// ZtoY1 = AccY5-Yofs, ZtoY2 = -AccY6+Yofs    -> ZtoY = avg(ZtoY1, ZtoY2)
	arrZtoY[0] = AccY[4] - Yofs;
	arrZtoY[1] = -AccY[5] + Yofs;
	ZtoY = (arrZtoY[0] + arrZtoY[1]) /  2.0;
}

static void inertCal_DebounceButton (void)
{
	blueBtn = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	if (pressed == false && blueBtn == GPIO_PIN_RESET)
	{
		vTaskDelay(30);
		blueBtn = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		if (blueBtn == GPIO_PIN_RESET)
		{
			pressed = true;
			enable = true;
		}
	}
	else if (pressed == true && blueBtn == GPIO_PIN_SET)
	{
		vTaskDelay(30);
		blueBtn = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		if (blueBtn == GPIO_PIN_SET)
		{
			pressed = false;
		}
	}
}

static void inertCal_Accelerometer (void)
{
	if (enable == true && measurementIndex == 1)
	{
		// Set the inertial sensor in [0, 0, -1]. 6-point tumble calibration 6th measurement.
		// AccX5 = -ZtoX + Xofs, AccY5 = -ZtoY + Yofs, AccZ4 = -Zgain + Zofs
		temp = inertGetAccel();

		AccX[5] = temp.a_x;
		AccY[5] = temp.a_y;
		AccZ[5] = temp.a_z;

		measurementIndex = 2;
		enable = false;
	}
	else if (enable == true && measurementIndex == 2)
	{
		// Set the inertial sensor in [-1, 0, 0]. 6-point tumble calibration 2nd measurement.
		// AccX2 = -Xgain + Xofs, AccY2 = -XtoY + Yofs, AccZ2 = -XtoZ + Zofs
		temp = inertGetAccel();

		AccX[1] = temp.a_x;
		AccY[1] = temp.a_y;
		AccZ[1] = temp.a_z;

		measurementIndex = 3;
		enable = false;
	}
	else if (enable == true && measurementIndex == 3)
	{
		// Set the inertial sensor in [0, 0, +1]. 6-point tumble calibration 5th measurement.
		// AccX5 = ZtoX + Xofs, AccY5 = ZtoY + Yofs, AccZ4 = Zgain + Zofs
		temp = inertGetAccel();

		AccX[4] = temp.a_x;
		AccY[4] = temp.a_y;
		AccZ[4] = temp.a_z;

		measurementIndex = 4;
		enable = false;
	}
	else if (enable == true && measurementIndex == 4)
	{
		// Set the inertial sensor in [+1, 0, 0]. 6-point tumble calibration 1st measurement.
		// AccX1 = Xgain + Xofs, AccY1 = XtoY + Yofs, AccZ1 = XtoZ + Zofs
		temp = inertGetAccel();

		AccX[0] = temp.a_x;
		AccY[0] = temp.a_y;
		AccZ[0] = temp.a_z;

		measurementIndex = 5;
		enable = false;
	}
	else if (enable == true && measurementIndex == 5)
	{
		// Set the inertial sensor in [0, +1, 0]. 6-point tumble calibration 3rd measurement.
		// AccX3 = YtoZ + Xofs, AccY3 = Ygain + Yofs, AccZ3 = YtoZ + Zofs
		temp = inertGetAccel();

		AccX[2] = temp.a_x;
		AccY[2] = temp.a_y;
		AccZ[2] = temp.a_z;

		measurementIndex = 6;
		enable = false;
	}
	else if(enable == true && measurementIndex == 6)
	{
		// Set the inertial sensor in [0, -1, 0]. 6-point tumble calibration 4th measurement.
		// AccX4 = -YtoZ + Xofs, AccY4 = -Ygain + Yofs, AccZ4 = -YtoZ + Zofs
		temp = inertGetAccel();

		AccX[3] = temp.a_x;
		AccY[3] = temp.a_y;
		AccZ[3] = temp.a_z;

		measurementIndex = 7;
		enable = false;
	}
	else if(measurementIndex == 7)
	{
		// Calibration

		// Calculate the offsets.
		inertCal_CalculateOffset();

		// Calculate the gains.
		inertCal_CalculateGain();

		// Calculate the cross-axis gains.
		inertCal_CalculateCrossAxisGain();

		// Save the calibration result in the inert module.
		inert6PointCalibration(
									Xgain,
									Xofs,
									XtoY,
									XtoZ,
									Ygain,
									Yofs,
									YtoX,
									YtoZ,
									Zgain,
									Zofs,
									ZtoX,
									ZtoY
								);

		measurementIndex = 0;
	}
	else
	{
		// NO measurement

		// Start calibration
		if (enable == true)
		{
			measurementIndex = 1;
		}
	}
}
