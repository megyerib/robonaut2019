////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_servo.c
//!  \brief
//!  \details
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Includes ------------------------------------------------------------------------------------------------------------

#include <bsp_servo.h>
#include "app_servo.h"

#include "event_groups.h"

#include "trace.h"
#include "steer.h"
#include "bsp_common.h"

//TODO
#include "tim.h"
#include "motor.h"


// Typedefs ------------------------------------------------------------------------------------------------------------

#define LOG_RATE 	4
#define		BSP_SRV_HTIM2			 	htim2
#define		BSP_SRV_TIM_INSATNCE	 	TIM2
#define		BSP_SRV_TIM_CHANNEL		 	TIM_CHANNEL_1

// Local (static) & extern variables -----------------------------------------------------------------------------------

extern EventGroupHandle_t event_sharp;

extern QueueHandle_t qServoAngle_f;

// Local (static) function prototypes ----------------------------------------------------------------------------------
// Global function definitions -----------------------------------------------------------------------------------------

void TaskInit_Servo(void)
{
	//servoInit();

	xTaskCreate(Task_Servo,
				"TASK_SERVO",
				DEFAULT_STACK_SIZE,
				NULL,
				TASK_SRV_PRIO,
				NULL);
}

void Task_Servo(void* p)
{
	(void)p;

	//EventBits_t bits;
	double theta;
	double degree;
	uint32_t rate = LOG_RATE;
	GPIO_PinState btn;
	int enab = 0;
	//uint8_t comp = 114;

	while(1)
	{
		/*bits = xEventGroupGetBits(event_sharp);

		if(bits == 1)
		{
			servoSetAngle(2*PI/3);
		}
		else
		{
			servoSetAngle(PI/3);
		}*/

		btn = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

		if (enab == 0)
		{
			if (btn == GPIO_PIN_RESET)
			{
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
				steerSetAngle(15*PI/180);
			}
			else
			{
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
				steerSetAngle(0*PI/180);
			}
		}
		else
		{
			/*if (btn == GPIO_PIN_RESET)
			{
				steerSetAngle(15*PI/180);
			}
			else
			{
				steerSetAngle(0*PI/180);
			}*/
		}


		theta = servoGetAngle();

		// Angle in degree
		degree = theta * 180 /PI;
		// No warning
		degree = degree + 1 - 1;

		if(rate == 0)
		{
			traceBluetooth(BCM_LOG_SERVO_ANGLE, &theta);
			rate = LOG_RATE;
		}
		else
		{
			rate--;
		}


		vTaskDelay(TASK_DELAY_20_MS);
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

