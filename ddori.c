#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "math.h"
#include "stdlib.h"

#define A NXT_PORT_A
#define B NXT_PORT_B
#define C NXT_PORT_C
#define S1 NXT_PORT_S1
#define S2 NXT_PORT_S2
#define S3 NXT_PORT_S3
#define S4 NXT_PORT_S4

#define RAD 57.2958

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
DeclareEvent(MovementEvent);
DeclareEvent(SteeringEvent);
DeclareTask(Steering);
DeclareTask(Movement);
DeclareTask(Sonar);

/* Global Value */
int left_sonar_sensor;
int right_sonar_sensor;

/* Ecrobot Device Initialize */
void ecrobot_device_initialize()
{
	ecrobot_init_sonar_sensor(S3);
	ecrobot_init_sonar_sensor(S4);
}

void ecrobot_device_terminate()
{
	ecrobot_term_sonar_sensor(S3);
	ecrobot_term_sonar_sensor(S4);
}

/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	StatusType ercd;

	ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
	if (ercd != E_OK)
		ShutdownOS(ercd);
}

/* Auto-run Task -------------------------------------*/
TASK(Sonar) //15ms마다 입력 받음
{
	//Static Value
	static int left_sonar_array[5] = { 15, 15, 15, 15, 15 }; //rigiht sensor input array (sliding window)
	static int right_sonar_array[5] = { 15, 15, 15, 15, 15 }; //left sensor input array (sliding window)

															  //Value
	static U8 entry_count = 0;
	int i, j = 0;
	int left_sort[5], right_sort[5]; //sorting array
	int temp;

	//get sensor input
	left_sonar_array[entry_count] = ecrobot_get_sonar_sensor(S3);
	right_sonar_array[entry_count] = ecrobot_get_sonar_sensor(S4);

	//Min Max calibrate
	if (left_sonar_array[entry_count] >= 100)
	{
		left_sonar_array[entry_count] = 100;
	}
	if (right_sonar_array[entry_count] >= 100)
	{
		right_sonar_array[entry_count] = 100;
	}

	//Median calibrate
	for (i = 0; i < 5; i++)
	{
		left_sort[i] = left_sonar_array[i];
		right_sort[i] = right_sonar_array[i];
	}

	for (i = 0; i < 5; i++)
	{
		for (j = 0; j < 5 - i; j++)
		{
			if (left_sort[j] > left_sort[j + 1])
			{
				temp = left_sort[j];
				left_sort[j] = left_sort[j + 1];
				left_sort[j + 1] = temp;
			}
			if (right_sort[j] > right_sort[j + 1])
			{
				temp = right_sort[j];
				right_sort[j] = right_sort[j + 1];
				right_sort[j + 1] = temp;
			}
		}
	}

	//Median result
	left_sonar_sensor = left_sort[2];
	right_sonar_sensor = right_sort[2];

	//Steering Event Set when angle is not 0
	if (left_sonar_sensor - right_sonar_sensor != 0)
	{
		SetEvent(Steering, SteeringEvent);
	}

	//Movement Event Set when distance changed
	int avg = (left_sonar_array[entry_count] + right_sonar_array[entry_count]) / 2;
	int prev_avg = 0;
	if (entry_count == 0)
	{
		prev_avg = (left_sonar_array[5] + right_sonar_array[5]) / 2;
	}
	else
	{
		prev_avg = (left_sonar_array[entry_count - 1] + right_sonar_array[entry_count - 1]) / 2;
	}
	if (avg - prev_avg != 0)
	{
		SetEvent(Movement, MovementEvent);
	}

	//Entry Count Rotate
	entry_count++;
	if (entry_count == 5)
	{
		entry_count = 0;
	}

	TerminateTask();
}

TASK(Steering)
{
	// event waiting
	while (WaitEvent(SteeringEvent) == E_OK)
	{
		ClearEvent(SteeringEvent);

		//static value
		static int steering_count = 0;
		static double previous_steer = 0.0;

		//const value
		const double sonar_width = 15.5;

		//vaule
		double d = (double)left_sonar_sensor - (double)right_sonar_sensor;
		double angle = atan2(d, sonar_width);
		angle = angle * RAD; //angle

							 //angle Min Max calibration
		if (angle >= 80 || angle <= -80)
		{
			angle = previous_steer;
		}

		if (d > 0 && angle > 45.0)
		{
			angle = 45.0;
		}
		else if (d < 0 && (angle < -45.0 || angle > 45.0))
		{
			angle = -45.0;
		}

		// diraction control by angle
		steering_count = nxt_motor_get_count(B);
		previous_steer = angle;
		//angle over 25 
		if (angle > 25)
		{
			if (steering_count < angle * 2.0)
			{
				nxt_motor_set_speed(B, 30, 1);
			}
			else if (steering_count > angle * 2.0)
			{
				nxt_motor_set_speed(B, -32, 1);
			}
			else
			{
				nxt_motor_set_speed(B, 0, 1);
			}
		}
		//angle under 25 
		else
		{
			if (steering_count < angle * 1.8)
			{
				nxt_motor_set_speed(B, 28, 1);
			}
			else if (steering_count > angle * 1.8)
			{
				nxt_motor_set_speed(B, -30, 1);
			}
			else
			{
				nxt_motor_set_speed(B, 0, 1);
			}
		}
	}
}
TASK(Movement)
{
	// event waiting
	while (1)
	{
		WaitEvent(MovementEvent);
		ClearEvent(MovementEvent);
		//static value
		static int speed = 0;

		//const value
		const int low = 47; //low mode speed
		const int high = 92;//high mode speed

							//value
		int left_sensor = left_sonar_sensor;
		int right_sensor = right_sonar_sensor;
		int avg = (left_sensor + right_sensor) / 2; //distance between control car and follow car
		int diff = (avg - 15); //distance excluding braking distance
		int tmp = 0;

		// crash avoidance mode
		if (avg >= 80)
		{
			nxt_motor_set_speed(A, 0, 1);
			nxt_motor_set_speed(C, 0, 1);
		}

		else
		{
			//high speed mode
			if (diff > 18)
			{
				speed = high;
				if (diff > 25)
				{
					tmp++;
					speed = speed + tmp;
				}
				else
				{
					tmp--;
					speed = speed + tmp;
				}
			}
			//low speed mose
			else if (diff < -2)
			{
				speed = -low - 5;
			}
			else if (diff > 2)
			{
				tmp++;
				speed = low + tmp;
			}

			//stop when braking distance
			if (diff < 2 && diff > -2)
			{
				speed = 0;
			}

			// crash avoidance mode
			if (left_sensor >= 95 || right_sensor >= 95)
			{
				speed = 0;
			}

			//motor speed control
			nxt_motor_set_speed(A, 0, 1);
			nxt_motor_set_speed(C, 0, 1);

			nxt_motor_set_speed(A, -speed, 1);
			nxt_motor_set_speed(C, -speed, 1);
		}
	}

	TerminateTask();
}