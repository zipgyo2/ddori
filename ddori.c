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

DeclareCounter(SysTimerCnt);

/*Declare Resource*/
DeclareResource(Move);
DeclareResource(Turn);

/*Declare Task*/
DeclareTask(Steering);
DeclareTask(Movement);
DeclareTask(Sonar);

/* Global Value */
int left_sonar_sensor = 15, right_sonar_sensor = 15; // Result of Median Filter
int speed = 0;                                       //speed

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
  static int left_sonar_array[5] = {15, 15, 15, 15, 15};
  static int right_sonar_array[5] = {15, 15, 15, 15, 15};
  static U8 entry_count = 0;

  //Declare Value
  int left_sort[5], right_sort[5]; //Median Filter array
  int i, j = 0;
  int temp;

  // Read Sensor
  left_sonar_array[entry_count] = ecrobot_get_sonar_sensor(S3);
  right_sonar_array[entry_count] = ecrobot_get_sonar_sensor(S4);

  // Sonar Sensor Correction
  if (left_sonar_array[entry_count] >= 100)
  {
    left_sonar_array[entry_count] = 100;
  }
  if (right_sonar_array[entry_count] >= 100)
  {
    right_sonar_array[entry_count] = 100;
  }

  entry_count++;

  // Sensor Median Filter
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
  // Result of Median Filter
  left_sonar_sensor = left_sort[2];
  right_sonar_sensor = right_sort[2];

  //entry_count reset
  if (entry_count == 5)
  {
    entry_count = 0;
  }
}

TASK(Steering)
{
  //Static Value
  static int steering_count = 0;
  static double sonar_width = 15.5;
  static double previous_steer = 0.0;

  //Value
  double d = (double)left_sonar_sensor - (double)right_sonar_sensor;
  double angle = atan2(d, sonar_width);
  angle = angle * RAD; // Result of angle by difference of left_sonar_sensor and right_sonar_sensor

  // angle correction
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
  // set steer by angle
  steering_count = nxt_motor_get_count(B);
  previous_steer = angle;
  if (angle > 25)
  {
    if (steering_count < angle * 2.4)
    {
      nxt_motor_set_speed(B, 34, 1);
    }
    else if (steering_count > angle * 2.4)
    {
      nxt_motor_set_speed(B, -36, 1);
    }
    else
    {
      nxt_motor_set_speed(B, 0, 1);
    }
  }
  else
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
  display_goto_xy(8, 4);
  display_int(steering_count, 3);
  display_goto_xy(12, 4);
  display_int(angle, 3);
  display_update();

  TerminateTask();
}
TASK(Movement)
{
  //Constance Value
  const int low = 40;
  const int high = 90;
  //Value
  int left_sensor = left_sonar_sensor;
  int right_sensor = right_sonar_sensor;
  int avg = (left_sensor + right_sensor) / 2;
  int diff = (avg - 15);

  int tmp = 0;
  // set speed by previous distance and current distance
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
  else if (diff < -1)
  {
    tmp--;
    speed = -low - tmp;
  }
  else if (diff > 1)
  {
    tmp++;
    speed = low + tmp;
  }

  nxt_motor_set_speed(A, 0, 1);
  nxt_motor_set_speed(C, 0, 1);
  // set speed when too far
  if (avg >= 100)
    speed = 0;

  nxt_motor_set_speed(A, -speed, 1);
  nxt_motor_set_speed(C, -speed, 1);

  display_goto_xy(0, 7);
  display_int(diff, 3);
  display_goto_xy(4, 7);
  display_int(avg, 3);
  display_goto_xy(8, 7);
  display_int(speed, 3);
  display_update();

  TerminateTask();
}
