/*
 * motor_control.c
 *
 *  Created on: 15 avr. 2021
 *      Author: tille
 */

#include "motor_control.h"
#include "orientation.h"
#include <motors.h>
#include <math.h>

// regulator variable
#define Ki	40
#define Kp 3000
#define Kd 100
#define AWM_MAX  1000
#define AWM_MIN  -AWM_MAX
#define MIN_SPEED 50

// recovery variable
#define WAIT_UNTILL_DOWN 	500 //[ms]
#define WAIT_UNTILL_UP 		200 // MS
enum SIDE {BACK=-1, FRONT=1};
/*
 *  PID regulator
 *  input: 	difference between angle and goal angle
 *  		as well as the angular speed (in rad/s)
 */
static int regulator_speed(float input_angle, float input_speed);
/*
 *
 * sequence if the e-puck is down, boost him to recover the straight position
 * input: the sign
 */
static void recover(int8_t side);

static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     int speed = 0;
     static int intspeed = 0;
     while(1)
     {
		speed = - regulator_speed(get_angle(), get_angular_speed());

		//applies the speed from the PI regulator
		if(fabs(speed)>MIN_SPEED)
		{
			right_motor_set_speed(speed);
			left_motor_set_speed(speed);
		}
		else
		{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
		}
		// tombé
		if(fabs(get_angle()) > 1)
		{

		}
		chThdSleepMilliseconds(1);
     }
}

void motor_control_start(void)
{
	// setup motors
	motors_init();
	// setup IMU
	orientation_start();

	// launch the thread
	chThdCreateStatic(motor_control_thd_wa, sizeof(motor_control_thd_wa), NORMALPRIO+1, motor_control_thd, NULL);
}

static int regulator_speed(float input_angle, float input_speed)
{ // pid regulator
	//integrator
	static int integ = 0;
	integ += Ki * input_angle;
	// AWM
	if(integ > AWM_MAX)
		integ = AWM_MAX;
	else if (integ < AWM_MIN )
		integ = AWM_MIN;

	static int deriv = 0;
	//if ((signbit(input_speed) == signbit(input_angle)) & (input_angle>.1f) )//& input_speed >.05f)
		deriv = input_speed*Kd;

	return Kp*input_angle + integ + deriv;
}

static void recover(int8_t side)
{
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	chThdSleepMilliseconds(WAIT_UNTILL_DOWN);
	right_motor_set_speed(side*MOTOR_SPEED_LIMIT);
	left_motor_set_speed(side*MOTOR_SPEED_LIMIT);
	chThdSleepMilliseconds(WAIT_UNTILL_UP);
}
