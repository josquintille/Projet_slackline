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
#define Ki	10
#define Kp 0
#define Kd -1000
#define AWM_MAX  300
#define AWM_MIN  -AWM_MAX
#define MIN_SPEED 60
/*
 *  Dc part of the regulator (PI regulator)
 *  input: difference between angle and goal angle
 */
static int regulator_speed(float input_angle, float input_speed);

static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     int speed = 0;
     while(1)
     {
		speed = regulator_speed(get_angle(), -get_ang_speed());
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
	integ += Ki * input_speed;
	// AWM
	if(integ > AWM_MAX)
		integ = AWM_MAX;
	else if (integ < AWM_MIN )
		integ = AWM_MIN;

	static int deriv = 0;
	if ((signbit(input_speed) != signbit(input_angle)) & (input_speed>.5f))
		deriv = input_speed*Kd;

	return Kp*input_angle + integ + deriv;
}
