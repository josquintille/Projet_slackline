/*
 * motor_control.c
 *
 *  Created on: 15 avr. 2021
 *      Author: tille
 */

#include "motor_control.h"
#include "orientation.h"
#include <motors.h>

// regulator variable
#define Ki	0
#define Kp 2000
#define AWM_MIN  -100
#define AWM_MAX  -AWM_MIN
/*
 *  Dc part of the regulator (PI regulator)
 *  input: difference between angle and goal angle
 */
static int regulator_speed(float input_angle);

static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     int speed = 0;
     while(1)
     {
		speed = regulator_speed(-get_angle());
		//applies the speed from the PI regulator
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);

		chThdSleepMilliseconds(100);
     }
}

void motor_control_start(void)
{
	// setup motors
	motors_init();
	// setup IMU
	orientation_start();

	// launch the thread
	chThdCreateStatic(motor_control_thd_wa, sizeof(motor_control_thd_wa), NORMALPRIO, motor_control_thd, NULL);
}

static int regulator_speed(float input_angle)
{ // pi regulator
	static int integ = 0;
	input_angle = input_angle*input_angle*input_angle;
	integ += Ki * input_angle;
	// AWM
	if(integ > AWM_MAX)
		integ = AWM_MAX;
	else if (integ < AWM_MIN )
		integ = AWM_MIN;

	return Kp*input_angle + integ;
}
