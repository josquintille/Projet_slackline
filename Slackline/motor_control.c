/*
 * motor_control.c
 *
 *  Created on: 15 avr. 2021
 *      Author: tille
 */

#include "motor_control.h"
#include "orientation.h"
#include "distance.h"

#include <motors.h>
#include <math.h>

			#include <ch.h>
			#include <chprintf.h> //chprintf((BaseSequentialStream *)&SD3, "distance = %d\n",get_target_distance() );

#define SIDE(x)  (signbit(x) ?  BACK : FRONT)

// regulator parameters
#define Ki	40
#define Kp 2700
#define Kd 100
#define AWM_MAX  1000
#define AWM_MIN  -AWM_MAX
#define MIN_SPEED 50

// target moving parameters
#define DISTANCE_NEAR 5 //[mm]
#define DISTANCE_FAR 15 // [mm]
#define LEAN_SPEED 200
#define LEAN_ANGLE 0.1f //[rad]
#define MOVING_SPEED 700

enum MOVING_SEQUENCE {LEAN, LET_FALL, MOVE, DONE};

// recovery parameters
#define CRITICAL_ANGLE 		1//[rad]
#define WAIT_UNTILL_DOWN 	500 //[ms]
#define WAIT_UNTILL_UP 		200 // [ms]
enum PUCK_SIDE {BACK=-1,NONE = 0, FRONT=1};



/*
 *	set the both motor at the same speed
 */
static void set_motor_speed(int speed);
/*
 *  PID regulator
 *  input: 	difference between angle and goal angle
 *  		as well as the angular speed (in rad/s)
 */
static int regulator_speed(float input_angle, float input_speed);

/*
 * move the e-puck away or toward the target
 * in  : side Back or front, angle, angular speed of the e-pcuk
 * out : 0 if still moving 1
 */
static uint8_t move(int8_t side, float input_angle, float input_speed);
/*
 *
 * sequence if the e-puck is down, boost him to recover the straight position
 * input: BACK or FRONT, the side where the e-puck is down
 */
static void recover(int8_t side);
static int8_t get_falling_side(float angle, float speed);

static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     int motor_speed = 0;
     float angle = 0;
     float angular_speed = 0;
	 uint8_t falling_side = NONE;

     while(1)
     {	// get angle from IMU (orientation.h)
		angle = get_angle();
		angular_speed = get_angular_speed();


		motor_speed = 0;//regulator_speed(angle, angular_speed);

		set_motor_speed(motor_speed);

		// detect if the e-puck is down
		falling_side = get_falling_side(angle, angular_speed);
		if(falling_side != NONE)
			recover(falling_side);

		// let other thread work
		chThdSleepMilliseconds(1);
     }
}

void motor_control_start(void)
{
	// setup motors
	motors_init();
	// setup IMU
	orientation_start();
	// setup TOF
	init_distance();

	// launch the thread
	chThdCreateStatic(motor_control_thd_wa, sizeof(motor_control_thd_wa), NORMALPRIO+1, motor_control_thd, NULL);
}

static void set_motor_speed(int speed)
{
	// threshold to avoid jerk at low speed
	if(fabs(speed) > MIN_SPEED)
	{
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);
	}
	else
	{
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}
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

	return Kp*input_angle + integ + input_speed*Kd;
}


static uint8_t move(int8_t side, float input_angle, float input_speed)
{
	static enum MOVING_SEQUENCE state = DONE;
	switch(state)
	{
	case DONE:
		state = LEAN;
		set_motor_speed(-side*LEAN_SPEED);
		break;
	case LEAN:
		if(input_angle  )

	}
	return 0;
}

static int8_t get_falling_side(float angle, float speed)
{
	if(fabs(angle) < CRITICAL_ANGLE || (SIDE(angle) != SIDE(speed)))
		return NONE;
	else
		return SIDE(angle);
}

static void recover(int8_t side)
{
	set_motor_speed(0);
	chThdSleepMilliseconds(WAIT_UNTILL_DOWN);
	set_motor_speed(side*MOTOR_SPEED_LIMIT);
	chThdSleepMilliseconds(WAIT_UNTILL_UP);
}
