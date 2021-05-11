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
#define INTEG_THRESHOLD 0.02f
#define MIN_SPEED 50

#define STABLE_ANGLE 0.05 //[rad]
#define STABLE_SPEED 0.05 //[rad/s]

// target moving parameters
#define MIN_TARGET_DISTANCE 70 //[mm]
#define MAX_TARGET_DISTANCE 130 // [mm]
#define LEAN_SPEED 200
#define LEAN_ANGLE 0.3f //[rad]
#define FALL_SPEED .8//[rad/s]
#define FALL_ANGLE .7//[rad]
#define MOVING_SPEED 700

typedef enum moving_sequence_t {LEAN, LET_FALL, MOVE, DONE, NOT_DONE} moving_sequence_t;

// recovery parameters
#define CRITICAL_ANGLE 		1//[rad]
#define WAIT_UNTILL_DOWN 	500 //[ms]
#define WAIT_UNTILL_UP 		200 // [ms]
enum puck_side_t {BACK=-1,NONE = 0, FRONT=1};

#define SLEEP_TIME 1 //[ms]

static control_mode_t control_mode = BALANCE;
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

static void balance_mode(float input_angle, float input_speed);

static void following_mode(float input_angle, float input_speed);
/*
 * move the e-puck away or toward the target
 * in  : side Back or front, angle, angular speed of the e-pcuk
 * out : side if still moving, NONE if done
 */
static int8_t move(int8_t side, float input_angle, float input_speed);
/*
 * sequence if the e-puck is down, boost him to recover the straight position
 */
static void recover(void);
static int8_t get_falling_side(float angle, float speed);


/////////////////////////DEFINITION///////////////////////////////


static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     float angle = 0;
     float angular_speed = 0;

	 int8_t falling_side = NONE;
     while(1)
     {	// get angle from IMU (orientation.h)
		angle = get_angle();
		angular_speed = get_angular_speed();


		switch(control_mode)
		{
		case BALANCE:
			balance_mode(angle,angular_speed);
			break;
		case FOLLOW_TARGET:
			following_mode(angle,angular_speed);
			break;
		default:
			balance_mode(angle,angular_speed);
		}

		// detect if the e-puck is down
		falling_side = get_falling_side(angle, angular_speed);
		if(falling_side != NONE)
			recover();

		// let other thread work
		chThdSleepMilliseconds(SLEEP_TIME);
     }
}

void set_control_mode(control_mode_t mode)
{
	control_mode = mode;
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
{	// pid regulator
	// integrator
	static float integ = 0;
	if(fabs(input_angle) > INTEG_THRESHOLD)
		integ += Ki*input_angle;
	// AWM
	if(integ > AWM_MAX)
		integ = AWM_MAX;
	else if (integ < AWM_MIN )
		integ = AWM_MIN;

	return Kp*input_angle + integ + Kd*input_speed;
}

static void balance_mode(float input_angle, float input_speed)
{
	set_motor_speed(regulator_speed(input_angle, input_speed));
}

static void following_mode(float input_angle, float input_speed)
{
	static int8_t current_movement = NONE;

	// update side (don't change side during sequence)
	if(current_movement == NONE)
	{
		uint16_t distance = get_target_distance();

		if(distance > MAX_TARGET_DISTANCE)
			current_movement = FRONT;
		else if (distance < MIN_TARGET_DISTANCE)
			current_movement = BACK;
	}

	if(current_movement != NONE)
		current_movement = move(current_movement,input_angle,input_speed);
	else // idle: stay stable
		balance_mode(input_angle,input_speed);
}

static int8_t move(int8_t side, float input_angle, float input_speed)
{
	static moving_sequence_t state = DONE;
	switch(state)
	{
	case DONE:
		state = LEAN;
		//set_motor_speed(-side*LEAN_SPEED);
		break;
	case LEAN:
		balance_mode(input_angle-side*LEAN_ANGLE, 0);
		if(fabs(input_angle) >= LEAN_ANGLE )
		{
			state = LET_FALL;
			set_motor_speed(0);
		}
		break;
	case LET_FALL:
		if(fabs(input_speed) >= FALL_SPEED )
		{
			state = MOVE;
			set_motor_speed(side*MOVING_SPEED);
		}
		break;
	case MOVE:
		if(fabs(input_angle) <= STABLE_ANGLE )
		{
			state = DONE;
			side = NONE;
			set_motor_speed(0);
		}
	default:
		state = DONE;
	}

	return side;
}

static int8_t get_falling_side(float angle, float speed)
{
	if(fabs(angle) < CRITICAL_ANGLE || (SIDE(angle) != SIDE(speed)))
		return NONE;
	else
		return SIDE(angle);
}

static void recover(void)
{
	set_motor_speed(0);
	chThdSleepMilliseconds(WAIT_UNTILL_DOWN);
	float angle = 0;
	while(fabs(angle = get_angle()) > STABLE_ANGLE)
	{
		balance_mode(angle,get_angular_speed());
		chThdSleepMilliseconds(SLEEP_TIME);
	}
}
