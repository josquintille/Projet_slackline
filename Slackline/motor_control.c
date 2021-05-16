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
#define Ki 40
#define Kp 2700
#define Kd 1000
#define AWM_MAX  1000
#define AWM_MIN  -AWM_MAX
#define INTEG_THRESHOLD 0.025
#define DERIV_THRESHOLD 4*STABLE_SPEED
#define MIN_SPEED 50

#define STABLE_ANGLE 0.05 //[rad]
#define STABLE_SPEED 0.05 //[rad/s]

// target moving parameters
#define MIN_TARGET_DISTANCE 70 //[mm]
#define MAX_TARGET_DISTANCE 130 // [mm]
#define LEAN_SPEED 200
#define LEAN_ANGLE 0.3f //[rad]
#define FALL_SPEED .2//[rad/s]
#define FALL_ANGLE .7//[rad]
#define MOVING_ACCELERATION 100000*SLEEP_TIME/1000
#define MOVING_SPEED 700

typedef enum moving_sequence_t {LEAN, LET_FALL, ACCELERATE, MOVE, DONE, NOT_DONE} moving_sequence_t;

// recovery parameters
#define CRITICAL_ANGLE 		1//[rad]
#define WAIT_UNTILL_DOWN 	1000 //[ms]
#define WAIT_SPEED_MAX		100 //[ms]
enum puck_side_t {BACK=-1,NONE = 0, FRONT=1};

#define SLEEP_TIME 1 //[ms]

static control_mode_t control_mode = BALANCE;
static _Bool reset_move_sequence = false;
static _Bool reset_integrator = false;
/*
 *	set the both motor at the same speed
 */
static void set_motor_speed(int speed);
/*
 *  PID regulator
 *  input: 	difference between angle and goal angle
 *  		as well as the angular speed (in rad/s)
 */
static int regulator_speed(float angle, float angular_speed);
static _Bool is_stable(float angle, float angular_speed);
static void balance_mode(float angle, float angular_speed);

static void following_mode(float angle, float angular_speed);
/*
 * move the e-puck away or toward the target
 * in  : side Back or front, angle, angular speed of the e-pcuk
 * out : side if still moving, NONE if done
 */
static int8_t move(int8_t side, float angle, float angular_speed);
/*
 * sequence if the e-puck is down, boost him to recover the straight position
 */
static void recover(void);
static _Bool has_fallen(float angle, float speed);


/////////////////////////DEFINITION///////////////////////////////


static THD_WORKING_AREA(motor_control_thd_wa, 512);
static THD_FUNCTION(motor_control_thd, arg)
{
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     float angle = 0;
     float angular_speed = 0;

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

			balance_mode(angle,angular_speed);
		}

		// recover if the e-puck is down
		if(has_fallen(angle, angular_speed))
			recover();

		// let other thread work
		chThdSleepMilliseconds(SLEEP_TIME);
     }
}

void set_control_mode(control_mode_t mode)
{
	if(control_mode != mode)
	{
		control_mode = mode;
		reset_move_sequence = true;
		reset_integrator = true;
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



static int regulator_speed(float angle, float angular_speed)
{
	// pid regulator
	// integrator
	static float integ = 0;
	if(reset_integrator)
	{
		integ = 0;
		reset_integrator = false;
	}
	//if(fabs(angle) > INTEG_THRESHOLD)
		integ += Ki*angle;
	// AWM
	if(integ > AWM_MAX)
		integ = AWM_MAX;
	else if (integ < AWM_MIN )
		integ = AWM_MIN;
	float deriv = 0;
	//if(fabs(angular_speed) > DERIV_THRESHOLD)
		deriv = Kd*(angular_speed);//-SIDE(angular_speed)*DERIV_THRESHOLD);
	chprintf((BaseSequentialStream *)&SD3, "%.1f, %d, %.1f;\n",Kp*angle,integ, deriv);
	return Kp*angle + integ + deriv;
}


static _Bool is_stable(float angle, float angular_speed)
{
	if(fabs(angle)<STABLE_ANGLE && fabs(angular_speed)<STABLE_SPEED)
		return true;
	else
		return false;
}

static void balance_mode(float angle, float angular_speed)
{
	set_motor_speed(regulator_speed(angle, angular_speed));
}

static void following_mode(float angle, float angular_speed)
{
	static int8_t current_movement = NONE;

	// update side (don't change side during sequence)
	if(current_movement == NONE)
	{
		uint16_t distance = get_target_distance();
		if(distance > MAX_TARGET_DISTANCE)
		{
			if(SIDE(angle) != BACK)
				current_movement = FRONT;
		}
		else if (distance < MIN_TARGET_DISTANCE)
		{
			if(SIDE(angle) != FRONT)
			current_movement = BACK;
		}
	}

	if(current_movement != NONE)
		current_movement = move(current_movement,angle,angular_speed);
	else // idle: stay stable
		balance_mode(angle,angular_speed);
}




static int8_t move(int8_t side, float angle, float angular_speed)
{
	static moving_sequence_t state = DONE;
	static int acc_speed = 0;
	if(reset_move_sequence)
	{
		state = DONE;
		reset_move_sequence = false;
	}
	switch(state)
	{
	case DONE:
		state = LEAN;
//		set_motor_speed(-side*LEAN_SPEED);
		reset_integrator = true;
		break;
	case LEAN:
		set_motor_speed(Kp*(angle-side*LEAN_ANGLE));
		if(fabs(angle) >= LEAN_ANGLE && SIDE(angle) == side)
		{
			state = LET_FALL;
			set_motor_speed(0);
		}
		break;
	case LET_FALL:
		if(fabs(angular_speed) >= FALL_SPEED  && SIDE(angle) == side)
		{
			state = ACCELERATE;
			//set_motor_speed(side*MOVING_SPEED);
		}
		break;
	case ACCELERATE:
		reset_integrator = true;
		if(fabs(acc_speed) >= fabs(regulator_speed(angle,angular_speed)))
		{
			state = MOVE;
			reset_integrator = true;
			acc_speed = 0;
		}
		else
		{
			acc_speed += side*MOVING_ACCELERATION;
			set_motor_speed(acc_speed);
		}
		break;
	case MOVE:
		balance_mode(angle, angular_speed);
		if(fabs(angle) <= STABLE_ANGLE )
		{
			state = DONE;
			side = NONE;
			set_motor_speed(0);
		}
		break;
	default:
		state = DONE;
	}

	return side;
}

static _Bool has_fallen(float angle, float speed)
{
	if(fabs(angle) < CRITICAL_ANGLE || (SIDE(angle) != SIDE(speed)))
		return false;
	else
		return true;
}

static void recover(void)
{
	set_motor_speed(0);
	chThdSleepMilliseconds(WAIT_UNTILL_DOWN);
	set_motor_speed(-SIDE(get_angle())*MOTOR_SPEED_LIMIT);
	chThdSleepMilliseconds(WAIT_SPEED_MAX);

	float angle = 0, angular_speed = 0;
	_Bool fail_to_recover = false;

	reset_integrator = true;


	angle = get_angle();
	angular_speed = get_angular_speed();

	while(fabs(angle) > STABLE_ANGLE)
	{
		angle = get_angle();
		angular_speed = get_angular_speed();

		// fail to recover : wait to be adjusted manually
		if( SIDE(angle)==SIDE(angular_speed) && !(fabs(angular_speed) < STABLE_ANGLE) )
		{
			fail_to_recover = true;
			set_motor_speed(0);
		}
		if(!fail_to_recover)
			set_motor_speed(SIDE(angle)*MOTOR_SPEED_LIMIT);
		chThdSleepMilliseconds(SLEEP_TIME);
	}
	// Avoid overshoot
	//reset_integrator = true;
}
