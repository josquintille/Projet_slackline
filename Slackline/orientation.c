/*
 * orientation.c
 *
 *	Communicate with the IMU to get the relative orientation of the e-puck
 *	Find the angle between the vertical and the z axis in the z-y plane
 *
 *  Created on: 17 avr. 2021
 *      Author: tille
 */
#define DEBUGING
#ifdef DEBUGING

	#include <ch.h>
	#include <chprintf.h>

#endif

#include <msgbus/messagebus.h>
#include <sensors/imu.h>

#include <math.h>



#define MS2S(tim) 	tim/1e3

#define AXIS_OF_ANGLE 	X_AXIS
#define AXIS_GRAVITY 	Y_AXIS
#define AXIS_DOWN		Z_AXIS

#define OMEGA_N			0.1// cut frequency of the complementary filters
#define FILTER_FACTOR	0.99// exp(-OMEGA_N*THREAD_PERIOD)

#define NB_SAMPLES_GYRO 10 //for the moving average filter


#define THREAD_PERIOD	4 //[ms]
// Bus to communicate with the IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static float angle = 0;
static float angular_speed = 0;

static void update_data(float acceleration[], float current_speed);

static THD_WORKING_AREA(orientation_thd_wa, 512);
static THD_FUNCTION(orientation_thd, arg)
{
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;

     systime_t time;
     while(1)
     {
    	 time = chVTGetSystemTime();
    	 messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    	 update_data(imu_values.acceleration, imu_values.gyro_rate[AXIS_OF_ANGLE]);

		  // go to sleep
		 chThdSleepUntilWindowed(time, time + MS2ST(THREAD_PERIOD));
     }
}
void orientation_start(void)
{
	// setup IMU
	imu_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	calibrate_acc();
	calibrate_gyro();

	// launch the thread (priority +1 for the integrator)
	chThdCreateStatic(orientation_thd_wa, sizeof(orientation_thd_wa), NORMALPRIO+1, orientation_thd, NULL);
}
static void update_data(float acceleration[], float current_speed)
{
	// angle from accelerometer (- because gyro and acc axes are not the same)
	float angle_acc_input = 0;
	angle_acc_input = -atan2(acceleration[AXIS_GRAVITY],-acceleration[AXIS_DOWN]);

	// apply low-pass filter to angle_acc
	static float angle_acc_f = 0; //previous acc angle
	angle_acc_f = FILTER_FACTOR*angle_acc_f + (1-FILTER_FACTOR)*angle_acc_input;


	// angle from gyro
	static float angle_gyro = 0;
	float angle_gyro_prev = angle_gyro;
	angle_gyro += current_speed * MS2S(THREAD_PERIOD);

	// apply high-pass complementary filter to angle_gyro
	static float angle_gyro_f = 0; //previous gyro angle, filtered
	angle_gyro_f = FILTER_FACTOR*(angle_gyro_f+angle_gyro-angle_gyro_prev);


	// apply moving average filter to current speed
	static float buff_angular_speeds[NB_SAMPLES_GYRO-1] = {0}; //-1 <-- current speed is not in buff
	static uint8_t buff_head = 0;
	// compute mean value
	angular_speed = current_speed;
	for(uint8_t i = 0; i <= NB_SAMPLES_GYRO-2; i++)
	{
		angular_speed += buff_angular_speeds[i];
	}
	angular_speed /= NB_SAMPLES_GYRO;
	// update buffer
	buff_angular_speeds[buff_head] = current_speed;
	buff_head = (buff_head >= NB_SAMPLES_GYRO-2) ? 0 : buff_head+1;


	// update angle
	angle = angle_acc_f + angle_gyro_f;
}


float get_angle(void)
{
	return angle;
}

float get_angular_speed(void)
{
	return angular_speed;
}

