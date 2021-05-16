/*
 * orientation.c
 *
 *	Communicate with the IMU to get the relative orientation of the e-puck
 *	Find the angle between the vertical and the z axis in the z-y plane
 *
 *  Created on: 17 avr. 2021
 *      Author: tille
 */
#include <msgbus/messagebus.h>
#include <sensors/imu.h>

#include <math.h>



#define MS2S(tim) 	tim/1e3

#define AXIS_OF_ANGLE 	X_AXIS
#define AXIS_GRAVITY 	Y_AXIS
#define AXIS_DOWN		Z_AXIS

#define CUTOFF_FREQ_FILTER	2.5133// [rad/s]
#define COMP_FILTER_FACTOR	0.99// exp(-CUTOFF_FREQ_FILTER*THREAD_PERIOD)
#define SPEED_FILTER_FACTOR	0.96

#define THREAD_PERIOD	4 //[ms]

// Bus to communicate with the IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static float angle = 0;
static float angular_speed = 0;

/*
 * update angle and angular_speed from the imu data
 */
static void update_data(float acceleration[], float current_speed);

static THD_WORKING_AREA(orientation_thd_wa, 512);
static THD_FUNCTION(orientation_thd, arg)
{
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;

     systime_t time;
     while(true)
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
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	imu_start();
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
	angle_acc_f = COMP_FILTER_FACTOR*angle_acc_f + (1-COMP_FILTER_FACTOR)*angle_acc_input;


	// apply high-pass complementary filter and integrator to angle_gyro
	static float angle_gyro_f = 0; //previous gyro angle, filtered
	angle_gyro_f = COMP_FILTER_FACTOR*angle_gyro_f + (1-COMP_FILTER_FACTOR)/CUTOFF_FREQ_FILTER*current_speed;


	// update angle
	angle = angle_acc_f + angle_gyro_f;

	// filter angular speed
	angular_speed = SPEED_FILTER_FACTOR*angular_speed + (1-SPEED_FILTER_FACTOR)*current_speed;
}


float get_angle(void)
{
	return angle;
}

float get_angular_speed(void)
{
	return angular_speed;
}

