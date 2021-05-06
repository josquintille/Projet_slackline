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



#define TIM2SEC(tim) 	tim/1e6

#define AXIS_OF_ANGLE 	X_AXIS
#define AXIS_GRAVITY 	Y_AXIS
#define AXIS_DOWN		Z_AXIS

#define OMEGA_N			0.1// cut frequency of the complementary filters
#define FILTER_FACTOR	0.99// exp(-OMEGA_N*THREAD_PERIODE)

#define THREAD_PERIODE 1 //[ms]
#define CORRECTION_GYRO 3.9

// Bus to communicate with the IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static float angle = 0;
static float angular_speed = 0;
//static float temp_raw_angle_acc = 0; // for debugging, to be deleted
//static float temp_raw_angle_gyro = 0; // for debugging, to be deleted

static void update_data(float acceleration[], float current_speed);

static THD_WORKING_AREA(orientation_thd_wa, 512);
static THD_FUNCTION(orientation_thd, arg)
{
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;

     systime_t time;
     //int i = 0;
     while(1)
     {
    	 time = chVTGetSystemTime();
    	 messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	 update_data(imu_values.acceleration, imu_values.gyro_rate[AXIS_OF_ANGLE]);

		 //prints values in readable units
		 /*chprintf((BaseSequentialStream *)&SD3, "%Ax=%.4f Ay=%.4f Az=%.4f Gx=%.4f Gy=%.4f Gz=%.4f (%x)\n\n",
				 imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
				 imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
				 imu_values.status);
		 //chprintf((BaseSequentialStream *)&SD3, "%angle_acc=%.4f\n",angle_acc*180/3.141592653 );
		//chprintf((BaseSequentialStream *)&SD3, "%angle_gyro=%.4f\n",angle_gyro*180/3.141592653 );
    	 if (++i>10)
    	 {
			 //chprintf((BaseSequentialStream *)&SD3, "%angle = %.4f\n",angle*180/3.141592653 );
			 //chprintf((BaseSequentialStream *)&SD3, "%angular speed = %.4f\n",angular_speed*180/3.141592653 );

    		 //chprintf((BaseSequentialStream *)&SD3, "%.4f, ",temp_raw_angle_acc*180/3.141592653 );
			 //chprintf((BaseSequentialStream *)&SD3, " %.4f,",temp_raw_angle_gyro*180/3.141592653 );
			 //chprintf((BaseSequentialStream *)&SD3, " %.4f;\n",angle*180/3.141592653 );

			 i = 0;
    	 }*/

		 // go to sleep
		 chThdSleepUntilWindowed(time, time + MS2ST(THREAD_PERIODE));
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
	//temp_raw_angle_acc = angle_acc_input;

	// apply low-pass filter to angle_acc
	static float angle_acc_f = 0; //previous acc angle
	angle_acc_f = FILTER_FACTOR*angle_acc_f + (1-FILTER_FACTOR)*angle_acc_input;

	// angle from gyro (timer used to calculate dt)
	static float angle_gyro = 0;
	float angle_gyro_prev = angle_gyro;
	angle_gyro += current_speed * CORRECTION_GYRO * THREAD_PERIODE/1000;
	//temp_raw_angle_gyro = angle_gyro;

	// apply high-pass complementary filter to angle_gyro
	static float angle_gyro_f = 0; //previous gyro angle, filtered
	angle_gyro_f = FILTER_FACTOR*(angle_gyro_f+angle_gyro-angle_gyro_prev);


	// update angular speed
	angular_speed = current_speed;

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

