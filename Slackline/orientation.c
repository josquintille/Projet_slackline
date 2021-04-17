/*
 * orientation.c
 *
 *	Communicate with the IMU to get the relative orientation of the e-puck
 *	Find the angle between the vertical and the z axis in the z-y plane
 *
 *  Created on: 17 avr. 2021
 *      Author: tille
 */


#include <sensors/imu.h>
#include <msgbus/messagebus.h>

#define DEBUGING

#ifdef DEBUGING

	#include <ch.h>
	#include <chprintf.h>

#endif

// Bus to communicate with the IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static THD_WORKING_AREA(orientation_thd_wa, 512);
static THD_FUNCTION(orientation_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_init(&bus, &bus_lock, &bus_condvar);
     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;
     while(1)
     {
    	 messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
		 //prints raw values
		 chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
				 imu_values.acc_raw[X_AXIS], imu_values.acc_raw[Y_AXIS], imu_values.acc_raw[Z_AXIS],
				 imu_values.gyro_raw[X_AXIS], imu_values.gyro_raw[Y_AXIS], imu_values.gyro_raw[Z_AXIS]);

		 //prints raw values with offset correction
		 chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
				 imu_values.acc_raw[X_AXIS]-imu_values.acc_offset[X_AXIS],
				 imu_values.acc_raw[Y_AXIS]-imu_values.acc_offset[Y_AXIS],
				 imu_values.acc_raw[Z_AXIS]-imu_values.acc_offset[Z_AXIS],
				 imu_values.gyro_raw[X_AXIS]-imu_values.gyro_offset[X_AXIS],
				 imu_values.gyro_raw[Y_AXIS]-imu_values.gyro_offset[Y_AXIS],
				 imu_values.gyro_raw[Z_AXIS]-imu_values.gyro_offset[Z_AXIS]);

		 //prints values in readable units
		 chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n",
				 imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
				 imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
				 imu_values.status);
     }
}
void orientation_start(void)
{
	// setup IMU
	imu_start();
	//calibrate_acc();
	//calibrate_gyro();

	// launch the thread
	chThdCreateStatic(orientation_thd_wa, sizeof(orientation_thd_wa), NORMALPRIO, orientation_thd, NULL);
}
