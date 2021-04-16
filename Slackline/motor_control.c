/*
 * motor_control.c
 *
 *  Created on: 15 avr. 2021
 *      Author: tille
 */

#include <motor_control.h>

#include <motors.h>

// pour la communication avec l'IMU

#include <sensors/imu.h>
#include <msgbus/messagebus.h>

static messagebus_t bus;
//messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values))
//chprintf((BaseSequentialStream *)&SDU1, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n\n", imu_values.acc_raw[0], imu_values.acc_raw[1], imu_values.acc_raw[2], imu_values.gyro_raw[0], imu_values.gyro_raw[1], imu_values.gyro_raw[2]);
//messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
//imu_msg_t imu_values;

static THD_WORKING_AREA(motor_control_thd_wa, 128);
static THD_FUNCTION(motor_control_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;

     while(1)
     {
    	 messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	 chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n\n", imu_values.acc_raw[0], imu_values.acc_raw[1], imu_values.acc_raw[2], imu_values.gyro_raw[0], imu_values.gyro_raw[1], imu_values.gyro_raw[2]);

     }
}

void motor_control_start(void)
{
	// setup motors
	motors_init();
	// setup IMU
	imu_start();
	calibrate_acc();
	calibrate_gyro();

	// launch the thread
	chThdCreateStatic(motor_control_thd_wa, sizeof(motor_control_thd_wa), NORMALPRIO, motor_control_thd, NULL);
}
