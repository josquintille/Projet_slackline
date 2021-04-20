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


#define AXIS_OF_ANGLE 	X_AXIS
#define TIM2SEC(tim) 	tim/1e6

// Bus to communicate with the IMU
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static float angle = 0;

static void timer11_start(void);
static void update_angle(float current_speed);

static THD_WORKING_AREA(orientation_thd_wa, 512);
static THD_FUNCTION(orientation_thd, arg)
{
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
     imu_msg_t imu_values;
     while(1)
     {
    	 messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	 update_angle(imu_values.gyro_rate[AXIS_OF_ANGLE]);

		 //prints values in readable units
		/* chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n",
				 imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
				 imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
				 imu_values.status);*/
		 chprintf((BaseSequentialStream *)&SD3, "%angle=%.4f\n",angle*180/3.141592653 );
		 chThdSleepMilliseconds(10);
     }
}
void orientation_start(void)
{
	// timer
	timer11_start();
	// setup IMU
	imu_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	calibrate_acc();
	calibrate_gyro();

	// launch the thread
	chThdCreateStatic(orientation_thd_wa, sizeof(orientation_thd_wa), NORMALPRIO, orientation_thd, NULL);
}



// integrate the angular speed to get the angle

/*
 * will update the static variable "angle" based on the time integrale of
 * the angular velocity
 * 	input : angular_speed in the wanted direction [rad/s]
 */
static void update_angle(float current_speed)
{
	static float previous_speed = 0;

	// calculate dt
	chSysLock();
	volatile uint16_t time = GPTD11.tim->CNT;
	// integrate with triangle methode
	angle += 0.5*(previous_speed+current_speed)*TIM2SEC(time);
	GPTD11.tim->CNT = 0;
// modifeir : utiliser sleepUntilwindowd !!!
	chSysUnlock();

	previous_speed = current_speed;
}
static void timer11_start(void)
{
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}

float get_angle(void)
{
	return angle;
}

