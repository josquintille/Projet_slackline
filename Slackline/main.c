#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/proximity.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <motor_control.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    //usb_start();
    //starts the camera
    //dcmi_start();
	//po8030_start();
	//inits the motors
	motors_init();

	//start the TOF sensor
	VL53L0X_start();

	//start the 8 proximity sensors and calibrate
	proximity_start();
	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity"); //va en panic
	proximity_msg_t prox_values;
	calibrate_ir();

	//stars the threads for the pi regulator and the processing of the image
	motor_control_start();

    /* Infinite loop. */
    while (1) {
    	//read distance sensor and send via Bluetooth
    	chprintf((BaseSequentialStream *)&SD3, "DISTANCE SENSOR (TOF):\t");
    	chprintf((BaseSequentialStream *)&SD3, "%d\n\n", VL53L0X_get_dist_mm());

    	//read proximity sensors and send via USB (serial by wire)
    	chprintf((BaseSequentialStream *)&SD3, "PROXIMITY SENSOR 0:\t");
		chprintf((BaseSequentialStream *)&SD3, "%d\n", get_prox(0));

    	//waits 0.5 second
        chThdSleepMilliseconds(500);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
