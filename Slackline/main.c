#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <selector.h>

#include "motor_control.h"

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

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	motor_control_start();
	//process_image_start();

	systime_t time;
    /* Infinite loop. */
    while (1) {
    	time = chVTGetSystemTime();

    	switch(get_selector()) {
			case 0: // Mode balance
				//code
				chprintf((BaseSequentialStream *)&SD3, "Mode balance\n");
				break;

			case 1: // Mode obstacle
				// code
				chprintf((BaseSequentialStream *)&SD3, "Mode obstacle\n");
				break;

			default : // Does nothing
				chprintf((BaseSequentialStream *)&SD3, "Mode default\n");
				break;
    	}

    	//waits 1 second
    	chThdSleepUntilWindowed(time, time + MS2ST(500));
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
