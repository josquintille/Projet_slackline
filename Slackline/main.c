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

#define MODE_BALANCE 0
#define MODE_OBSTACLE 1


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
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	motor_control_start();

	systime_t time;
    /* Infinite loop. */
    while (1) {
    	time = chVTGetSystemTime();
    	static uint8_t current_mode = 0;

    	if(current_mode != get_selector())
    	{
    		current_mode = get_selector();
			switch (current_mode)
			{
			case MODE_BALANCE:
				set_control_mode(BALANCE);
				break;

			case MODE_OBSTACLE:
				set_control_mode(FOLLOW_TARGET);
				break;

			default: // mode balance
				set_control_mode(BALANCE);
				break;
			}
    	}

    	//waits 0.5 second
    	chThdSleepUntilWindowed(time, time + MS2ST(500));
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
