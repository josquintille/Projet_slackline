/*
 * main.c
 *
 *  Created on: 15 avr. 2021
 *      Author: tille
 */
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <main.h>
#include <motors.h>
#include <selector.h>
#include <leds.h>

#include "motor_control.h"

#define MODE_BALANCE	1
#define MODE_OBSTACLE	0
#define ILLEGAL_MODE	16

#define LED_ON	1
#define LED_OFF	0


void display_led(uint8_t mode)
{
	uint8_t body_value = 0, red_value = 0;

	switch(mode)
	{
	case MODE_BALANCE:
		body_value = LED_ON;
		red_value = LED_OFF;
		break;

	case MODE_OBSTACLE:
		body_value = LED_OFF;
		red_value = LED_ON;
		break;

	default : // mode balance
		body_value = LED_ON;
		red_value = LED_OFF;
		break;
	}

	set_body_led(body_value);
	set_led(LED1, red_value);
	set_led(LED3, red_value);
	set_led(LED5, red_value);
	set_led(LED7, red_value);
	set_front_led(red_value);
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

	//stars the threads for the pi regulator and the processing of the image
	motor_control_start();

	systime_t time;
    /* Infinite loop. */
    while (true) {
    	time = chVTGetSystemTime();
    	static uint8_t current_mode = ILLEGAL_MODE;

    	if(current_mode != get_selector())
    	{
    		current_mode = get_selector();
			display_led(current_mode);
			switch(current_mode)
			{
			case MODE_OBSTACLE:
				set_control_mode(FOLLOW_TARGET);
				break;

			case MODE_BALANCE:
				set_control_mode(BALANCE);
				break;

			default : // mode balance
				set_control_mode(BALANCE);
				break;
			}
    	}
    	//waits 0.05 second
    	chThdSleepUntilWindowed(time, time + MS2ST(50));
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
