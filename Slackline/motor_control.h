/*
 * motor_control.h
 *
 *  Created on: 15 avr. 2021
 *      Author: tille
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

typedef enum control_mode_t {BALANCE, FOLLOW_TARGET} control_mode_t;
/*
 * initialize the thread
 */
void motor_control_start(void);
/*
 * set the mode:
 * in:
 *  FOLLOW_TARGET -> keep a distance of 10cm between the obstacle and the e-puck
 *  BALANCE -> keep the e-puck straight
 */
void set_control_mode(control_mode_t mode);

#endif /* MOTOR_CONTROL_H_ */
