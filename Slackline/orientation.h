/*
 * orientation.h
 *
 *  Created on: 17 avr. 2021
 *      Author: tille
 */

#ifndef ORIENTATION_H_
#define ORIENTATION_H_

void orientation_start(void);

/*
 * get the position and speed of the e-puck
 */
float get_angle(void);
float get_angular_speed(void);

#endif /* ORIENTATION_H_ */
