/*
 * orientation.h
 *
 *  Created on: 17 avr. 2021
 *      Author: tille
 */

#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include <stdint.h>

void orientation_start(void);

int16_t get_angle(void);
int16_t get_angular_speed(void);

#endif /* ORIENTATION_H_ */
