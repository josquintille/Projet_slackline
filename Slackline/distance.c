/*
 * distance.c
 *
 *  Created on: 8 mai 2021
 *      Author: tille
 */

#include <sensors/VL53L0X/VL53L0X.h>

#include "distance.h"
#include "orientation.h"

#include <math.h>

#define OFFSET 34 //[mm]

void init_distance(void)
{
	VL53L0X_start();
}
uint16_t get_target_distance(void)
{
	return cos(get_angle())*(VL53L0X_get_dist_mm()-OFFSET);
}
