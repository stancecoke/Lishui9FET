/*
 * config.h
 *
 *  Created on: 30.09.2023
 *      Author: stancecoke
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define EXTERNAL 1
#define INTERNAL 0
#define CAL_I 38

#define P_FACTOR 500
#define I_FACTOR 200

#define PERIOD 3000 //@48MHz for 16kHz PWM frequency
#define THROTTLE_OFFSET 100
#define THROTTLE_MAX 3000
#define BATTERY_CURRENT_MAX 1500

#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 11
#define PULSES_PER_REVOLUTION 1
#define SPEEDSOURCE INTERNAL

#endif /* INC_CONFIG_H_ */
