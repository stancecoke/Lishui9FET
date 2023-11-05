/*
 * config.h
 *
 *  Created on: 30.09.2023
 *      Author: stancecoke
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define CAL_I 38

#define P_FACTOR 200
#define I_FACTOR 10

#define PERIOD 3000 //@48MHz for 16kHz PWM frequency
#define THROTTLE_OFFSET 700
#define THROTTLE_MAX 2820
#define BATTERY_CURRENT_MAX 15000
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 76
#define PULSES_PER_REVOLUTION 1
#define SPEEDLIMIT 25
#define PAS_TIMEOUT 8000 // half a second
#define RAMP_END 3000
#define TS_COEF 6000
#define NCTE_OFFSET 1960
#define ERIDER_OFFSET 1500
#endif /* INC_CONFIG_H_ */
