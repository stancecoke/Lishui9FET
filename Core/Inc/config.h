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

#define P_FACTOR 5000
#define I_FACTOR 5000

#define PERIOD 3000 //@48MHz for 16kHz PWM frequency
#define THROTTLE_OFFSET 620
#define THROTTLE_MAX 2820
#define BATTERY_CURRENT_MAX 1500

#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 60
#define PULSES_PER_REVOLUTION 1
#define SPEEDSOURCE INTERNAL

#define PAS_TIMEOUT 8000 // half a second
#define RAMP_END 3000

#endif /* INC_CONFIG_H_ */
