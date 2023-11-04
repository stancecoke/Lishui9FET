/*
Library for King-Meter displays

Copyright © 2015 Michael Fabry (Michael@Fabry.de)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef KINGMETER_H
#define KINGMETER_H


// Includes
#include "config.h"
#include "stdint.h"



// Definitions
#define KM_MAX_WHEELTIME 0x0DAC          // Maximum Wheeltime reported to the display (e.g. when wheel is stopped)



typedef struct
{
    // Parameters received from display in setting mode:
    uint16_t WheelSize_mm;              // Unit: 1mm
    uint8_t  DoubleGearRatio;         // KM_PASDIR_FORWARD / KM_PASDIR_BACKWARD
    uint8_t  PAS_SCN_Tolerance;         // Number of PAS signals to start the motor
    uint8_t  Gear_Ratio;               // Gear Ratio of Motor
    uint8_t  LegalFlag;           // KM_HND_HL_NO / KM_HND_HL_YES
    uint8_t  SS_Ext_Int;           // KM_HND_HF_NO / KM_HND_HF_YES
    uint8_t  SYS_SSP_SlowStart;         // 1..4 Level of soft ramping at start
    uint8_t  SPS_SpdMagnets;            // Number of magnets of speedsensor
    uint16_t VOL_1_UnderVolt_x10;       // Unit: 0.1V

}RX_SETTINGS_t;



#define KM_HEADLIGHT_OFF    0x00
#define KM_HEADLIGHT_ON     0x01
#define KM_HEADLIGHT_LOW    0x02
#define KM_HEADLIGHT_HIGH   0x03

#define KM_BATTERY_NORMAL   0x00
#define KM_BATTERY_LOW      0x01

#define KM_PUSHASSIST_OFF   0x00
#define KM_PUSHASSIST_ON    0x01

#define KM_POWERASSIST_OFF  0x00
#define KM_POWERASSIST_ON   0x01

#define KM_THROTTLE_OFF     0x00
#define KM_THROTTLE_ON      0x01

#define KM_CRUISE_OFF       0x00
#define KM_CRUISE_ON        0x01

#define KM_OVERSPEED_NO     0x00        // Speed below limit
#define KM_OVERSPEED_YES    0x01        // Overspeed detected


typedef struct
{
    // Parameters received from display in operation mode:
    uint8_t  AssistLevel;               // 0..255 Power Assist Level
    uint8_t  Headlight;                 // KM_HEADLIGHT_OFF / KM_HEADLIGHT_ON / KM_HEADLIGHT_LOW / KM_HEADLIGHT_HIGH
    uint8_t  Battery;                   // KM_BATTERY_NORMAL / KM_BATTERY_LOW
    uint8_t  PushAssist;                // KM_PUSHASSIST_OFF / KM_PUSHASSIST_ON
    uint8_t  PowerAssist;               // KM_POWERASSIST_OFF / KM_POWERASSIST_ON
    uint8_t  Throttle;                  // KM_THROTTLE_OFF / KM_THROTTLE_ON
    uint8_t  CruiseControl;             // KM_CRUISE_OFF / KM_CRUISE_ON
    uint8_t  OverSpeed;                 // KM_OVERSPEED_OFF / KM_OVERSPEED_ON
    uint16_t SPEEDMAX_Limit;        	// Unit: km/h
    uint16_t CUR_Limit_mA;              // Unit: mA

}RX_PARAM_t;



#define KM_ERROR_NONE           0x00
#define KM_ERROR_COMM           0x30
#define KM_ERROR_OVHT			0x25
#define KM_ERROR_IOVHT			0x26

typedef struct
{
    // Parameters to be send to display in operation mode:
    uint8_t  Battery;                   // KM_BATTERY_NORMAL / KM_BATTERY_LOW
    uint16_t Wheeltime_ms;              // Unit:1ms
    uint8_t  Error;                     // KM_ERROR_NONE, ..
    uint16_t Current_x10;               // Unit: 0.1A

}TX_PARAM_t;





 #define KM_MAX_RXBUFF 64
 #define KM5S_NM_RXBUFF 15 // KM5S RX-Buffer length for normal mode
 #define KM_MAX_TXBUFF 13


typedef struct
{
    uint8_t         RxState;
    int8_t          DirectSetpoint;

    uint8_t         RxBuff[KM_MAX_RXBUFF];
    uint8_t         RxCnt;

    RX_SETTINGS_t   Settings;
    RX_PARAM_t      Rx;
    TX_PARAM_t      Tx;

}KINGMETER_t;




// Public function prototypes




void KingMeter_Service(KINGMETER_t* KM_ctx);

void KingMeter_Init (KINGMETER_t* KM_ctx);


#endif // KINGMETER_H

