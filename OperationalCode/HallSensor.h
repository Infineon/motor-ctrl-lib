/*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#pragma once

#include "General.h"
#include "AdapTrackLoop.h"

typedef union
{
    struct
    {
        uint32_t u : 1;
        uint32_t v : 1;
        uint32_t w : 1;
        uint32_t reserved : 29;	// always 0
    };
    uint32_t uvw;
} UVW_SIGNAL_t;

typedef struct
{
    UVW_SIGNAL_t signal;		// [], updated by hardware interface
    UVW_SIGNAL_t signal_prev;	// []

    float per_cap_freq;		    // [Hz], period capture frequency, set by hardware interface
    uint32_t per_cap;		    // [ticks], captured period, updated by hardware interface
    
    TIMER_t zero_spd_timer;		    // [], for zero speed detection
    ADAP_TRACK_LOOP_t track_loop;   // [], adaptive tracking loop for fine angle tracking

    float w_sign;			    // [], speed sign
    float w_conv_coeff;		    // [(Ra/sec-elec).(period-ticks)], speed conversion coefficient
} HALL_SENS_t;


extern HALL_SENS_t hall[MOTOR_CTRL_NO_OF_MOTOR];

void HALL_SENSOR_Init(MOTOR_t *motor_ptr);	// hall[0].time_cap_freq to be set by hardware interface first
void HALL_SENSOR_Reset(MOTOR_t *motor_ptr);
void HALL_SENSOR_RunISR0(MOTOR_t *motor_ptr);

