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

// Incremental encoder can be any of the following three types
// All types will be eventually transformed into count-direction type by hw peripherals
// Direction: positive = a leads b = cw = 1; negative = a lags b = ccw = 0
typedef struct
{
    union // quadratue a & b type
    {
        struct
        {
            uint8_t b : 1;
            uint8_t a : 1;
        };
        uint8_t a_b : 2;
    };
    union // count & direction type
    {
        struct
        {
            uint8_t cnt : 1;
            uint8_t dir : 1;
        };
        uint8_t dir_cnt : 2;
    };
    union // clockwise & counter-clockwise type
    {
        struct
        {
            uint8_t ccw : 1;
            uint8_t cw : 1;
        };
        uint8_t cw_ccw : 2;
    };
} INC_ENC_SIGNAL_t;

typedef struct
{
    float per_cap_freq;		// [Hz], period capture frequency, set by hardware interface
    uint16_t per_cap;       // [ticks], captured period, updated by hardware interface
    float dir_cap;          // [], captured direction, +1 or -1
    int16_t pos_cap;        // [ticks], captured position

    int16_t pos_prev;       // [ticks], position, previous
    int16_t pos_delta;      // [ticks], position delta
    int16_t pos_mod;        // [ticks], position modulo

    TIMER_t zero_spd_timer;		    // [], for zero speed detection
    ADAP_TRACK_LOOP_t track_loop;   // [], adaptive tracking loop for fine angle tracking

    float w_conv_coeff;	 // [(Ra/sec-elec).(period-ticks)], speed conversion coefficient
    float th_conv_coeff; // [Ra-elec/(position-ticks)], angle conversion coefficient
    ELEC_t w_ffm_thresh; // [Ra/sec-elec], feed forward calculation method threshold
} INC_ENCODER_t;

extern INC_ENCODER_t inc_encoder;

void INC_ENCODER_Init();
void INC_ENCODER_Reset(ELEC_t th0);
void INC_ENCODER_RunISR0();