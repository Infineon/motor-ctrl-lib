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

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

#pragma once
#include "General.h"

#define SIX_PULSE	6	// Six pulses are applied in total
#define TWO_PULSE	2	// Two out of six pulses closest to rotor position

typedef enum
{
    Not_Started = 0U,
    In_Progress_On_Pulse,
    In_Progress_Off_Pulse,
    Processing_Results,
    Finished_Success,
    Finished_Ambiguous,
} SIX_PULSE_INJ_STATE_t;

typedef struct
{
    int8_t label[SIX_PULSE];
    float value[SIX_PULSE];
} SIX_PULSE_INJ_DATA_t;

typedef struct
{
    TIMER_t timer_on;
    TIMER_t timer_off;

    SIX_PULSE_INJ_STATE_t state;
    SIX_PULSE_INJ_DATA_t i_peak;
    SIX_PULSE_INJ_DATA_t k;	// k = k_num/k_den = exp(Ts/tau)
    float k_num, k_den;
    float d_pos, d_neg;

    int8_t pulse_num;
    int8_t labels_i_peak[TWO_PULSE];
    int8_t labels_k[TWO_PULSE];
    int8_t labels_unwrapped[TWO_PULSE];

    UVW_t i_uvw_sign;
    float i_fb, i_fb_prev;

    ELEC_t th_r_est;
} SIX_PULSE_INJ_t;

void SIX_PULSE_INJ_Init(MOTOR_t *motor_ptr);
void SIX_PULSE_INJ_Reset(MOTOR_t *motor_ptr);
void SIX_PULSE_INJ_RunISR0(MOTOR_t *motor_ptr);
void SIX_PULSE_INJ_RunISR1(MOTOR_t *motor_ptr);

#endif
