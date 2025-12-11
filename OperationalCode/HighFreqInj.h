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
#include "Biquad.h"
#include "PLL.h"

// INPUTS:
// vars.w_final_filt,		from vars.w_est
// vars.i_ab_fb_tot,

// OUTPUTS:
// ctrl.high_freq_inj.v_ab_cmd,	added to the main voltage components
// vars.w_est,
// vars.th_r_est,
// vars.th_s_est,			SFO
// vars.delta_est,			SFO
// vars.la_qd_s_est, 		SFO
// vars.park_r,
// vars.park_s,				SFO
// vars.i_qd_r_fb,
// vars.i_qd_s_fb,			SFO
// vars.i_ab_fb

typedef struct
{
    ELEC_t th_h;
    PARK_t park_h;

    BIQUAD_t lpf_i_q_r;
    BIQUAD_t lpf_i_d_r;

    QD_t i_qd_r_fb_tot;
    QD_t i_qd_r_fb_hf;
    QD_t i_qd_r_fb_demod;
} HIGH_FREQ_INJ_SIN_t;


typedef struct
{
    TIMER_t wave_gen_timer;
    float wave;
    
    AB_t i_ab_fb_prev;      // previous
    AB_t d_i_ab_fb;         // delta
    AB_t d_i_ab_fb_prev;    // delta, previous
    AB_t dd_i_ab_fb;        // delta-delta
    QD_t dd_i_qd_r_fb;      // delta-delta

    INTEG_DUMP_FILT_t pll_notch;
} HIGH_FREQ_INJ_SQR_t;

typedef struct
{
    // Type-specific variables (sine wave or square wave) ......
    HIGH_FREQ_INJ_SIN_t sine;
    HIGH_FREQ_INJ_SQR_t square;

    // Common variables ........................................
    PI_t pi_pll_r;
    BILINEAR_INTEG_t integ_pll_r;

    QD_t v_qd_r_cmd;
    AB_t v_ab_cmd;

#if defined(CTRL_METHOD_SFO)
    POLAR_t la_polar_r_est;
    PARK_t park_delta;
#endif
    // .........................................................
} HIGH_FREQ_INJ_t;

void HIGH_FREQ_INJ_Init(MOTOR_t* motor_ptr);
void HIGH_FREQ_INJ_Reset(MOTOR_t* motor_ptr, const ELEC_t w0, const ELEC_t th0);
void HIGH_FREQ_INJ_RunFiltISR0(MOTOR_t* motor_ptr); // Current separation filters
void HIGH_FREQ_INJ_RunCtrlISR0(MOTOR_t* motor_ptr);
void HIGH_FREQ_INJ_RunISR0(MOTOR_t* motor_ptr);

#endif
