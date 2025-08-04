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

#include "PLL.h"
#include "Biquad.h"


// H(s)=s^2/(s^3+c1*s^2+c2*s+c3)*gain
// c1=(k1+k2+k3)*|w0|
// c2=(k1*k2+k1*k3+k2*k3)*|w0|^2
// c3=k1*k2*k3*|w0|^3
// th_p=atan(((k1+k2+k3)-(k1*k2*k3))/(1-(k1*k2+k1*k3+k2*k3)))
// th_p'=th_p*sign(w0)

typedef struct
{	// Shared between alpha/beta axes
    float gain;
    float c1;
    float c2;
    float c3;
} FLUX_FILTS_SHARED_t;

typedef struct
{	// One filter per alpha/beta axis
    BILINEAR_INTEG_t bilinear_1;
    BILINEAR_INTEG_t bilinear_2;
    BILINEAR_INTEG_t bilinear_3;
} FLUX_FILT_t;

void FLUX_FILT_Reset(MOTOR_t *motor_ptr,FLUX_FILT_t* flux_filt, const float la_lead_0);
float FLUX_FILT_Run(MOTOR_t *motor_ptr,FLUX_FILT_t* flux_filt, const float input);

typedef struct
{
    FLUX_FILT_t flux_filt_alpha;
    FLUX_FILT_t flux_filt_beta;
    FLUX_FILTS_SHARED_t flux_filts_shared;
    BIQUAD_t biquad_w;
    AB_t d_la_ab;
    AB_t la_ab_lead;
    AB_t la_ab_lead_adj;
    AB_t i_ab_lead;
    PLL_t pll_r;
#if defined(CTRL_METHOD_SFO)
    PLL_t pll_s;
#endif
    float w_filt_elec;
    float w_filt_elec_sign;
    PARK_t phase_comp;
} OBS_t;

extern OBS_t obs[MOTOR_CTRL_NO_OF_MOTOR];

void OBS_Init(MOTOR_t *motor_ptr);
void OBS_Reset(MOTOR_t *motor_ptr,AB_t* la_ab_lead, ELEC_t* w0, ELEC_t* th0);
void OBS_RunISR0(MOTOR_t *motor_ptr);
