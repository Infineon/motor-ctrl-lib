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

#include "Controller.h"


void ADAP_TRACK_LOOP_SetSpeedFeedForwardForAdapGain(ADAP_TRACK_LOOP_t* atl_instance, ELEC_t* w_ff_adap_gain)
{
    atl_instance->w_ff_adap_gain = w_ff_adap_gain;
}

void ADAP_TRACK_LOOP_Init(ADAP_TRACK_LOOP_t* atl_instance, ADAP_TRACK_LOOP_PARAMS_t* atl_params,float sample_time,float motor_poles)
{
    atl_instance->w_ff_coeff = atl_params->w0_w *sample_time;
    atl_instance->w_fb_coeff = ONE_OVER_TWO_PI * (float)(atl_params->cpr) / (0.5f * motor_poles * atl_params->tau_ratio);
    atl_instance->w0_th = atl_params->w0_th;
    ADAP_TRACK_LOOP_SetSpeedFeedForwardForAdapGain(atl_instance, &atl_instance->w_ff_filt);
}

void ADAP_TRACK_LOOP_Reset(ADAP_TRACK_LOOP_t* atl_instance, ELEC_t th0)
{
    atl_instance->th_r_coarse.elec = th0.elec;
    atl_instance->th_r_coarse_prev.elec = th0.elec;
    atl_instance->w_ff.elec = 0.0f;

    atl_instance->w_ff_filt.elec = 0.0f;
    atl_instance->w_tot.elec = 0.0f;
    atl_instance->th_r_est.elec = th0.elec;
}

RAMFUNC_BEGIN
void ADAP_TRACK_LOOP_RunISR0(ADAP_TRACK_LOOP_t* atl_instance,float sample_time)
{
    // Velocity feed forward filter
    atl_instance->w_ff_filt.elec += (atl_instance->w_ff.elec - atl_instance->w_ff_filt.elec) * atl_instance->w_ff_coeff;

    // Angle estimation loop
    atl_instance->adaptive_gain = SAT(atl_instance->w0_th.min, atl_instance->w0_th.max, atl_instance->w_fb_coeff * ABS(atl_instance->w_ff_adap_gain->elec));
    atl_instance->w_tot.elec = atl_instance->w_ff_filt.elec + atl_instance->adaptive_gain * Wrap2Pi(atl_instance->th_r_coarse.elec - atl_instance->th_r_est.elec);
    atl_instance->th_r_est.elec = Wrap2Pi(atl_instance->th_r_est.elec + sample_time * atl_instance->w_tot.elec);
}
RAMFUNC_END
