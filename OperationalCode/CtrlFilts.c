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

/**
 * @file CtrlFilts.c
 * @brief Control filters implementation
 *
 * Implements filtering functions for speed and acceleration signals. The speed
 * anti-resonant filter removes mechanical resonances from the feedback signal,
 * while the acceleration estimator provides feed-forward information for improved
 * dynamic response.
 */

#include "Controller.h"

/**
 * @brief Initialize control filters
 *
 * Configures the speed anti-resonant filter and acceleration estimator based on
 * parameter settings. The anti-resonant filter uses configured zero and pole
 * frequencies, while the acceleration estimator uses a resonant filter with Q=1.
 *
 * @param motor_ptr Pointer to motor structure
 */
void CTRL_FILTS_Init(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    BIQUAD_PoleZeroInit(&ctrl_ptr->filt.spd_ar, params_ptr->sys.samp.fs1, 1.0f, params_ptr->filt.spd_ar_wz, params_ptr->filt.spd_ar_wp);
    float acc_w0_squared = POW_TWO(params_ptr->filt.acc_w0);
    // H(s)=(w0^2*s)/(s^2+w0*s+w0^2): Q=1; BW=w0/Q=w0; s->0: H(s)=s
    RESONANT_UpdateParams(&ctrl_ptr->filt.acc, acc_w0_squared, params_ptr->filt.acc_w0, acc_w0_squared, params_ptr->sys.samp.ts1);
}

/**
 * @brief Reset control filters to initial state
 *
 * Clears all filter states and associated variables to zero.
 *
 * @param motor_ptr Pointer to motor structure
 */
void CTRL_FILTS_Reset(MOTOR_t *motor_ptr)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    BIQUAD_Reset(&ctrl_ptr->filt.spd_ar, 0.0f);
    RESONANT_Reset(&ctrl_ptr->filt.acc);
    vars_ptr->w_final_filt.elec = 0.0f;
    vars_ptr->w_final_filt_abs.elec = 0.0f;
    vars_ptr->acc_cmd_int.elec = 0.0f;
}

/**
 * @brief Run speed anti-resonant filter (ISR1)
 *
 * Applies the speed anti-resonant filter to the final speed feedback signal
 * if enabled. Also computes the absolute value of the filtered speed.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
void CTRL_FILTS_RunSpeedISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    // Speed anti-resonant filter
    vars_ptr->w_final_filt.elec = (params_ptr->filt.spd_ar_en == En) ? BIQUAD_Run(&ctrl_ptr->filt.spd_ar, vars_ptr->w_final.elec) : vars_ptr->w_final.elec;
    vars_ptr->w_final_filt_abs.elec = ABS(vars_ptr->w_final_filt.elec);
}
RAMFUNC_END

/**
 * @brief Run acceleration estimator filter (ISR1)
 *
 * Estimates acceleration from the speed command using a resonant filter.
 * The acceleration estimate is used for feed-forward in the control loop.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
void CTRL_FILTS_RunAccelISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    // Acceleration estimator filter
    vars_ptr->acc_cmd_int.elec = RESONANT_Run(&ctrl_ptr->filt.acc, vars_ptr->w_cmd_int.elec);
#if defined(PC_TEST)
    vars_ptr->test[28] = vars_ptr->acc_cmd_int.elec;
#endif
}
RAMFUNC_END

/**
 * @brief Run all control filters (ISR1)
 *
 * Executes both the speed filter and acceleration estimator in sequence.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
void CTRL_FILTS_RunAllISR1(MOTOR_t *motor_ptr)
{
    CTRL_FILTS_RunSpeedISR1(motor_ptr);
    CTRL_FILTS_RunAccelISR1(motor_ptr);
}
RAMFUNC_END
