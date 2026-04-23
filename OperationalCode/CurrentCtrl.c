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
 * @file CurrentCtrl.c
 * @brief Current controller implementation
 *
 * Implements current control for RFO (rotor field-oriented) and TBC (trapezoidal
 * block commutation) methods. RFO uses decoupled QD control with cross-coupling
 * feed-forward compensation. TBC uses scalar current control with optional bypass mode.
 */

#include "Controller.h"

#if defined(CTRL_METHOD_RFO)

/**
 * @brief Initialize current controller (RFO)
 *
 * Configures Q and D axis PI controllers with bandwidth reduction coefficient
 * and enables feed-forward compensation for cross-coupling terms.
 *
 * @param motor_ptr Pointer to motor structure
 * @param bw_red_coeff Bandwidth reduction coefficient
 */
void CURRENT_CTRL_Init(MOTOR_t *motor_ptr, const float bw_red_coeff)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    PI_UpdateParams(&ctrl_ptr->curr.pi_q, params_ptr->ctrl.curr.kp.q * bw_red_coeff, params_ptr->ctrl.curr.ki.q * bw_red_coeff * params_ptr->sys.samp.ts0, -params_ptr->ctrl.curr.v_max.q, params_ptr->ctrl.curr.v_max.q);
    PI_UpdateParams(&ctrl_ptr->curr.pi_d, params_ptr->ctrl.curr.kp.d * bw_red_coeff, params_ptr->ctrl.curr.ki.d * bw_red_coeff * params_ptr->sys.samp.ts0, -params_ptr->ctrl.curr.v_max.d, params_ptr->ctrl.curr.v_max.d);
    ctrl_ptr->curr.en_ff = true;
};

/**
 * @brief Reset current controller (RFO)
 *
 * Resets the Q and D axis PI controller states.
 *
 * @param motor_ptr Pointer to motor structure
 */
void CURRENT_CTRL_Reset(MOTOR_t *motor_ptr)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    PI_Reset(&ctrl_ptr->curr.pi_q);
    PI_Reset(&ctrl_ptr->curr.pi_d);
}

/**
 * @brief Run current controller (ISR0, RFO)
 *
 * Executes the RFO current control loop:
 * - Transforms currents to QD frame via Park transform
 * - Calculates flux linkages
 * - Applies cross-coupling feed-forward compensation
 * - Runs Q and D axis PI controllers
 * - Transforms voltage commands back to AB frame
 * - Adds high frequency injection if enabled
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
void CURRENT_CTRL_RunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    if (!sm_ptr->vars.high_freq.used) // handled by high freqeuncy injection module due to required frequency spectrum separation
    {
        // Current rotation
        ParkInit(vars_ptr->th_r_final.elec, &vars_ptr->park_r);
        ParkTransform(&vars_ptr->i_ab_fb, &vars_ptr->park_r, &vars_ptr->i_qd_r_fb);
    }

    // Flux calculations
    vars_ptr->la_qd_r_est.q = params_ptr->motor.lq * vars_ptr->i_qd_r_fb.q;
    vars_ptr->la_qd_r_est.d = params_ptr->motor.ld * vars_ptr->i_qd_r_fb.d + params_ptr->motor.lam;

    // Feed forwards
    if (ctrl_ptr->curr.en_ff)
    {
        ctrl_ptr->curr.ff.q = vars_ptr->la_qd_r_est.d * vars_ptr->w_final.elec * params_ptr->ctrl.curr.ff_coef;
        ctrl_ptr->curr.ff.d = -vars_ptr->la_qd_r_est.q * vars_ptr->w_final.elec * params_ptr->ctrl.curr.ff_coef;
    }
    else
    {
        ctrl_ptr->curr.ff = QD_Zero;
    }

    // PIs
    PI_Run(&ctrl_ptr->curr.pi_q, vars_ptr->i_qd_r_cmd.q, vars_ptr->i_qd_r_fb.q, ctrl_ptr->curr.ff.q);
    PI_Run(&ctrl_ptr->curr.pi_d, vars_ptr->i_qd_r_cmd.d, vars_ptr->i_qd_r_fb.d, ctrl_ptr->curr.ff.d);
    vars_ptr->v_qd_r_cmd.q = ctrl_ptr->curr.pi_q.output;
    vars_ptr->v_qd_r_cmd.d = ctrl_ptr->curr.pi_d.output;

    // Voltage derotation
    ParkTransformInv(&vars_ptr->v_qd_r_cmd, &vars_ptr->park_r, &vars_ptr->v_ab_cmd);
    vars_ptr->v_ab_cmd_tot = vars_ptr->v_ab_cmd;
    if (sm_ptr->vars.high_freq.used) // adding high frequency excitation components
    {
        vars_ptr->v_ab_cmd_tot.alpha += ctrl_ptr->high_freq_inj.v_ab_cmd.alpha;
        vars_ptr->v_ab_cmd_tot.beta += ctrl_ptr->high_freq_inj.v_ab_cmd.beta;
    }
}
RAMFUNC_END

#elif defined(CTRL_METHOD_TBC)

/**
 * @brief Initialize current controller (TBC)
 *
 * Configures the scalar PI controller for TBC current control.
 * Feed-forward is enabled only for PC testing.
 *
 * @param motor_ptr Pointer to motor structure
 */
void CURRENT_CTRL_Init(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    PI_UpdateParams(&ctrl_ptr->curr.pi, params_ptr->ctrl.curr.kp, params_ptr->ctrl.curr.ki * params_ptr->sys.samp.ts0, -params_ptr->ctrl.curr.v_max, params_ptr->ctrl.curr.v_max);
#if defined(PC_TEST)
    ctrl_ptr->curr.en_ff = true;
#else
    ctrl_ptr->curr.en_ff = false;
#endif
};

/**
 * @brief Reset current controller (TBC)
 *
 * Resets the PI controller state.
 *
 * @param motor_ptr Pointer to motor structure
 */
void CURRENT_CTRL_Reset(MOTOR_t *motor_ptr)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    PI_Reset(&ctrl_ptr->curr.pi);
}

/**
 * @brief Run current controller (ISR0, TBC)
 *
 * Executes TBC current control with optional bypass mode.
 * Applies back-EMF feed-forward compensation when enabled.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
void CURRENT_CTRL_RunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    // Feed forward
    ctrl_ptr->curr.ff = ctrl_ptr->curr.en_ff ? (params_ptr->motor.lam * vars_ptr->w_cmd_int.elec * params_ptr->ctrl.curr.ff_coef) : 0.0f;

    if (params_ptr->ctrl.curr.bypass)
    {
        vars_ptr->v_s_cmd.rad = params_ptr->ctrl.curr.k_bypass * vars_ptr->i_cmd_int + ctrl_ptr->curr.ff;
    }
    else
    {
        // PI
        PI_Run(&ctrl_ptr->curr.pi, vars_ptr->i_cmd_int, vars_ptr->i_s_fb, ctrl_ptr->curr.ff);
        vars_ptr->v_s_cmd.rad = ctrl_ptr->curr.pi.output;
    }
}
RAMFUNC_END

#endif
