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
 * @file SpeedCtrl.c
 * @brief Speed controller implementation
 *
 * Implements speed control with PI feedback and feed-forward compensation
 * for multiple control methods (RFO, SFO, TBC).
 */

#include "Controller.h"

/**
 * @brief Initialize speed controller
 *
 * Configures PI controller parameters based on the active control method.
 * - RFO: Output is current command
 * - SFO: Output is torque command
 * - TBC: Output is current command with bypass option
 *
 * @param motor_ptr Pointer to motor instance
 */
void SPEED_CTRL_Init(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_TBC)
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
#endif

#if defined(CTRL_METHOD_RFO)
    PI_UpdateParams(&ctrl_ptr->speed.pi, params_ptr->ctrl.speed.kp, params_ptr->ctrl.speed.ki * params_ptr->sys.samp.ts1, -params_ptr->motor.i_peak, params_ptr->motor.i_peak);
#elif defined(CTRL_METHOD_SFO)
    PI_UpdateParams(&ctrl_ptr->speed.pi, params_ptr->ctrl.speed.kp, params_ptr->ctrl.speed.ki * params_ptr->sys.samp.ts1, -params_ptr->motor.T_max, params_ptr->motor.T_max);
#elif defined(CTRL_METHOD_TBC)
    ctrl_ptr->speed.pi.output_max = (params_ptr->ctrl.curr.bypass == false) ? SQRT_TWO * params_ptr->motor.i_peak : SQRT_TWO * params_ptr->ctrl.curr.v_max / params_ptr->motor.r;
    ctrl_ptr->speed.pi.output_min = -ctrl_ptr->speed.pi.output_max;
    PI_UpdateParams(&ctrl_ptr->speed.pi, params_ptr->ctrl.speed.kp, params_ptr->ctrl.speed.ki * params_ptr->sys.samp.ts1, ctrl_ptr->speed.pi.output_min, ctrl_ptr->speed.pi.output_max);
#endif
}

/**
 * @brief Reset speed controller state
 *
 * Resets the PI controller integrator and error states.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SPEED_CTRL_Reset(MOTOR_t *motor_ptr)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PI_Reset(&ctrl_ptr->speed.pi);
}

/**
 * @brief Calculate feed-forward compensation terms
 *
 * Computes inertia, friction, and viscous damping compensation based on
 * commanded acceleration and speed.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SPEED_CTRL_CalcFeedForwards(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    ctrl_ptr->speed.ff_inertia = params_ptr->ctrl.speed.ff_k_inertia * vars_ptr->acc_cmd_int.elec;
    ctrl_ptr->speed.ff_friction = params_ptr->ctrl.speed.ff_k_friction * SIGN(vars_ptr->w_cmd_int.elec);
    ctrl_ptr->speed.ff_viscous = params_ptr->ctrl.speed.ff_k_viscous * vars_ptr->w_cmd_int.elec;
    ctrl_ptr->speed.ff_total = ctrl_ptr->speed.ff_inertia + ctrl_ptr->speed.ff_friction + ctrl_ptr->speed.ff_viscous;
}

/**
 * @brief Perform integrator back-calculation for anti-windup
 *
 * Updates the PI integrator using back-calculation method to prevent windup
 * when the output is saturated.
 *
 * @param motor_ptr Pointer to motor instance
 * @param cmd Command value (i_cmd_spd or T_cmd_spd depending on control method)
 */
void SPEED_CTRL_IntegBackCalc(MOTOR_t *motor_ptr, const float cmd)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    SPEED_CTRL_CalcFeedForwards(motor_ptr);
    PI_IntegBackCalc(&ctrl_ptr->speed.pi, cmd, vars_ptr->w_cmd_int.elec - vars_ptr->w_final_filt.elec, ctrl_ptr->speed.ff_total);
}

/**
 * @brief Execute speed controller in ISR (slow control loop)
 *
 * Runs the speed control loop:
 * - Calculates feed-forward terms
 * - Executes PI controller
 * - Outputs current or torque command based on control method
 *
 * @param motor_ptr Pointer to motor instance
 */
RAMFUNC_BEGIN
void SPEED_CTRL_RunISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    // Feed forwards
    SPEED_CTRL_CalcFeedForwards(motor_ptr);
    // PI
    PI_Run(&ctrl_ptr->speed.pi, vars_ptr->w_cmd_int.elec, vars_ptr->w_final_filt.elec, ctrl_ptr->speed.ff_total);
    // Output
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    vars_ptr->i_cmd_spd = ctrl_ptr->speed.pi.output;
#elif defined(CTRL_METHOD_SFO)
    vars_ptr->T_cmd_spd = ctrl_ptr->speed.pi.output;
#endif
}
RAMFUNC_END
