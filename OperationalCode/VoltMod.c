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
 * @file VoltMod.c
 * @brief Voltage modulation implementation
 *
 * Implements space vector modulation (SVM), neutral point modulation (NPM),
 * and hybrid modulation strategies for three-phase inverter control.
 */

#include "Controller.h"
#if defined(PC_TEST)

static void (*NeutPointOrSpaceVectModISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = {EmptyFcn };   // Neutral point or space vector modulation (both single and three shunt)
static void (*HybridModRunISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = { EmptyFcn };   // Hybrid modulation for single shunt configuration
static void (*CurrReconstRunISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = {EmptyFcn };   // Calculating current-reconstruction sample times for single shunt configuration

#else
static void (*NeutPointOrSpaceVectModISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])(MOTOR_t*) = { [0 ... (MOTOR_CTRL_NO_OF_MOTOR - 1)] = EmptyFcn };   // Neutral point or space vector modulation (both single and three shunt)
static void (*HybridModRunISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])(MOTOR_t*) = { [0 ... (MOTOR_CTRL_NO_OF_MOTOR - 1)] = EmptyFcn };   // Hybrid modulation for single shunt configuration
static void (*CurrReconstRunISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])(MOTOR_t*) = { [0 ... (MOTOR_CTRL_NO_OF_MOTOR - 1)] = EmptyFcn };   // Calculating current-reconstruction sample times for single shunt configuration
#endif

/**
 * @brief Compute SVM sector, active-vector duty cycles, and modulation index (ISR0)
 *
 * Determines the space-vector sector (1-6) from the alpha-beta voltage command
 * using auxiliary sector variables (x, y, z). Calculates the normalised
 * first and second active-vector duty cycles (duty_first, duty_second) and
 * updates both the instantaneous modulation index (mi) and its filtered
 * value (mi_filt).
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void SVMCalcSectorDutyMIISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    float sector_variable_x = -vars_ptr->v_ab_cmd_tot.beta * TWO_OVER_SQRT_THREE;
    float sector_variable_y = vars_ptr->v_ab_cmd_tot.alpha + vars_ptr->v_ab_cmd_tot.beta * ONE_OVER_SQRT_THREE;
    float sector_variable_z = vars_ptr->v_ab_cmd_tot.alpha - vars_ptr->v_ab_cmd_tot.beta * ONE_OVER_SQRT_THREE;

    if (sector_variable_z >= 0.0f) // Sectors 1, 2 and 6
    {
        if (sector_variable_y >= 0.0f) // Sectors 1 and 6
        {
            if (sector_variable_x >= 0.0f) // Sector 1
            {
                ctrl_ptr->volt_mod.svm.v_first = sector_variable_y;
                ctrl_ptr->volt_mod.svm.v_second = sector_variable_x;
                ctrl_ptr->volt_mod.svm.sector = 1;
            }
            else // Sector 6
            {
                ctrl_ptr->volt_mod.svm.v_first = sector_variable_z;
                ctrl_ptr->volt_mod.svm.v_second = -sector_variable_x;
                ctrl_ptr->volt_mod.svm.sector = 6;
            }
        }
        else // Sector 2
        {
            ctrl_ptr->volt_mod.svm.v_first = -sector_variable_y;
            ctrl_ptr->volt_mod.svm.v_second = sector_variable_z;
            ctrl_ptr->volt_mod.svm.sector = 2;
        }
    }
    else // Sectors 3, 4 and 5
    {
        if (sector_variable_y <= 0.0f) // Sectors 3 and 4
        {
            if (sector_variable_x >= 0.0f) // Sector 3
            {
                ctrl_ptr->volt_mod.svm.v_first = sector_variable_x;
                ctrl_ptr->volt_mod.svm.v_second = -sector_variable_z;
                ctrl_ptr->volt_mod.svm.sector = 3;
            }
            else // Sector 4
            {
                ctrl_ptr->volt_mod.svm.v_first = -sector_variable_x;
                ctrl_ptr->volt_mod.svm.v_second = -sector_variable_y;
                ctrl_ptr->volt_mod.svm.sector = 4;
            }
        }
        else // Sector 5
        {
            ctrl_ptr->volt_mod.svm.v_first = -sector_variable_z;
            ctrl_ptr->volt_mod.svm.v_second = sector_variable_y;
            ctrl_ptr->volt_mod.svm.sector = 5;
        }
    }

    float max_voltage_inv = 1.5f * ctrl_ptr->volt_mod.v_dc_inv;
    ctrl_ptr->volt_mod.svm.duty_first = ctrl_ptr->volt_mod.svm.v_first * max_voltage_inv;
    ctrl_ptr->volt_mod.svm.duty_second = ctrl_ptr->volt_mod.svm.v_second * max_voltage_inv;
    ctrl_ptr->volt_mod.mi = vars_ptr->v_s_cmd.rad * max_voltage_inv;
    ctrl_ptr->volt_mod.mi_filt += (ctrl_ptr->volt_mod.mi - ctrl_ptr->volt_mod.mi_filt) * params_ptr->ctrl.volt.five_seg.w0_filt * params_ptr->sys.samp.ts0;
}
RAMFUNC_END

/**
 * @brief Run space vector modulation at ISR0 rate
 *
 * Calls SVMCalcSectorDutyMIISR0 to obtain the active sector and vector duties.
 * Handles overmodulation mode I by clamping when the total active duty exceeds
 * 1.0. Applies hysteretic 5-segment or 7-segment switching based on the
 * filtered modulation index. Maps active and zero vector duties to the
 * UVW phase compare registers and sets xyz/uvw index.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void SVMRunISR0(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    SVMCalcSectorDutyMIISR0(motor_ptr);

    float total_duty = ctrl_ptr->volt_mod.svm.duty_first + ctrl_ptr->volt_mod.svm.duty_second;
    if (total_duty <= 1.0f)
    {
        ctrl_ptr->volt_mod.svm.duty_zero = 1.0f - total_duty;
    }
    else // overmodulation mode I
    {
        ctrl_ptr->volt_mod.svm.duty_first = ctrl_ptr->volt_mod.svm.duty_first / total_duty;
        ctrl_ptr->volt_mod.svm.duty_second = 1.0f - ctrl_ptr->volt_mod.svm.duty_first;
        ctrl_ptr->volt_mod.svm.duty_zero = 0.0f;
        // TBD: Figuring out a way to avoid duty_zero = 0 for current sensing.
        // One way to deal with it could be defining the MI and going to overmodulation with minimum a defined on-time
        // for LS FET
    }

    if (params_ptr->ctrl.volt.five_seg.en == En)
    {
        // 5-segment vs 7-segment determination
        if (ctrl_ptr->volt_mod.mi_filt > params_ptr->ctrl.volt.five_seg.active_mi)
        {
            ctrl_ptr->volt_mod.svm.five_segment = true;
        }
        else if (ctrl_ptr->volt_mod.mi_filt < params_ptr->ctrl.volt.five_seg.inactive_mi)
        {
            ctrl_ptr->volt_mod.svm.five_segment = false;
        }

        // 5-segment vs 7-segment voltage application
        if (ctrl_ptr->volt_mod.svm.five_segment)
        {
            ctrl_ptr->volt_mod.svm.duty_zero *= 2.0f;
        }
    }

    /***************************************
     *    sector 1: 100 & 110        sector 2: 010 & 110         sector 3: 010 & 011         sector 4: 001 & 011 sector
     * 5: 001 & 101         sector 6: 100 & 101
     * __|--------------------|__  ______|------------|______  __________|----|__________  __________|----|__________
     * ______|------------|______  __|--------------------|__
     * ______|------------|______  __|--------------------|__  __|--------------------|__  ______|------------|______
     * __________|----|__________  __________|----|______
     * __________|----|__________  __________|----|__________  ______|------------|______  __|--------------------|__
     * __|--------------------|__  ______|------------|__________ v1  v2  v0  v2  v1          v1  v2  v0  v2  v1 v1  v2
     * v0  v2  v1          v1  v2  v0  v2  v1          v1  v2  v0  v2  v1          v1  v2  v0  v2  v1
     * ************************************/

    ctrl_ptr->volt_mod.uvw_idx_prev = ctrl_ptr->volt_mod.uvw_idx;
    switch (ctrl_ptr->volt_mod.svm.sector)
    {
    default:
    case 0x1: // sector 1
        vars_ptr->d_uvw_cmd.u = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.v = vars_ptr->d_uvw_cmd.u - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.w = vars_ptr->d_uvw_cmd.v - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(0U, 1U, 2U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(0U, 1U, 2U);

        break;
    case 0x2: // sector 2
        vars_ptr->d_uvw_cmd.v = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.u = vars_ptr->d_uvw_cmd.v - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.w = vars_ptr->d_uvw_cmd.u - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(1U, 0U, 2U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(1U, 0U, 2U);

        break;
    case 0x3: // sector 3
        vars_ptr->d_uvw_cmd.v = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.w = vars_ptr->d_uvw_cmd.v - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.u = vars_ptr->d_uvw_cmd.w - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(1U, 2U, 0U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(2U, 0U, 1U);

        break;
    case 0x4: // sector 4
        vars_ptr->d_uvw_cmd.w = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.v = vars_ptr->d_uvw_cmd.w - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.u = vars_ptr->d_uvw_cmd.v - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(2U, 1U, 0U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(2U, 1U, 0U);

        break;
    case 0x5: // sector 5
        vars_ptr->d_uvw_cmd.w = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.u = vars_ptr->d_uvw_cmd.w - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.v = vars_ptr->d_uvw_cmd.u - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(2U, 0U, 1U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(1U, 2U, 0U);

        break;
    case 0x6: // sector 6
        vars_ptr->d_uvw_cmd.u = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.w = vars_ptr->d_uvw_cmd.u - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.v = vars_ptr->d_uvw_cmd.w - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(0U, 2U, 1U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(0U, 2U, 1U);
        break;
    }
    vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
}
RAMFUNC_END

/**
 * @brief Run neutral point modulation at ISR0 rate
 *
 * Converts the alpha-beta voltage command to three-phase voltages via inverse
 * Clarke transform, then determines the neutral-point voltage offset as the
 * negative average of the maximum and minimum phase voltages. Adds the offset
 * to each phase voltage and converts the result to normalised duty cycles
 * saturated to [0, 1]. Updates the modulation index (mi).
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void NPMRunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    ClarkeTransformInv(&vars_ptr->v_ab_cmd_tot, &vars_ptr->v_uvw_n_cmd);

    ctrl_ptr->volt_mod.uvw_idx_prev = ctrl_ptr->volt_mod.uvw_idx;
    SortUVW(&vars_ptr->v_uvw_n_cmd, &ctrl_ptr->volt_mod.xyz_idx, &ctrl_ptr->volt_mod.uvw_idx);

    float *v_uvw_cmd = STRUCT_TO_ARRAY(vars_ptr->v_uvw_n_cmd);

    ctrl_ptr->volt_mod.npm.v_neutral = -0.5f * (v_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 0U)] + v_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 2U)]);

    vars_ptr->v_uvw_z_cmd.u = vars_ptr->v_uvw_n_cmd.u + ctrl_ptr->volt_mod.npm.v_neutral;
    vars_ptr->v_uvw_z_cmd.v = vars_ptr->v_uvw_n_cmd.v + ctrl_ptr->volt_mod.npm.v_neutral;
    vars_ptr->v_uvw_z_cmd.w = vars_ptr->v_uvw_n_cmd.w + ctrl_ptr->volt_mod.npm.v_neutral;

    vars_ptr->d_uvw_cmd.u = SAT(0.0f, 1.0f, vars_ptr->v_uvw_z_cmd.u * ctrl_ptr->volt_mod.v_dc_inv + 0.5f);
    vars_ptr->d_uvw_cmd.v = SAT(0.0f, 1.0f, vars_ptr->v_uvw_z_cmd.v * ctrl_ptr->volt_mod.v_dc_inv + 0.5f);
    vars_ptr->d_uvw_cmd.w = SAT(0.0f, 1.0f, vars_ptr->v_uvw_z_cmd.w * ctrl_ptr->volt_mod.v_dc_inv + 0.5f);

    vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;

    ctrl_ptr->volt_mod.mi = vars_ptr->v_s_cmd.rad * ctrl_ptr->volt_mod.v_dc_inv * 1.5f; // 2/3Vdc = 100% modulation
}
RAMFUNC_END

/**
 * @brief Run phase-shift SVM for single-shunt current sensing (ISR0)
 *
 * Computes asymmetric PWM compare values (independent rise and fall edges) to
 * guarantee a minimum active-vector on-time (adc_d_min) required for single-shunt
 * ADC sampling. Calculates duty_first_shift and duty_second_shift to widen any
 * active vector shorter than adc_d_min, then rescales both active duties if
 * their total would exceed the available period. Maps per-sector rise and fall
 * compare values to d_uvw_cmd and d_uvw_cmd_fall. Supports 5-segment and
 * 7-segment switching.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void SVMPhaseShiftRunISR0(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    SVMCalcSectorDutyMIISR0(motor_ptr);

    if (ctrl_ptr->volt_mod.svm.duty_first < params_ptr->sys.analog.shunt.single_shunt.adc_d_min)
    {
        ctrl_ptr->volt_mod.svm.duty_first_shift = (params_ptr->sys.analog.shunt.single_shunt.adc_d_min - ctrl_ptr->volt_mod.svm.duty_first) * 0.5f;
    }
    else
    {
        ctrl_ptr->volt_mod.svm.duty_first_shift = 0.0f;
    }
    if (ctrl_ptr->volt_mod.svm.duty_second < (params_ptr->sys.analog.shunt.single_shunt.adc_d_min + ctrl_ptr->volt_mod.svm.duty_first_shift))
    {
        ctrl_ptr->volt_mod.svm.duty_second_shift = params_ptr->sys.analog.shunt.single_shunt.adc_d_min + ctrl_ptr->volt_mod.svm.duty_first_shift - ctrl_ptr->volt_mod.svm.duty_second;
    }
    else
    {
        ctrl_ptr->volt_mod.svm.duty_second_shift = 0.0f;
    }

    float total_duty = ctrl_ptr->volt_mod.svm.duty_first + ctrl_ptr->volt_mod.svm.duty_second;
    float available_duty = 1.0f - ctrl_ptr->volt_mod.svm.duty_first_shift - ctrl_ptr->volt_mod.svm.duty_second_shift;

    if (total_duty > available_duty)
    {
        ctrl_ptr->volt_mod.svm.duty_first = ctrl_ptr->volt_mod.svm.duty_first * available_duty / total_duty;
        ctrl_ptr->volt_mod.svm.duty_second = ctrl_ptr->volt_mod.svm.duty_second * available_duty / total_duty;
    }
    ctrl_ptr->volt_mod.svm.duty_zero = available_duty - ctrl_ptr->volt_mod.svm.duty_first - ctrl_ptr->volt_mod.svm.duty_second + 2 * ctrl_ptr->volt_mod.svm.duty_first_shift;

    if (params_ptr->ctrl.volt.five_seg.en == En)
    {
        // 5-segment vs 7-segment determination
        if (ctrl_ptr->volt_mod.mi_filt > params_ptr->ctrl.volt.five_seg.active_mi)
        {
            ctrl_ptr->volt_mod.svm.five_segment = true;
        }
        else if (ctrl_ptr->volt_mod.mi_filt < params_ptr->ctrl.volt.five_seg.inactive_mi)
        {
            ctrl_ptr->volt_mod.svm.five_segment = false;
        }

        // 5-segment vs 7-segment voltage application
        if (ctrl_ptr->volt_mod.svm.five_segment)
        {
            ctrl_ptr->volt_mod.svm.duty_zero = ctrl_ptr->volt_mod.svm.duty_zero * 2.0f - 2 * ctrl_ptr->volt_mod.svm.duty_first_shift;
        }
    }

    ctrl_ptr->volt_mod.uvw_idx_prev = ctrl_ptr->volt_mod.uvw_idx;

    float compare0_rise, compare0_fall, compare1_rise, compare1_fall, compare2_rise, compare2_fall;

    compare0_rise = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f + ctrl_ptr->volt_mod.svm.duty_first_shift;
    compare0_fall = compare0_rise - (2 * ctrl_ptr->volt_mod.svm.duty_first_shift);
    compare1_rise = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f - ctrl_ptr->volt_mod.svm.duty_first - ctrl_ptr->volt_mod.svm.duty_first_shift;
    compare1_fall = compare1_rise + (2 * ctrl_ptr->volt_mod.svm.duty_first_shift);
    compare2_rise = MAX((1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f - ctrl_ptr->volt_mod.svm.duty_first - ctrl_ptr->volt_mod.svm.duty_second - ctrl_ptr->volt_mod.svm.duty_second_shift),0);
    compare2_fall = MAX((compare2_rise + (2 * ctrl_ptr->volt_mod.svm.duty_second_shift)), 0);

    switch (ctrl_ptr->volt_mod.svm.sector)
    {
    default:
    case 0x1: // sector 1
        vars_ptr->d_uvw_cmd.u = compare0_rise;
        vars_ptr->d_uvw_cmd_fall.u = compare0_fall;
        vars_ptr->d_uvw_cmd.v = compare1_rise;
        vars_ptr->d_uvw_cmd_fall.v = compare1_fall;
        vars_ptr->d_uvw_cmd.w = compare2_rise;
        vars_ptr->d_uvw_cmd_fall.w = compare2_fall;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(0U, 1U, 2U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(0U, 1U, 2U);
        break;
    case 0x2: // sector 2
        vars_ptr->d_uvw_cmd.v = compare0_rise;
        vars_ptr->d_uvw_cmd_fall.v = compare0_fall;
        vars_ptr->d_uvw_cmd.u = compare1_rise;
        vars_ptr->d_uvw_cmd_fall.u = compare1_fall;
        vars_ptr->d_uvw_cmd.w = compare2_rise;
        vars_ptr->d_uvw_cmd_fall.w = compare2_fall;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(1U, 0U, 2U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(1U, 0U, 2U);
        break;
    case 0x3: // sector 3
        vars_ptr->d_uvw_cmd.v = compare0_rise;
        vars_ptr->d_uvw_cmd_fall.v = compare0_fall;
        vars_ptr->d_uvw_cmd.w = compare1_rise;
        vars_ptr->d_uvw_cmd_fall.w = compare1_fall;
        vars_ptr->d_uvw_cmd.u = compare2_rise;
        vars_ptr->d_uvw_cmd_fall.u = compare2_fall;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(1U, 2U, 0U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(2U, 0U, 1U);
        break;
    case 0x4: // sector 4
        vars_ptr->d_uvw_cmd.w = compare0_rise;
        vars_ptr->d_uvw_cmd_fall.w = compare0_fall;
        vars_ptr->d_uvw_cmd.v = compare1_rise;
        vars_ptr->d_uvw_cmd_fall.v = compare1_fall;
        vars_ptr->d_uvw_cmd.u = compare2_rise;
        vars_ptr->d_uvw_cmd_fall.u = compare2_fall;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(2U, 1U, 0U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(2U, 1U, 0U);
        break;
    case 0x5: // sector 5
        vars_ptr->d_uvw_cmd.w = compare0_rise;
        vars_ptr->d_uvw_cmd_fall.w = compare0_fall;
        vars_ptr->d_uvw_cmd.u = compare1_rise;
        vars_ptr->d_uvw_cmd_fall.u = compare1_fall;
        vars_ptr->d_uvw_cmd.v = compare2_rise;
        vars_ptr->d_uvw_cmd_fall.v = compare2_fall;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(2U, 0U, 1U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(1U, 2U, 0U);
        break;
    case 0x6: // sector 6
        vars_ptr->d_uvw_cmd.u = compare0_rise;
        vars_ptr->d_uvw_cmd_fall.u = compare0_fall;
        vars_ptr->d_uvw_cmd.w = compare1_rise;
        vars_ptr->d_uvw_cmd_fall.w = compare1_fall;
        vars_ptr->d_uvw_cmd.v = compare2_rise;
        vars_ptr->d_uvw_cmd_fall.v = compare2_fall;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(0U, 2U, 1U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(0U, 2U, 1U);
        break;
    }
}
RAMFUNC_END

/**
 * @brief Compute single-shunt current-reconstruction ADC sample points (ISR0)
 *
 * For hybrid-modulation single-shunt sensing, pre-computes two ADC trigger
 * instants (d_samp[0] and d_samp[1]) by blending adjacent sorted XYZ phase
 * duties with the current-sense settle ratio (cs_settle_raio). These values
 * are used by the hardware to schedule ADC conversions within the PWM period.
 * Replaced by EmptyFcn via the function pointer when three-shunt sensing is
 * active.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
void CurrReconstRunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    float* d_uvw_cmd = STRUCT_TO_ARRAY(vars_ptr->d_uvw_cmd);
    vars_ptr->d_samp[1] = params_ptr->sys.analog.shunt.cs_settle_raio * d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 0U)] + (1.0f - params_ptr->sys.analog.shunt.cs_settle_raio) * d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 1U)];
    vars_ptr->d_samp[0] = params_ptr->sys.analog.shunt.cs_settle_raio * d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 1U)] + (1.0f - params_ptr->sys.analog.shunt.cs_settle_raio) * d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 2U)];

#if defined(PC_TEST)
    float d_xyz_cmd[3] = { d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 0U)], d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 1U)], d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 2U)] };
    vars_ptr->test[77] = d_xyz_cmd[0];
    vars_ptr->test[78] = d_xyz_cmd[1];
    vars_ptr->test[79] = d_xyz_cmd[2];
    vars_ptr->test[80] = d_xyz_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.uvw_idx, 0U)];
    vars_ptr->test[81] = d_xyz_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.uvw_idx, 1U)];
    vars_ptr->test[82] = d_xyz_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.uvw_idx, 2U)];
    vars_ptr->test[83] = vars_ptr->d_samp[0];
    vars_ptr->test[84] = vars_ptr->d_samp[1];
    vars_ptr->test[85] = (vars_ptr->d_samp[0] - 0.5f) * vars_ptr->v_dc;
    vars_ptr->test[86] = (vars_ptr->d_samp[1] - 0.5f) * vars_ptr->v_dc;
#endif
}
RAMFUNC_END

/**
 * @brief Compute phase-shift single-shunt ADC sample timing (ISR0)
 *
 * Determines two ADC trigger instants (d_samp[0] and d_samp[1]) for single-shunt
 * current reconstruction in phase-shift SVM mode. Each sample point is placed at
 * the larger of half the active-vector duty or the minimum ADC window (adc_d_min),
 * offset by the sample-and-hold delay (adc_d_sh_delay). Used when
 * single_shunt.type == Phase_Shift.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
void CurrReconstPhaseShiftRunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    float* d_uvw_cmd = STRUCT_TO_ARRAY(vars_ptr->d_uvw_cmd);
    
    if ((ctrl_ptr->volt_mod.svm.duty_first *0.5f) <= params_ptr->sys.analog.shunt.single_shunt.adc_d_min)
    {   //0.5ta <= TMin, set second sample point to TMin
       vars_ptr->d_samp[1] = (d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 1U)]) + (params_ptr->sys.analog.shunt.single_shunt.adc_d_min) + params_ptr->sys.analog.shunt.single_shunt.adc_d_sh_delay;
    }
    else
    {   //0.5ta > TMin, set second point to 0.5ta
       vars_ptr->d_samp[1] = (d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 1U)]) + (ctrl_ptr->volt_mod.svm.duty_first *0.5f) + params_ptr->sys.analog.shunt.single_shunt.adc_d_sh_delay;
    }
    if ((ctrl_ptr->volt_mod.svm.duty_second *0.5f) <= params_ptr->sys.analog.shunt.single_shunt.adc_d_min)
    {   //0.5ta <= TMin, set second sample point to TMin
       vars_ptr->d_samp[0] = (d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 2U)]) + (params_ptr->sys.analog.shunt.single_shunt.adc_d_min) + params_ptr->sys.analog.shunt.single_shunt.adc_d_sh_delay;
    }
    else
    {   //0.5ta > TMin, set second point to 0.5ta
       vars_ptr->d_samp[0] = (d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 2U)]) + (ctrl_ptr->volt_mod.svm.duty_second *0.5f) + params_ptr->sys.analog.shunt.single_shunt.adc_d_sh_delay;
    }
}
RAMFUNC_END

/**
 * @brief Run voltage modulator at fast ISR rate (ISR0)
 *
 * Top-level modulation pipeline executed at the ISR0 rate:
 * 1. Computes v_dc_inv and stator voltage magnitude v_s_cmd
 * 2. Runs hybrid modulation (single-shunt sector tracking) via function pointer
 * 3. Runs neutral-point or space-vector modulation via function pointer
 * 4. Runs single-shunt current-reconstruction timing calc via function pointer
 * Modulation and current-reconstruction methods are selected at init time;
 * three-shunt disables hybrid modulation and current-reconstruction.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
void VOLT_MOD_RunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    ctrl_ptr->volt_mod.v_dc_inv = 1.0f / vars_ptr->v_dc;
    vars_ptr->v_s_cmd.rad = sqrtf(POW_TWO(vars_ptr->v_ab_cmd_tot.alpha) + POW_TWO(vars_ptr->v_ab_cmd_tot.beta));

    HybridModRunISR0Wrap[motor_ptr->motor_instance](motor_ptr);

    NeutPointOrSpaceVectModISR0Wrap[motor_ptr->motor_instance](motor_ptr);

    CurrReconstRunISR0Wrap[motor_ptr->motor_instance](motor_ptr);
}
RAMFUNC_END

/** @brief Initialise voltage modulation: select NPM or SVM based on params, enable/disable
 *         hybrid modulation for single-shunt sensing, and clear the modulation index filter.
 *  @param motor_ptr  Pointer to the motor instance */
void VOLT_MOD_Init(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    // Modulation method:
    switch (params_ptr->ctrl.volt.mod_method)
    {
    default:
    case Neutral_Point_Modulation:
        NeutPointOrSpaceVectModISR0Wrap[motor_ptr->motor_instance] = NPMRunISR0;
        break;
    case Space_Vector_Modulation:
        NeutPointOrSpaceVectModISR0Wrap[motor_ptr->motor_instance] = SVMRunISR0;
        break;
    }

    // Shunt configuration, SVM duty shift and current reconstruction:
    if ((params_ptr->sys.analog.shunt.type == Single_Shunt) && (params_ptr->sys.analog.shunt.single_shunt.type == Hyb_Mod))
    {
        VOLT_MOD_EnDisHybMod(motor_ptr, En);
    }
    else if ((params_ptr->sys.analog.shunt.type == Single_Shunt) && (params_ptr->sys.analog.shunt.single_shunt.type == Phase_Shift))
    {
        ctrl_ptr->volt_mod.hm.status = Dis;
        NeutPointOrSpaceVectModISR0Wrap[motor_ptr->motor_instance] = SVMPhaseShiftRunISR0;
        CurrReconstRunISR0Wrap[motor_ptr->motor_instance] = CurrReconstPhaseShiftRunISR0;
        HybridModRunISR0Wrap[motor_ptr->motor_instance] = EmptyFcn;
    }
    else // leg shunt
    {
        VOLT_MOD_EnDisHybMod(motor_ptr, Dis);
    }

    ctrl_ptr->volt_mod.mi_filt = 0.0f;
    ctrl_ptr->volt_mod.svm.duty_first_shift = 0.0f;
    ctrl_ptr->volt_mod.svm.duty_second_shift = 0.0f;
}

/** @brief Enable or disable single-shunt hybrid modulation at runtime.
 *         On enable: installs CurrReconstRunISR0 and HybridModRunISR0 function pointers and resets
 *         hybrid-mod integrator state.  On disable: reverts both pointers to EmptyFcn and sets
 *         ADC sample positions to their maximum safe defaults.
 *  @param motor_ptr  Pointer to the motor instance
 *  @param en         En to activate hybrid modulation, Dis to deactivate */
void VOLT_MOD_EnDisHybMod(MOTOR_t *motor_ptr, EN_DIS_t en)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    if ((ctrl_ptr->volt_mod.hm.status == En) && (en == Dis))
    {
        ctrl_ptr->volt_mod.hm.status = Dis;
        CurrReconstRunISR0Wrap[motor_ptr->motor_instance] = EmptyFcn;
        HybridModRunISR0Wrap[motor_ptr->motor_instance] = EmptyFcn;
        vars_ptr->d_samp[0] = 1.0f - FLT_EPSILON;
        vars_ptr->d_samp[1] = 1.0f - FLT_EPSILON;
    }
    else if ((ctrl_ptr->volt_mod.hm.status == Dis) && (en == En))
    {
        ctrl_ptr->volt_mod.hm.status = En;
        CurrReconstRunISR0Wrap[motor_ptr->motor_instance] = CurrReconstRunISR0;
        HybridModRunISR0Wrap[motor_ptr->motor_instance] = HybridModRunISR0;
        ctrl_ptr->volt_mod.hm.th_error = 0.0f;
        ctrl_ptr->volt_mod.hm.th_mod = 0.0f;
    }
}
