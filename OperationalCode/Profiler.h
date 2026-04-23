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
 * @file Profiler.h
 * @brief Motor parameter identification and auto-tuning module
 *
 * Implements automatic identification of motor parameters (resistance, inductances, flux linkage)
 * and mechanical load parameters (inertia, friction, viscous damping) as well as control tuning.
 */

#pragma once

#include "General.h"
#include "Biquad.h"

// typedef struct
//{
//     /*float* param;*/
//     float val_slow;
//     float val_moderate;
//     float val_fast;
// } TUNING_t;

/**
 * @brief Set controller tuning parameters for a given response speed
 *
 * Writes bandwidth-dependent parameters (current/speed loop gains, observer
 * gains) into params_ptr based on the requested response_speed preset.
 *
 * @param params_ptr    Pointer to the parameter structure to update
 * @param response_speed Desired bandwidth preset (Slow / Moderate / Fast)
 */
void PROFILER_SetTuningParams(PARAMS_t *params_ptr, SPEED_ATTRIB_t response_speed);

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

/**
 * @brief Motor profiler estimated output parameters
 *
 * Holds the parameters identified by the motor profiler including electrical,
 * mechanical load, and open-loop V/Hz values.
 */
typedef struct
{
    // Motor Parameters:
    float r;   /**< Stator resistance [ohm] */
    float lq;  /**< Q-axis inductance [H] */
    float ld;  /**< D-axis inductance [H] */
    float lam; /**< PM flux linkage [Wb] */

    // Mechanical (Load) Parameters:
    float inertia;  /**< Moment of inertia [(N.m)/(rad/sec-mech).sec] */
    float viscous;  /**< Viscous damping coefficient [(N.m)/(rad/sec-mech)] */
    float friction; /**< Coulomb friction torque [N.m] */

    // Open-Loop Volt/Hz Parameters:
    float v_min;        /**< Minimum boost voltage for V/Hz startup [Vpk] */
    float v_to_f_ratio; /**< Voltage-to-frequency slope [Vpk/(rad/sec-elec)] */

} PROFILER_OUTPUT_t;

/**
 * @brief Motor profiler internal state
 *
 * Holds all working variables for the motor parameter identification algorithm,
 * including PI controllers for α/β current regulation, HF signal generation,
 * linear regression buffers for mechanical parameter estimation, and shared
 * progress/status tracking.
 */
typedef struct
{
    // Motor Profiler:
    PI_t pi_i_alpha;            /**< α-axis current PI controller for HF injection */
    PI_t pi_i_beta;             /**< β-axis current PI controller for HF injection */
    AB_t v_ab_mag_hf;           /**< HF voltage injection magnitude in αβ frame [Vpk] */
    AB_t v_ab_cmd_hf;           /**< HF voltage command in αβ frame [V] */
    AB_t v_ab_cmd_lf_filt[4];   /**< LF component of voltage command (filter cascade) [V] */
    float v_s_cmd_lf_filt;      /**< Filtered LF stator voltage magnitude [V] */
    AB_t i_ab_fb_hf;            /**< HF current feedback in αβ frame [A] */
    AB_t i_ab_fb_hf_sq;         /**< Squared HF current feedback (power measure) */
    AB_t i_ab_fb_hf_sq_filt[4]; /**< Filtered squared HF current (inductance extraction) */
    float w_h;                  /**< HF injection angular frequency [rad/sec-elec] */
    float th_h;                 /**< HF injection phase angle [rad] */
    PARK_t park_h;              /**< Sin/cos of HF injection angle for Park transform */
    float z_sq;                 /**< Impedance squared estimate (num/den ratio) */
    float num;                  /**< Numerator accumulator for impedance least-squares */
    float den;                  /**< Denominator accumulator for impedance least-squares */
    float lam;                  /**< Instantaneous flux linkage estimate [Wb] */
    float lam_filt[4];          /**< Filtered flux linkage (cascade stages) [Wb] */

    // Mechanical Profiler:
    LIN_REG_t lin_reg_mech;     /**< Linear regression state for mechanical param estimation */
    BILINEAR_INTEG_t trq_integ; /**< Torque-speed integrator for inertia calculation [Nm.sec] */

    // Open-Loop Volt/Hz Profiler:
    LIN_REG_t lin_reg_volt; /**< Linear regression state for V/Hz slope estimation */

    // Shared:
    float progress;                            /**< Profiling progress [0.0 - 1.0] */
    float progress_mult;                       /**< Progress scaling factor for current sub-task */
    bool freq_sep_active;                      /**< True when frequency-separation mode is active */
    TASK_STATUS_t ramp_down_status;            /**< Ramp-down task completion status */
    PROFILER_OUTPUT_t out_saved;               /**< Snapshot of parameters before profiling started */
    PROFILER_OUTPUT_t out_est;                 /**< Estimated parameter results from profiling */
    TIMER_t timer;                             /**< General profiler step timer */
    ELEC_MECH_t w_cmd_final;                   /**< Final speed command target (elec and mech) [rad/sec] */
    ELEC_t w_cmd_list[PROF_SPEED_POINTS + 1U]; /**< List of speed command setpoints for profiling sweep */
    uint8_t list_index;                        /**< Current index into w_cmd_list */

} PROFILER_t;

extern PROFILER_t profiler[MOTOR_CTRL_NO_OF_MOTOR];

/**
 * @brief Initialize the motor profiler
 *
 * Sets up PI controllers, biquad filters, linear regression buffers, and
 * speed command ramp list used during the profiling sequence.
 *
 * @param motor_ptr Pointer to motor instance
 */
void PROFILER_Init(MOTOR_t *motor_ptr);

/**
 * @brief Entry hook called when the profiler state is entered
 *
 * Resets all profiler state, records a snapshot of current parameters, and
 * arms the first speed command in the profiling sequence.
 *
 * @param motor_ptr Pointer to motor instance
 */
void PROFILER_Entry(MOTOR_t *motor_ptr);

/**
 * @brief Exit hook called when the profiler state is left
 *
 * Copies estimated parameters to the live parameter set (or restores saved
 * parameters if profiling was aborted) and recalculates dependent values.
 *
 * @param motor_ptr Pointer to motor instance
 */
void PROFILER_Exit(MOTOR_t *motor_ptr);

/**
 * @brief Execute one ISR0 cycle of the motor profiler
 *
 * Runs the active profiler sub-state (R, Ld, Lq, mechanical, or V/Hz
 * estimation) and advances the internal progress counter.
 *
 * @param motor_ptr Pointer to motor instance
 */
void PROFILER_RunISR0(MOTOR_t *motor_ptr);

#endif
