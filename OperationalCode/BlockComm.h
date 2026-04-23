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
 * @file BlockComm.h
 * @brief Block commutation (six-step) control
 *
 * This module implements block (trapezoidal) commutation control for BLDC motors,
 * using Hall sensor signals to determine commutation timing.
 */

#pragma once

#if defined(CTRL_METHOD_TBC)

#include "General.h"

/**
 * @brief Block commutation controller structure
 *
 * Contains state variables for block commutation control including Hall signals,
 * phase coefficients, duty cycle commands, and high-Z states.
 */
typedef struct
{
    UVW_SIGNAL_t hall_equiv; /**< Equivalent Hall signal (actual or computed) */

    UVW_t i_uvw_coeff; /**< Current coefficients for each phase */

    float d_cmd;   /**< Duty cycle command [0.0-1.0] */
    float v_s_mag; /**< Magnitude of voltage command */
    bool v_s_sign; /**< Sign of voltage command (true=positive) */

    UVW_SIGNAL_t high_z_state;      /**< Current high-Z (high impedance) state for each phase */
    UVW_SIGNAL_t high_z_state_prev; /**< Previous high-Z state for each phase */
    UVW_SIGNAL_t enter_high_z_flag; /**< Flag indicating transition into high-Z state */
    UVW_SIGNAL_t exit_high_z_flag;  /**< Flag indicating transition out of high-Z state */

} BLOCK_COMM_t;

/**
 * @brief Compute equivalent Hall signal from rotor angle
 * @param th_r Rotor angle (must be wrapped to [-π, π))
 * @return Equivalent Hall sensor signal
 */
UVW_SIGNAL_t BLOCK_COMM_EquivHallSignal(ELEC_t th_r);

/**
 * @brief Initialize block commutation controller
 * @param motor_ptr Pointer to motor instance
 */
void BLOCK_COMM_Init(MOTOR_t *motor_ptr);

/**
 * @brief Execute block commutation current sampling (ISR)
 * @param motor_ptr Pointer to motor instance
 */
void BLOCK_COMM_RunCurrSampISR0(MOTOR_t *motor_ptr);

/**
 * @brief Execute block commutation voltage modulation (ISR)
 * @param motor_ptr Pointer to motor instance
 */
void BLOCK_COMM_RunVoltModISR0(MOTOR_t *motor_ptr);

#endif