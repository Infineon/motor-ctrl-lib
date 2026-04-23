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
 * @file TrapComm.h
 * @brief Trapezoidal commutation control for BLDC motors
 *
 * Implements trapezoidal (six-step) commutation control for brushless DC motors
 * using Hall sensor feedback.
 */

#pragma once

#if defined(CTRL_METHOD_TBC)

#include "General.h"

/**
 * @brief Trapezoidal commutation controller state structure
 *
 * Holds all state variables for six-step trapezoidal commutation,
 * including sector tracking, ramp control, current references, and PI controllers.
 */
typedef struct
{
    ELEC_t th_ramp;     /**< Electrical angle ramp offset for sector interpolation [rad] */
    uint16_t sect;      /**< Current commutation sector (0-5) */
    uint16_t sect_prev; /**< Previous commutation sector (0-5) */
    uint16_t ramp_cnt;  /**< Ramp step counter within the current sector */
    int16_t dir;        /**< Rotation direction (+1 forward, -1 reverse) */

    bool new_sect;          /**< True when a new sector transition is detected */
    bool ramp_done;         /**< True when the within-sector ramp has completed */
    bool enter_high_z_flag; /**< Flag to request entering high-impedance state */
    bool exit_high_z_flag;  /**< Flag to request exiting high-impedance state */

    XYZ_t i_xyz_ref; /**< Current reference in XYZ (two-phase) frame [A] */
    XYZ_t i_xyz_cmd; /**< Current command in XYZ frame [A] */
    XYZ_t i_xyz_fb;  /**< Current feedback in XYZ frame [A] */
    XYZ_t v_xyz_cmd; /**< Voltage command in XYZ frame [V] */
    UVW_t v_uvw_cmd; /**< Voltage command in three-phase UVW frame [V] */

    PI_t pi_one;  /**< PI controller for ramp phase 1 */
    PI_t pi_two;  /**< PI controller for ramp phase 2 */
    PI_t pi_main; /**< Main current PI controller */
} TRAP_COMM_t;

/**
 * @brief Initialize trapezoidal commutation controller
 *
 * Updates PI controller parameters from motor params and detects the initial
 * commutation sector from Hall sensor feedback.
 *
 * @param motor_ptr Pointer to motor instance
 */
void TRAP_COMM_Init(MOTOR_t *motor_ptr);

/**
 * @brief Reset trapezoidal commutation controller states
 *
 * Resets all PI controller integrators to zero.
 *
 * @param motor_ptr Pointer to motor instance
 */
void TRAP_COMM_Reset(MOTOR_t *motor_ptr);

/**
 * @brief Execute one ISR0 cycle of trapezoidal commutation
 *
 * Performs sector detection, current reference generation, PI current control,
 * and computes the three-phase voltage commands for the inverter.
 *
 * @param motor_ptr Pointer to motor instance
 */
void TRAP_COMM_RunISR0(MOTOR_t *motor_ptr);

#endif