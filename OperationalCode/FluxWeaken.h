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
 * @file FluxWeaken.h
 * @brief Flux weakening interface for RFO and SFO
 *
 * Provides flux weakening functionality to extend the operating speed range
 * above base speed by reducing magnetic field. Uses different strategies for
 * RFO (adjusting D-axis current) and SFO (adjusting flux command).
 */

#pragma once

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

#include "General.h"

/**
 * @brief Flux weakening structure
 *
 * Contains state variables for flux weakening control. Structure differs
 * based on control method (RFO uses integrator for D-current, SFO uses direct
 * flux command limiting).
 */
typedef struct
{
    bool activated;   /**< Flux weakening active flag */
    float v_q_lim_sq; /**< Q-axis voltage limit squared */
    float v_q_lim;    /**< Q-axis voltage limit */
#if defined(CTRL_METHOD_RFO)
    float vq_error;     /**< Q-axis voltage error (RFO) */
    float integ;        /**< Integrator state (RFO) */
    MINMAX_t integ_lim; /**< Integrator limits (RFO) */
    float id_lim;       /**< D-axis current limit (RFO) */
    float id_fw;        /**< D-axis flux weakening current (RFO) */
#elif defined(CTRL_METHOD_SFO)
    float la_cmd_lim; /**< Flux command limit (SFO) */
#endif
} FLUX_WEAKEN_t;

#if defined(CTRL_METHOD_RFO)
/**
 * @brief Reset flux weakening (RFO only)
 * @param motor_ptr Pointer to motor structure
 */
void FLUX_WEAKEN_Reset(MOTOR_t *motor_ptr);
#endif

/**
 * @brief Run flux weakening control (ISR1)
 * @param motor_ptr Pointer to motor structure
 */
void FLUX_WEAKEN_RunISR1(MOTOR_t *motor_ptr);

#endif
