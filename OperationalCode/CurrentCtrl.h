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
 * @file CurrentCtrl.h
 * @brief Current controller interface
 *
 * Provides current control for RFO and TBC control methods. RFO uses separate
 * Q and D axis PI controllers in the rotor reference frame, while TBC uses a
 * single PI controller for scalar current control.
 */

#pragma once

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)

#include "General.h"

/**
 * @brief Current controller structure
 *
 * Contains PI controllers and feed-forward terms for current regulation.
 * Structure differs based on control method (RFO or TBC).
 */
typedef struct
{
    bool en_ff; /**< Enable feed-forward flag */
#if defined(CTRL_METHOD_RFO)
    QD_t ff;   /**< Feed-forward voltage (QD frame, RFO) */
    PI_t pi_q; /**< Q-axis current PI controller (RFO) */
    PI_t pi_d; /**< D-axis current PI controller (RFO) */
#elif defined(CTRL_METHOD_TBC)
    float ff; /**< Feed-forward voltage (scalar, TBC) */
    PI_t pi;  /**< Current PI controller (TBC) */
#endif
} CURRENT_CTRL_t;

#if defined(CTRL_METHOD_RFO)
/**
 * @brief Initialize current controller (RFO)
 * @param motor_ptr Pointer to motor structure
 * @param bw_red_coeff Bandwidth reduction coefficient
 */
void CURRENT_CTRL_Init(MOTOR_t *motor_ptr, const float bw_red_coeff);
#elif defined(CTRL_METHOD_TBC)
/**
 * @brief Initialize current controller (TBC)
 * @param motor_ptr Pointer to motor structure
 */
void CURRENT_CTRL_Init(MOTOR_t *motor_ptr);
#endif

/**
 * @brief Reset current controller to initial state
 * @param motor_ptr Pointer to motor structure
 */
void CURRENT_CTRL_Reset(MOTOR_t *motor_ptr);

/**
 * @brief Run current controller (ISR0)
 * @param motor_ptr Pointer to motor structure
 */
void CURRENT_CTRL_RunISR0(MOTOR_t *motor_ptr);

#endif
