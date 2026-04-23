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
 * @file CtrlFilts.h
 * @brief Control filters interface
 *
 * Provides filtering functions for speed and acceleration signals in the control loop.
 * Includes anti-resonant filtering for speed feedback and resonant filtering for
 * acceleration estimation from speed command.
 */

#pragma once

#include "Biquad.h"
#include "ResonantFilt.h"

/**
 * @brief Control filters structure
 *
 * Contains filters used in the control loop for signal conditioning and estimation.
 */
typedef struct
{
    BIQUAD_t spd_ar; /**< Speed anti-resonant filter */
    RESONANT_t acc;  /**< Acceleration estimator filter */

} CTRL_FILTS_t;

/**
 * @brief Initialize control filters
 * @param motor_ptr Pointer to motor structure
 */
void CTRL_FILTS_Init(MOTOR_t *motor_ptr);

/**
 * @brief Reset control filters to initial state
 * @param motor_ptr Pointer to motor structure
 */
void CTRL_FILTS_Reset(MOTOR_t *motor_ptr);

/**
 * @brief Run speed filter (ISR1)
 * @param motor_ptr Pointer to motor structure
 */
void CTRL_FILTS_RunSpeedISR1(MOTOR_t *motor_ptr);

/**
 * @brief Run acceleration estimator filter (ISR1)
 * @param motor_ptr Pointer to motor structure
 */
void CTRL_FILTS_RunAccelISR1(MOTOR_t *motor_ptr);

/**
 * @brief Run all control filters (ISR1)
 * @param motor_ptr Pointer to motor structure
 */
void CTRL_FILTS_RunAllISR1(MOTOR_t *motor_ptr);
