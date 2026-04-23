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
 * @file PhaseAdvance.h
 * @brief Phase advance control for IPM motors
 *
 * Implements maximum torque per ampere (MTPA) control for interior permanent magnet
 * motors using phase advance techniques.
 */

#pragma once

#if defined(CTRL_METHOD_RFO)
#include "General.h"

/**
 * @brief Phase advance control structure
 *
 * Contains motor type classification and inductance ratios for phase advance calculation.
 */
typedef struct
{
    MOTOR_TYPE_t motor_type; /**< Motor type (SPM or IPM) */
    float lam_over_dl;       /**< Flux linkage over inductance difference ratio */
    float dl_over_lam;       /**< Inductance difference over flux linkage ratio */
} PHASE_ADV_t;

/**
 * @brief Initialize phase advance module
 * @param motor_ptr Pointer to motor control structure
 */
void PHASE_ADV_Init(MOTOR_t *motor_ptr);

/**
 * @brief Run phase advance control in slow ISR
 * @param motor_ptr Pointer to motor control structure
 */
void PHASE_ADV_RunISR1(MOTOR_t *motor_ptr);

/**
 * @brief Calculate optimal stator current magnitude
 * @param motor_ptr Pointer to motor control structure
 * @param i_qd qd-axis current reference
 * @param i_s Pointer to store optimal stator current magnitude
 */
void PHASE_ADV_CalcOptIs(MOTOR_t *motor_ptr, QD_t *i_qd, float *i_s);

#endif
