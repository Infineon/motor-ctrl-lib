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
 * @file Trq.h
 * @brief Torque control and estimation module
 *
 * Implements torque control (SFO mode) and torque estimation (RFO and SFO modes)
 * for permanent magnet synchronous motors.
 */

#pragma once

#include "General.h"

#if defined(CTRL_METHOD_SFO)

/**
 * @brief Torque PI controller state (SFO only)
 */
typedef struct
{
    PI_t pi; /**< PI controller state for torque regulation */
} TRQ_t;

/**
 * @brief Initialize the torque PI controller (SFO only)
 *
 * Computes and writes PI gains and output limits into the torque controller
 * from the loaded motor parameters.
 *
 * @param motor_ptr Pointer to motor instance
 */
void TRQ_Init(MOTOR_t *motor_ptr);

/**
 * @brief Execute one ISR0 cycle of the torque PI controller (SFO only)
 *
 * Computes the q-axis current reference by running the PI controller on
 * the torque error (torque reference minus estimated torque).
 *
 * @param motor_ptr Pointer to motor instance
 */
void TRQ_RunCtrlISR0(MOTOR_t *motor_ptr);

/**
 * @brief Calculate instantaneous electromagnetic torque
 *
 * Computes torque using the cross-product formula:
 * trq = 3/2 * pole_pairs * (lam + (Ld - Lq) * id) * iq
 *
 * @param motor_ptr Pointer to motor instance
 * @param i_qd_r    Pointer to q/d-axis current vector in rotor frame [A]
 * @param trq       Pointer to output torque value [N.m]
 */
void TRQ_CalcTrq(MOTOR_t *motor_ptr, QD_t *i_qd_r, float *trq);

#endif

/**
 * @brief Reset torque controller state
 *
 * Clears the PI integrator. Called during state machine transitions
 * to ensure a bumpless start (SFO and RFO).
 *
 * @param motor_ptr Pointer to motor instance
 */
void TRQ_Reset(MOTOR_t *motor_ptr);

/**
 * @brief Run torque observer at ISR0 rate (SFO and RFO)
 *
 * Estimates instantaneous electromagnetic torque from the current d/q feedback
 * and stores the result for use by higher-level controllers.
 *
 * @param motor_ptr Pointer to motor instance
 */
void TRQ_RunObsISR0(MOTOR_t *motor_ptr);
