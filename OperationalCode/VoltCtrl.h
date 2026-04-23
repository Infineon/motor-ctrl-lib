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
 * @file VoltCtrl.h
 * @brief Open-loop voltage control module
 *
 * Implements open-loop V/Hz (voltage per frequency) control for motor startup
 * and low-speed operation.
 */

#pragma once

/**
 * @brief Reset open-loop voltage controller state
 *
 * Resets the electrical angle and voltage ramp to safe initial values
 * ready for the next open-loop startup sequence.
 *
 * @param motor_ptr Pointer to motor instance
 */
void VOLT_CTRL_Reset(MOTOR_t *motor_ptr);

/**
 * @brief Execute one ISR0 cycle of the open-loop voltage (V/Hz) controller
 *
 * Advances the voltage ramp according to the V/Hz ratio, integrates the
 * angle reference at the commanded electrical frequency, and writes the
 * resulting alpha/beta voltage commands.
 *
 * @param motor_ptr Pointer to motor instance
 */
void VOLT_CTRL_RunISR0(MOTOR_t *motor_ptr);
