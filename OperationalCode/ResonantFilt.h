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
 * @file ResonantFilt.h
 * @brief Resonant filter implementation
 *
 * Implements second-order resonant filter with transfer function H(s)=(a1*s)/(s^2+b1*s+b0).
 * Useful for accelerometer integration and vibration suppression.
 */

#pragma once

/**
 * @brief Resonant filter structure
 *
 * Second-order resonant filter implementation.
 * At low frequencies: H(s)~a1/b0*s
 * At high frequencies: H(s)~a1/s
 * At wc: H(s)=a1/b1
 */
typedef struct
{
    float integ_ff; /**< Feedforward integrator state */
    float integ_fb; /**< Feedback integrator state */
    float k;        /**< Gain coefficient k=a1*ts */
    float kp;       /**< Proportional coefficient kp=b1/a1 */
    float ki;       /**< Integral coefficient ki=b0/a1*ts */
} RESONANT_t;

/**
 * @brief Reset resonant filter state
 * @param filt Pointer to filter structure
 */
void RESONANT_Reset(RESONANT_t *filt);

/**
 * @brief Update resonant filter parameters
 * @param filt Pointer to  filter structure
 * @param a1 Numerator coefficient
 * @param b1 First denominator coefficient
 * @param b0 Second denominator coefficient
 * @param ts Sampling time
 */
void RESONANT_UpdateParams(RESONANT_t *filt, const float a1, const float b1, const float b0, const float ts);

/**
 * @brief Run resonant filter
 * @param filt Pointer to filter structure
 * @param input Input signal sample
 * @return Filtered output
 */
float RESONANT_Run(RESONANT_t *filt, const float input);
