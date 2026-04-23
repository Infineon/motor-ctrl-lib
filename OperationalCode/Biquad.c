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
 * @file Biquad.c
 * @brief Biquad filter implementation
 *
 * Implements initialization and execution functions for a biquad (second-order IIR) filter.
 */

#include "Controller.h"

/**
 * @brief Initialize biquad filter from pole/zero representation
 *
 * Converts pole/zero representation to continuous-time coefficients and
 * initializes the filter.
 *
 * @param biquad Pointer to biquad filter instance
 * @param fs Sampling frequency [Hz]
 * @param k DC gain
 * @param wz Array of two zero frequencies [rad/s] (use INFINITY for no zero)
 * @param wp Array of two pole frequencies [rad/s] (use INFINITY for no pole)
 */
void BIQUAD_PoleZeroInit(BIQUAD_t *biquad, const float fs, const float k, const float wz[2], const float wp[2])
{
    biquad->k = k;
    float wz_inv[2], wp_inv[2];
    for (uint8_t index = 0; index < 2; ++index)
    {
        biquad->wz[index] = wz[index];
        biquad->wp[index] = wp[index];
        wz_inv[index] = isinf(wz[index]) ? 0.0f : 1.0f / wz[index];
        wp_inv[index] = isinf(wp[index]) ? 0.0f : 1.0f / wp[index];
    }

    float a[3], b[3];
    a[0] = biquad->k;
    a[1] = biquad->k * (wz_inv[0] + wz_inv[1]);
    a[2] = biquad->k * (wz_inv[0] * wz_inv[1]);
    b[0] = 1.0f;
    b[1] = (wp_inv[0] + wp_inv[1]);
    b[2] = (wp_inv[0] * wp_inv[1]);

    BIQUAD_ContinuousInit(biquad, fs, a, b);
}

/**
 * @brief Initialize biquad filter from continuous-time coefficients
 *
 * Performs bilinear transformation (Tustin's method) to convert continuous-time
 * transfer function to discrete-time.
 *
 * @param biquad Pointer to biquad filter instance
 * @param fs Sampling frequency [Hz]
 * @param a Numerator coefficients [a0, a1, a2] of H(s)
 * @param b Denominator coefficients [b0, b1, b2] of H(s)
 */
void BIQUAD_ContinuousInit(BIQUAD_t *biquad, const float fs, const float a[3], const float b[3])
{
    for (uint8_t index = 0; index < 3; ++index)
    {
        biquad->a[index] = a[index];
        biquad->b[index] = b[index];
    }

    float m[3], n[3];
    m[0] = biquad->a[0] + biquad->a[1] * fs + biquad->a[2] * POW_TWO(fs);
    m[1] = -biquad->a[1] * fs - 2.0f * biquad->a[2] * POW_TWO(fs);
    m[2] = biquad->a[2] * POW_TWO(fs);

    n[0] = biquad->b[0] + biquad->b[1] * fs + biquad->b[2] * POW_TWO(fs);
    n[1] = -biquad->b[1] * fs - 2.0f * biquad->b[2] * POW_TWO(fs);
    n[2] = biquad->b[2] * POW_TWO(fs);

    BIQUAD_DiscreteInit(biquad, fs, m, n);
}

/**
 * @brief Initialize biquad filter from discrete-time coefficients
 *
 * Normalizes coefficients so that n[0] = 1.0f.
 *
 * @param biquad Pointer to biquad filter instance
 * @param fs Sampling frequency [Hz]
 * @param m Numerator coefficients [m0, m1, m2] of H(z)
 * @param n Denominator coefficients [n0, n1, n2] of H(z)
 */
void BIQUAD_DiscreteInit(BIQUAD_t *biquad, const float fs, const float m[3], const float n[3])
{
    biquad->fs = fs;

    for (uint8_t index = 0; index < 3; ++index)
    {
        biquad->m[index] = m[index] / n[0];
        biquad->n[index] = n[index] / n[0];
    }
}

/**
 * @brief Reset biquad filter states to achieve specified DC output
 *
 * Initializes internal state variables so that a constant input produces
 * the specified output value.
 *
 * @param biquad Pointer to biquad filter instance
 * @param output Desired output value (initializes states for DC input)
 */
void BIQUAD_Reset(BIQUAD_t *biquad, const float output)
{
    float D = output / (biquad->m[0] + biquad->m[1] + biquad->m[2]);
    for (uint8_t index = 0; index < 2; ++index)
    {
        biquad->d[index] = D;
    }
}

/**
 * @brief Execute one iteration of the biquad filter
 *
 * Implements the difference equation:
 * y[n] = m0*d[n] + m1*d[n-1] + m2*d[n-2]
 * where d[n] = x[n] - n1*d[n-1] - n2*d[n-2]
 *
 * @param biquad Pointer to biquad filter instance
 * @param input Input value
 * @return Filtered output value
 */
RAMFUNC_BEGIN
float BIQUAD_Run(BIQUAD_t *biquad, float input)
{
    biquad->input = input;
    biquad->d[2] = biquad->d[1];
    biquad->d[1] = biquad->d[0];
    biquad->d[0] = input - biquad->n[1] * biquad->d[1] - biquad->n[2] * biquad->d[2];
    biquad->output = biquad->m[0] * biquad->d[0] + biquad->m[1] * biquad->d[1] + biquad->m[2] * biquad->d[2];
    return (biquad->output);
}
RAMFUNC_END