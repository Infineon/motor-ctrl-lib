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
 * @file ResonantFilt.c
 * @brief Resonant filter implementation
 *
 * Implements second-order resonant filter for vibration suppression and integration.
 */

#include "Controller.h"

/** @brief Reset resonant filter: clear both integrator states to zero.
 *  @param filt  Pointer to RESONANT_t filter structure */
void RESONANT_Reset(RESONANT_t *filt)
{
    filt->integ_ff = 0.0f;
    filt->integ_fb = 0.0f;
}

/** @brief Compute resonant filter gain coefficients from the continuous-time design parameters.
 *         The filter transfer function is H(s) = a1*s / (s^2 + b1*s + b0) (§6.2 of FW Ref Manual).
 *         Derives k = a1*ts, kp = b1/a1, ki = (b0/a1)*ts.
 *         Typical usage: a1 = ω0², b1 = ω0, b0 = ω0² giving ζ=0.5, Q=1, BW=ω0.
 *  @param filt  Pointer to RESONANT_t filter structure
 *  @param a1    Numerator gain coefficient of H(s) [= ω0²]
 *  @param b1    Denominator linear coefficient of H(s) [= ω0, natural angular frequency in rad/s]
 *  @param b0    Denominator constant coefficient of H(s) [= ω0², resonant frequency squared]
 *  @param ts    Sampling period [s] */
void RESONANT_UpdateParams(RESONANT_t *filt, const float a1, const float b1, const float b0, const float ts)
{
    filt->k = a1 * ts;
    filt->kp = b1 / a1;
    filt->ki = b0 / a1 * ts;
}

/** @brief Execute one step of the second-order resonant filter and return the acceleration estimate.
 *         H(s) = a1*s/(s^2+b1*s+b0) acts as a bandpass differentiator; its output is used as
 *         the inertia feedforward term in the speed loop (acceleration estimator, §6.2 of FW Ref Manual).
 *         Updates integ_fb then integ_ff using the discrete-time state-space recurrence.
 *  @param filt   Pointer to RESONANT_t filter structure
 *  @param input  Filter input signal (speed feedback [rad/s-elec])
 *  @return       Estimated angular acceleration [rad/s²-elec] (= integ_ff) */
RAMFUNC_BEGIN
float RESONANT_Run(RESONANT_t *filt, const float input)
{
    filt->integ_fb += filt->ki * filt->integ_ff;
    filt->integ_ff += (input - filt->kp * filt->integ_ff - filt->integ_fb) * filt->k;
    return (filt->integ_ff);
}
RAMFUNC_END