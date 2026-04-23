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
 * @file Observer.h
 * @brief Flux observer for sensorless motor control
 *
 * This module implements a flux-based observer for sensorless position and speed estimation.
 * Uses adaptive flux filters and phase-locked loops to track rotor position.
 * Transfer function: H(s)=s^2/(s^3+c1*s^2+c2*s+c3)*gain
 */

#pragma once

#include "PLL.h"
#include "Biquad.h"

/**
 * @brief Shared flux filter coefficients
 *
 * Coefficients shared between alpha and beta flux filters.
 * c1=(k1+k2+k3)*|w0|, c2=(k1*k2+k1*k3+k2*k3)*|w0|^2, c3=k1*k2*k3*|w0|^3
 */
typedef struct
{
    float gain; /**< Filter gain */
    float c1;   /**< First-order coefficient */
    float c2;   /**< Second-order coefficient */
    float c3;   /**< Third-order coefficient */
} FLUX_FILTS_SHARED_t;

/**
 * @brief Single-axis flux filter structure
 *
 * Third-order flux filter for one axis (alpha or beta).
 */
typedef struct
{
    BILINEAR_INTEG_t bilinear_1; /**< First bilinear integrator */
    BILINEAR_INTEG_t bilinear_2; /**< Second bilinear integrator */
    BILINEAR_INTEG_t bilinear_3; /**< Third bilinear integrator */
} FLUX_FILT_t;

/**
 * @brief Reset flux filter state
 * @param motor_ptr Pointer to motor control structure
 * @param flux_filt Pointer to flux filter
 * @param la_lead_0 Initial flux linkage value
 */
void FLUX_FILT_Reset(MOTOR_t *motor_ptr, FLUX_FILT_t *flux_filt, const float la_lead_0);

/**
 * @brief Run flux filter
 * @param motor_ptr Pointer to motor control structure
 * @param flux_filt Pointer to flux filter
 * @param input Input signal (back-EMF)
 * @return Filtered flux linkage estimate
 */
float FLUX_FILT_Run(MOTOR_t *motor_ptr, FLUX_FILT_t *flux_filt, const float input);

/**
 * @brief Observer state structure
 *
 * Contains all state variables and filters for flux-based position and speed observation.
 */
typedef struct
{
    FLUX_FILT_t flux_filt_alpha;           /**< Alpha-axis flux filter */
    FLUX_FILT_t flux_filt_beta;            /**< Beta-axis flux filter */
    FLUX_FILTS_SHARED_t flux_filts_shared; /**< Shared filter coefficients */
    BIQUAD_t biquad_w;                     /**< Speed filter */
    AB_t d_la_ab;                          /**< Back-EMF alpha-beta */
    AB_t la_ab_lead;                       /**< Flux linkage with phase lead */
    AB_t la_ab_lead_adj;                   /**< Adjusted flux linkage */
    AB_t i_ab_lead;                        /**< Current with phase lead */
    PLL_t pll_r;                           /**< PLL for rotor frame */
#if defined(CTRL_METHOD_SFO)
    PLL_t pll_s; /**< PLL for stator frame */
#endif
    float w_filt_elec;      /**< Filtered electrical speed */
    float w_filt_elec_sign; /**< Sign of filtered speed */
    PARK_t phase_comp;      /**< Phase compensation transform */
} OBS_t;

extern OBS_t obs[MOTOR_CTRL_NO_OF_MOTOR];

/**
 * @brief Initialize observer module
 * @param motor_ptr Pointer to motor control structure
 */
void OBS_Init(MOTOR_t *motor_ptr);

/**
 * @brief Reset observer state
 * @param motor_ptr Pointer to motor control structure
 * @param la_ab_lead Initial flux linkage estimate
 * @param w0 Initial speed estimate
 * @param th0 Initial position estimate
 */
void OBS_Reset(MOTOR_t *motor_ptr, AB_t *la_ab_lead, ELEC_t *w0, ELEC_t *th0);

/**
 * @brief Run observer in fast ISR
 * @param motor_ptr Pointer to motor control structure
 */
void OBS_RunISR0(MOTOR_t *motor_ptr);
