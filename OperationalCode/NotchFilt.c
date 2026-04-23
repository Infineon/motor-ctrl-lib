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
 * @file NotchFilt.c
 * @brief Notch filter implementations
 *
 * Implements integrate-and-dump and sinc notch filters.
 */

#include "Controller.h"

/**
 * @brief Initialize integrate-and-dump filter
 *
 * Sets up notch ratio and scaling coefficient.
 *
 * @param filt Pointer to filter structure
 * @param notch_ratio Notch frequency ratio (minimum 1)
 */
void INTEG_DUMP_FILT_Init(INTEG_DUMP_FILT_t *filt, const uint32_t notch_ratio)
{
    filt->notch_ratio = MAX(1U, notch_ratio);
    filt->coeff = 1.0f / filt->notch_ratio;
}

/**
 * @brief Initialize sinc filter
 *
 * Sets up notch ratio and scaling coefficient, saturating ratio to MAX_NOTCH_RATIO.
 *
 * @param filt Pointer to filter structure
 * @param notch_ratio Notch frequency ratio (1 to MAX_NOTCH_RATIO)
 */
void SINC_FILT_Init(SINC_FILT_t *filt, const uint32_t notch_ratio)
{
    filt->notch_ratio = SAT(1U, MAX_NOTCH_RATIO, notch_ratio);
    filt->coeff = 1.0f / filt->notch_ratio;
}

/**
 * @brief Reset integrate-and-dump filter state
 *
 * Clears integrator, output, and sample index.
 *
 * @param filt Pointer to filter structure
 */
void INTEG_DUMP_FILT_Reset(INTEG_DUMP_FILT_t *filt)
{
    filt->samp_idx = 0U;
    filt->integ = 0.0f;
    filt->output = 0.0f;
}

/**
 * @brief Reset sinc filter state
 *
 * Clears integrator, output, sample buffer, and sample index.
 *
 * @param filt Pointer to filter structure
 */
void SINC_FILT_Reset(SINC_FILT_t *filt)
{
    filt->samp_idx = 0U;
    filt->integ = 0.0f;
    filt->output = 0.0f;
    for (uint32_t idx = 0U; idx < filt->notch_ratio; ++idx)
    {
        filt->samp[idx] = 0.0f;
    }
}

/**
 * @brief Run integrate-and-dump filter
 *
 * Integrates input over notch_ratio samples, then outputs scaled result.
 *
 * @param filt Pointer to filter structure
 * @param input Input signal sample
 */
void INTEG_DUMP_FILT_Run(INTEG_DUMP_FILT_t *filt, const float input)
{
    if (filt->samp_idx == filt->notch_ratio)
    {
        filt->output = filt->integ * filt->coeff;
        filt->integ = input;
        filt->samp_idx = 1U;
    }
    else
    {
        filt->integ += input;
        ++filt->samp_idx;
    }
}

/**
 * @brief Run sinc filter
 *
 * Implements moving average filter using circular buffer.
 *
 * @param filt Pointer to filter structure
 * @param input Input signal sample
 */
void SINC_FILT_Run(SINC_FILT_t *filt, const float input)
{
    filt->integ += (input - filt->samp[filt->samp_idx]);
    filt->samp[filt->samp_idx] = input;
    filt->output = filt->integ * filt->coeff;
    filt->samp_idx = (filt->samp_idx + 1U) % filt->notch_ratio;
}
