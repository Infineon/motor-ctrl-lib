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
 * @file NotchFilt.h
 * @brief Notch filter implementations
 *
 * Implements two types of notch filters:
 * Type 1: Integrate-and-dump filter (minimal memory, downsampling)
 * Type 2: Sinc filter (full-rate, more memory)
 */

#pragma once

// Two types of notch filters are implemented:
// Type 1: Integrate-and-dump filter
// Type 2: Sinc filter
// Integrate-and-dump filter (type 1) requires minimal memory but it is not full rate and involves downsampling.
// Sinc filter (type 2) is a full-rate filter and doesn't require downsampling. However, it requires more memory.

// TYPE 1 ..........................................................................
/**
 * @brief Integrate-and-dump notch filter structure
 *
 * Minimal memory notch filter with downsampling. Notch frequency = running frequency / notch ratio.
 */
typedef struct
{
    uint32_t notch_ratio; /**< Notch frequency divider ratio */
    uint32_t samp_idx;    /**< Sample index counter */
    float coeff;          /**< Scaling coefficient (1/notch_ratio) */
    float integ;          /**< Integrator accumulator */
    float output;         /**< Filter output */
} INTEG_DUMP_FILT_t;

/**
 * @brief Initialize integrate-and-dump filter
 * @param filt Pointer to filter structure
 * @param notch_ratio Notch frequency ratio
 */
void INTEG_DUMP_FILT_Init(INTEG_DUMP_FILT_t *filt, const uint32_t notch_ratio);

/**
 * @brief Reset integrate-and-dump filter state
 * @param filt Pointer to filter structure
 */
void INTEG_DUMP_FILT_Reset(INTEG_DUMP_FILT_t *filt);

/**
 * @brief Run integrate-and-dump filter
 * @param filt Pointer to filter structure
 * @param input Input signal sample
 */
void INTEG_DUMP_FILT_Run(INTEG_DUMP_FILT_t *filt, const float input);

// TYPE 2 ..........................................................................
#define MAX_NOTCH_RATIO 100U /**< Maximum sinc filter sample buffer length (limits notch ratio) */
/**
 * @brief Sinc notch filter structure
 *
 * Full-rate notch filter requiring more memory. Notch frequency = running frequency / notch ratio.
 */
typedef struct
{
    uint32_t notch_ratio;        /**< Notch frequency divider ratio */
    uint32_t samp_idx;           /**< Sample index counter */
    float coeff;                 /**< Scaling coefficient (1/notch_ratio) */
    float integ;                 /**< Integrator accumulator */
    float output;                /**< Filter output */
    float samp[MAX_NOTCH_RATIO]; /**< Circular buffer for samples */
} SINC_FILT_t;

/**
 * @brief Initialize sinc filter
 * @param filt Pointer to filter structure
 * @param notch_ratio Notch frequency ratio
 */
void SINC_FILT_Init(SINC_FILT_t *filt, const uint32_t notch_ratio);

/**
 * @brief Reset sinc filter state
 * @param filt Pointer to filter structure
 */
void SINC_FILT_Reset(SINC_FILT_t *filt);

/**
 * @brief Run sinc filter
 * @param filt Pointer to filter structure
 * @param input Input signal sample
 */
void SINC_FILT_Run(SINC_FILT_t *filt, const float input);
