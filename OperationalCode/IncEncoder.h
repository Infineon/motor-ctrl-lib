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
 * @file IncEncoder.h
 * @brief Incremental encoder position and speed feedback module
 *
 * This module handles incremental encoder signal processing for position and speed
 * feedback in motor control applications. Supports quadrature, count-direction,
 * and clockwise/counter-clockwise signal types with adaptive tracking.
 */

#pragma once

#include "General.h"
#include "AdapTrackLoop.h"

/**
 * @brief Incremental encoder signal union
 *
 * Supports three types of encoder signals:
 * - Quadrature A & B type
 * - Count & direction type
 * - Clockwise & counter-clockwise type
 * Direction convention: positive = a leads b = cw = 1; negative = a lags b = ccw = 0
 */
typedef struct
{
    union // quadratue a & b type
    {
        struct
        {
            uint8_t b : 1; /**< Quadrature B signal */
            uint8_t a : 1; /**< Quadrature A signal */
        };
        uint8_t a_b : 2; /**< A and B combined */
    };
    union // count & direction type
    {
        struct
        {
            uint8_t cnt : 1; /**< Count signal */
            uint8_t dir : 1; /**< Direction signal */
        };
        uint8_t dir_cnt : 2; /**< Direction and count combined */
    };
    union // clockwise & counter-clockwise type
    {
        struct
        {
            uint8_t ccw : 1; /**< Counter-clockwise signal */
            uint8_t cw : 1;  /**< Clockwise signal */
        };
        uint8_t cw_ccw : 2; /**< CW and CCW combined */
    };
} INC_ENC_SIGNAL_t;

/**
 * @brief Incremental encoder position and speed tracking structure
 *
 * Contains all state variables and filters for processing incremental encoder
 * signals and computing position and speed estimates.
 */
typedef struct
{
    float per_cap_freq; /**< [Hz] Period capture frequency from hardware */
    uint16_t per_cap;   /**< [ticks] Captured period from hardware */
    float dir_cap;      /**< [] Captured direction (+1 or -1) */
    int16_t pos_cap;    /**< [ticks] Captured position */

    int16_t pos_prev;  /**< [ticks] Previous position */
    int16_t pos_delta; /**< [ticks] Position change */
    int16_t pos_mod;   /**< [ticks] Position modulo counts per revolution */

    TIMER_t zero_spd_timer;       /**< [] Timer for zero speed detection */
    ADAP_TRACK_LOOP_t track_loop; /**< [] Adaptive tracking loop for fine angle tracking */

    float w_conv_coeff;       /**< [(Ra/sec-elec)/(period-ticks)] Speed conversion coefficient */
    float th_conv_coeff;      /**< [Ra-elec/(position-ticks)] Angle conversion coefficient */
    float th_mech_conv_coeff; /**< [Ra-mech/(position-ticks)] Mechanical angle conversion coefficient */
    ELEC_t w_ffm_thresh;      /**< [Ra/sec-elec] Feed forward calculation method threshold */
} INC_ENCODER_t;

extern INC_ENCODER_t inc_encoder[MOTOR_CTRL_NO_OF_MOTOR];

/**
 * @brief Initialize incremental encoder module
 * @param motor_ptr Pointer to motor control structure
 */
void INC_ENCODER_Init(MOTOR_t *motor_ptr);

/**
 * @brief Reset incremental encoder state
 * @param motor_ptr Pointer to motor control structure
 * @param th0 Initial position estimate
 */
void INC_ENCODER_Reset(MOTOR_t *motor_ptr, ELEC_t th0);

/**
 * @brief Run incremental encoder processing in fast ISR
 * @param motor_ptr Pointer to motor control structure
 */
void INC_ENCODER_RunISR0(MOTOR_t *motor_ptr);
