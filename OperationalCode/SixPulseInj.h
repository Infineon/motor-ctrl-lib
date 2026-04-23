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
 * @file SixPulseInj.h
 * @brief Six-pulse injection for initial rotor position detection
 *
 * Implements six-pulse voltage injection method for determining initial rotor position
 * in permanent magnet synchronous motors without position sensor.
 */

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

#pragma once
#include "General.h"

#define SIX_PULSE 6 /**< Total number of voltage pulses applied (one per space-vector sector) */
#define TWO_PULSE 2 /**< Number of highest-response pulses used for rotor angle estimation */

/**
 * @brief Six-pulse injection state machine states
 */
typedef enum
{
    Not_Started = 0U,      /**< Injection not yet started */
    In_Progress_On_Pulse,  /**< Voltage pulse is currently applied */
    In_Progress_Off_Pulse, /**< Observing current decay after pulse off */
    Processing_Results,    /**< Computing rotor angle from collected data */
    Finished_Success,      /**< Rotor angle determined successfully */
    Finished_Ambiguous,    /**< Result ambiguous; angle estimation unreliable */
} SIX_PULSE_INJ_STATE_t;

/**
 * @brief Per-pulse measurement record
 *
 * Stores one scalar metric (e.g. peak current or decay rate) for each of the
 * six injected voltage pulses, paired with the sector label of each pulse.
 */
typedef struct
{
    int8_t label[SIX_PULSE]; /**< Space-vector sector label for each pulse (-3 to +3) */
    float value[SIX_PULSE];  /**< Measured metric value for each pulse */
} SIX_PULSE_INJ_DATA_t;

/**
 * @brief Six-pulse injection controller state structure
 *
 * Holds all state variables for the six-pulse injection algorithm used to
 * detect initial rotor position without movement in PM synchronous motors.
 * Two metrics are computed per pulse: peak current (i_peak) and exponential
 * decay rate (k). The pulse direction with the highest response indicates
 * rotor alignment.
 */
typedef struct
{
    TIMER_t timer_on;  /**< Timer controlling the voltage pulse ON duration */
    TIMER_t timer_off; /**< Timer controlling the decay observation (OFF) duration */

    SIX_PULSE_INJ_STATE_t state; /**< Current state of the injection state machine */
    SIX_PULSE_INJ_DATA_t i_peak; /**< Peak current measured at end of each ON pulse [A] */
    SIX_PULSE_INJ_DATA_t k;      /**< Decay rate k = exp(Ts/tau) estimated by least-squares during OFF phase */
    float k_num, k_den;          /**< Numerator and denominator accumulators for least-squares k estimate */
    float d_pos, d_neg;          /**< PWM duty cycles for positive and negative pulse phases [0..1] */

    int8_t pulse_num;                   /**< Current pulse index (0 to SIX_PULSE-1) */
    int8_t labels_i_peak[TWO_PULSE];    /**< Space-vector labels of the two highest-peak-current pulses */
    int8_t labels_k[TWO_PULSE];         /**< Space-vector labels of the two highest-k (fastest-decay) pulses */
    int8_t labels_unwrapped[TWO_PULSE]; /**< Unwrapped (0-based, circular-corrected) labels used for angle calculation */

    UVW_t i_uvw_sign;      /**< Per-phase sign (+1/-1) to project three-phase current onto pulse axis */
    float i_fb, i_fb_prev; /**< Projected scalar current feedback and its previous value [A] */

    ELEC_t th_r_est; /**< Estimated initial rotor electrical angle [rad] */
} SIX_PULSE_INJ_t;

/**
 * @brief Initialize six-pulse injection controller
 *
 * Configures ON/OFF timers from parameters, computes PWM duty cycles from the
 * nominal pulse voltage and DC bus voltage, then resets the state machine.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SIX_PULSE_INJ_Init(MOTOR_t *motor_ptr);

/**
 * @brief Reset six-pulse injection state machine
 *
 * Sets the injection state back to Not_Started so the sequence can be
 * re-triggered on the next ISR0 call.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SIX_PULSE_INJ_Reset(MOTOR_t *motor_ptr);

/**
 * @brief Execute one ISR0 (fast) cycle of the six-pulse injection
 *
 * Drives the injection state machine:
 * - Not_Started: arms timers, applies first pulse duty cycle
 * - In_Progress_On_Pulse: waits for ON timer; at expiry zeroes duty cycle and arms OFF timer
 * - In_Progress_Off_Pulse: samples peak current on first OFF cycle; accumulates
 *   least-squares numerator/denominator for decay rate k on subsequent cycles;
 *   advances to next pulse or to Processing_Results when all six pulses are done
 *
 * @param motor_ptr Pointer to motor instance
 */
void SIX_PULSE_INJ_RunISR0(MOTOR_t *motor_ptr);

/**
 * @brief Execute one ISR1 (slow) cycle of the six-pulse injection
 *
 * Runs only in the Processing_Results state:
 * - Extracts the two best-candidate labels from both i_peak and k datasets
 * - Cross-verifies them to flag Finished_Success or Finished_Ambiguous
 * - Unwraps labels across the circular sector boundary
 * - Computes the estimated rotor angle:
 *   th_r_est = PI/2 + PI/4 * label[0] + PI/12 * label[1]
 *
 * @param motor_ptr Pointer to motor instance
 */
void SIX_PULSE_INJ_RunISR1(MOTOR_t *motor_ptr);

#endif
