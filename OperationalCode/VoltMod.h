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
 * @file VoltMod.h
 * @brief Voltage modulation (PWM) strategies
 *
 * This module implements various voltage modulation strategies including space vector
 * modulation (SVM), neutral point modulation (NPM), and hybrid modulation for different
 * shunt configurations.
 */

#pragma once

/**
 * @brief Space vector modulation structure
 *
 * Contains parameters for space vector modulation including sector identification,
 * vector voltages, duty cycles, and segment configuration.
 */
typedef struct
{
    int32_t sector;          /**< Current sector number [1-6] */
    float v_first;           /**< First active vector voltage [V] */
    float v_second;          /**< Second active vector voltage [V] */
    float duty_first;        /**< Duty cycle for first vector [0.0-1.0] */
    float duty_second;       /**< Duty cycle for second vector [0.0-1.0] */
    float duty_first_shift;  /**< Phase-shifted duty cycle for first active vector [%] */
    float duty_second_shift; /**< Phase-shifted duty cycle for second active vector [%] */
    float duty_zero;         /**< Zero vector duty cycle [0.0-1.0] */
    bool five_segment;       /**< True for 5-segment, false for 7-segment modulation */
} SVM_t;

/**
 * @brief Neutral point modulation structure
 *
 * Contains parameters for neutral point modulation.
 */
typedef struct
{
    float v_neutral; /**< Neutral point voltage [V] */
} NPM_t;

/**
 * @brief Hybrid modulation structure
 *
 * Implements hybrid modulation for single-shunt current sensing, which modifies
 * the modulation strategy to ensure adequate current sampling windows.
 */
typedef struct
{
    EN_DIS_t status;  /**< Enable/disable status */
    float kv;         /**< Voltage gain [V/V] */
    float th_min;     /**< Minimum angle threshold [rad-elec] */
    float th_error;   /**< Angle error [rad-elec] */
    float th_comp;    /**< Compensation angle [rad-elec] */
    float th_tot;     /**< Total angle [rad-elec] */
    float th_shifted; /**< Shifted angle [rad-elec] */
    float th_base;    /**< Base angle [rad-elec] */
    float th_sector;  /**< Sector angle [rad-elec] */
    float th_mod;     /**< Modulation angle [rad-elec] */
    PARK_t park;      /**< Park transformation coefficients */
} HM_t;

/**
 * @brief Voltage modulation controller structure
 *
 * Aggregates all modulation strategies and contains state variables for
 * modulation index tracking and phase indexing.
 */
typedef struct
{
    NPM_t npm;             /**< Neutral point modulation instance */
    SVM_t svm;             /**< Space vector modulation instance */
    HM_t hm;               /**< Hybrid modulation instance */
    float v_dc_inv;        /**< Inverse of DC bus voltage [1/V] */
    float mi;              /**< Modulation index (V_peak/(2*V_dc/3)) */
    float mi_filt;         /**< Filtered modulation index */
    uint32_t xyz_idx;      /**< Indices for UVW to XYZ conversion */
    uint32_t uvw_idx;      /**< Indices for XYZ to UVW conversion */
    uint32_t uvw_idx_prev; /**< Previous UVW indices */
} VOLT_MOD_t;

/**
 * @brief Initialize voltage modulation
 * @param motor_ptr Pointer to motor instance
 */
void VOLT_MOD_Init(MOTOR_t *motor_ptr);

/**
 * @brief Execute voltage modulation in ISR (fast control loop)
 * @param motor_ptr Pointer to motor instance
 */
void VOLT_MOD_RunISR0(MOTOR_t *motor_ptr);

/**
 * @brief Enable or disable hybrid modulation
 * @param motor_ptr Pointer to motor instance
 * @param en Enable (En) or disable (Dis)
 */
void VOLT_MOD_EnDisHybMod(MOTOR_t *motor_ptr, EN_DIS_t en);

/**
 * @brief Execute hybrid modulation in ISR
 * @param motor_ptr Pointer to motor instance
 */
void HybridModRunISR0(MOTOR_t *motor_ptr);
