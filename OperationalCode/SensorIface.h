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
 * @file SensorIface.h
 * @brief Sensor interface module for analog and digital sensors
 *
 * Provides interface for current sensors, voltage sensors, temperature sensors,
 * potentiometer, and digital inputs with calibration and filtering.
 */

#pragma once

#include "Params.h"

/**
 * @brief Analog sensor channel state
 *
 * Holds the raw ADC reading, calibrated value, and filtered output for a
 * single analog input channel.
 */
typedef struct
{
    float filt_coeff; /**< IIR filter coefficient (computed from cut-off frequency and sample period) */
    float raw;        /**< Raw ADC-scaled value before calibration [A, V, or °C depending on channel] */
    float calibrated; /**< Calibrated value after gain/offset correction */
    float filt;       /**< Low-pass filtered output */
} ANALOG_SENSOR_t;

/**
 * @brief Digital input signals
 *
 * Packed union providing bit-field access to direction, brake, and fault
 * digital inputs, as well as a single 32-bit register for atomic access.
 */
typedef union
{
    struct
    {
        uint32_t dir : 1;      /**< Direction switch input */
        uint32_t brk : 1;      /**< Brake switch input */
        uint32_t fault : 1;    /**< External fault pin */
        uint32_t padding : 29; /**< Reserved padding */
    };
    uint32_t all; /**< All digital input bits as a single word */
} DIGITAL_INPUT_t;

/**
 * @brief Sensor interface state for one motor instance
 *
 * Aggregates all analog sensor channels (phase currents, phase voltages, DC bus
 * voltage, power-stage temperature, potentiometer) and the digital input register
 * for a single motor. Also holds offset-nulling variables for current sensors.
 */
typedef struct
{
    ANALOG_SENSOR_t i_samp_0;    /**< Raw current ADC sample, channel 0 (shunt-type dependent) */
    ANALOG_SENSOR_t i_samp_1;    /**< Raw current ADC sample, channel 1 */
    ANALOG_SENSOR_t i_samp_2;    /**< Raw current ADC sample, channel 2 */
    float i_xyz[3];              /**< Ordered array of reconstructed phase currents [A] */
    uint32_t *uvw_idx;           /**< Pointer to ADC channel-to-UVW phase index mapping */
    ANALOG_SENSOR_t i_u;         /**< Phase U current sensor */
    ANALOG_SENSOR_t i_v;         /**< Phase V current sensor */
    ANALOG_SENSOR_t i_w;         /**< Phase W current sensor */
    UVW_t i_uvw_offset_null;     /**< Accumulated offset-nulling correction values [A] */
    float offset_null_loop_gain; /**< Integrator gain for offset-nulling loop */
    ANALOG_SENSOR_t v_uz;        /**< Phase U-to-neutral voltage sensor */
    ANALOG_SENSOR_t v_vz;        /**< Phase V-to-neutral voltage sensor */
    ANALOG_SENSOR_t v_wz;        /**< Phase W-to-neutral voltage sensor */
    ANALOG_SENSOR_t v_dc;        /**< DC bus voltage sensor */
    ANALOG_SENSOR_t temp_ps;     /**< Power-stage temperature sensor */
    ANALOG_SENSOR_t pot;         /**< Potentiometer (analog speed/torque command) */
    DIGITAL_INPUT_t digital;     /**< Digital input signals (direction, brake, fault) */
} SENSOR_IFACE_t;

extern SENSOR_IFACE_t sensor_iface[MOTOR_CTRL_NO_OF_MOTOR];

/**
 * @brief Initialize the sensor interface
 *
 * Computes filter coefficients for all analog sensors (currents, voltages,
 * temperature, potentiometer) based on the configured sampling period and
 * cut-off frequencies. Assigns phase-current ADC channel index mapping.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SENSOR_IFACE_Init(MOTOR_t *motor_ptr);

/**
 * @brief Reset the sensor interface
 *
 * Clears all raw, calibrated, and filtered sensor values to zero, and
 * resets offset-nulling accumulators.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SENSOR_IFACE_Reset(MOTOR_t *motor_ptr);
/**
 * @brief Run sensor interface at ISR0 (fast) rate
 *
 * Reads and processes phase currents and reconstructs the third phase for
 * two-shunt configurations. Also processes phase voltages and DC bus voltage.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SENSOR_IFACE_RunISR0(MOTOR_t *motor_ptr);

/**
 * @brief Run sensor interface at ISR1 (slow) rate
 *
 * Processes slowly-changing sensor signals: power-stage temperature and
 * potentiometer command input.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SENSOR_IFACE_RunISR1(MOTOR_t *motor_ptr);

/**
 * @brief Run current-sensor offset nulling at ISR0 rate
 *
 * Called during the Init state to accumulate and correct ADC offset errors
 * on all phase-current channels using a slow integrating loop.
 *
 * @param motor_ptr Pointer to motor instance
 */
void SENSOR_IFACE_OffsetNullISR0(MOTOR_t *motor_ptr);
