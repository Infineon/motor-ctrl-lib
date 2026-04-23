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
 * @file FaultProtect.h
 * @brief Fault detection and protection system interface
 *
 * Provides comprehensive fault detection and protection including over-current,
 * over-temperature, over/under-voltage, over-speed, phase loss, and hardware faults.
 * Implements motor thermal protection (I2T) and manages fault reactions (high-Z or motor short).
 *
 * Fault flow:
 * HW ----> Fault Detection ----> SW
 *                                  |
 *                                  | Processing
 *                                  V
 * HW <---- Fault Reaction <---- SW
 *
 * Hardware auto-braking is disabled to allow software to select appropriate
 * fault reactions based on fault type.
 */

#pragma once
#include <stdint.h>
#include "General.h"
#include "Controller.h"

// HW ---->		Fault Detection			----> SW
//											  |
//											  | Processing
//											  V
// HW <----		Fault Reaction (brake)	<---- SW
//
// Some fault detections happen in SW, some happen in HW.
// HW auto-braking is disabled because HW cannot have different fault reactions based on the fault type.
// HW only reports the faults and SW decides what reaction is needed (High_Z or Short_Motor).

/**
 * @brief Fault flags union
 *
 * Provides access to individual fault bits or complete fault register.
 * Organized into software-detected faults (32 bits) and hardware-detected faults (32 bits).
 */
typedef union
{
    struct
    {
        union // 32 bits, software faults space
        {
            struct
            {
                uint32_t oc : 1;         /**< Over-current */
                uint32_t ot_ps : 1;      /**< Over-temperature from power stage */
                uint32_t ov_vdc : 1;     /**< Over-voltage on DC bus */
                uint32_t uv_vdc : 1;     /**< Under-voltage on DC bus */
                uint32_t os : 1;         /**< Over-speed */
                uint32_t hall : 1;       /**< Hall sensor fault (invalid input) */
                uint32_t params : 1;     /**< Parameter fault (incompatibilities) */
                uint32_t brk : 1;        /**< Brake switch enabled */
                uint32_t em_stop : 1;    /**< Emergency stop */
                uint32_t encoder : 1;    /**< Encoder fault */
                uint32_t phase_loss : 1; /**< Phase loss detected */
                uint32_t reserved : 21;  /**< Reserved for future use */
            };
            uint32_t reg; /**< Register access to all SW faults */
        } sw;             /**< Software fault bits and register access */
        union             // 32 bits, hardware faults space
        {
            struct
            {
                uint32_t cs_ocp : 3;    /**< Current-sense over-current (phases W,V,U) */
                uint32_t cp : 1;        /**< Charge pump fault */
                uint32_t dvdd_ocp : 1;  /**< Digital VDD over-current */
                uint32_t dvdd_uv : 1;   /**< Digital VDD under-voltage */
                uint32_t dvdd_ov : 1;   /**< Digital VDD over-voltage */
                uint32_t bk_ocp : 1;    /**< Buck over-current */
                uint32_t ots : 1;       /**< Over-temperature shutdown */
                uint32_t otw : 1;       /**< Over-temperature warning */
                uint32_t rlock : 1;     /**< Rotor lock (disabled) */
                uint32_t wd : 1;        /**< Watchdog fault (disabled) */
                uint32_t otp : 1;       /**< One-time-programmable memory fault */
                uint32_t reserved : 19; /**< Reserved for future use */
            };
            uint32_t reg; /**< Register access to all HW faults */
        } hw;             /**< Hardware fault bits and register access */
    };
    uint64_t all; /**< Access to all faults (SW and HW) */
} FAULT_FLAGS_t;

/**
 * @brief Fault reaction types
 *
 * Defines the possible system responses to detected faults.
 */
typedef enum
{
    No_Reaction = 0U, /**< No reaction, continue operation */
    High_Z,           /**< Put gate driver in high-Z state */
    Short_Motor,      /**< Short motor phases for dynamic braking */
    Num_Reactions     /**< Number of reaction types */
} FAULT_REACTION_t;

/**
 * @brief Fault detection variables
 *
 * Internal variables used for fault detection and debouncing.
 */
typedef struct
{
    float oc_thresh;      /**< Over-current threshold */
    TIMER_t vdc_ov_timer; /**< DC bus over-voltage debounce timer */
    TIMER_t vdc_uv_timer; /**< DC bus under-voltage debounce timer */
} FAULT_VARS_t;

/**
 * @brief Phase loss detection structure
 *
 * Tracks phase currents to detect phase loss conditions.
 */
typedef struct
{
    UVW_t i_uvw_avg;    /**< Average phase currents */
    float filter_coeff; /**< Filter coefficient for averaging */
    TIMER_t timer;      /**< Detection timer */
    bool enable;        /**< Enable phase loss detection */
} FAULT_PHASE_LOSS_t;

/**
 * @brief Faults structure
 *
 * Contains all fault status, masks, and reaction information.
 */
typedef struct
{
    FAULT_FLAGS_t flags;                     /**< Current fault flags */
    FAULT_FLAGS_t flags_latched;             /**< Latched fault flags */
    FAULT_FLAGS_t react_mask[Num_Reactions]; /**< Reaction masks for each fault type */
    FAULT_FLAGS_t unclearable_mask;          /**< Mask of faults that cannot be cleared */
    FAULT_REACTION_t reaction;               /**< Current fault reaction */
    FAULT_VARS_t vars;                       /**< Fault detection variables */
    FAULT_PHASE_LOSS_t phase_loss;           /**< Phase loss detection state */
    bool react_mask_configured;              /**< True when react_mask has been set (init default or user override) */
} FAULTS_t;

/**
 * @brief Motor I²T (thermal) protection structure
 *
 * Implements motor thermal protection by tracking current squared over time.
 */
typedef struct
{
    float i_on;       /**< Current level to enable I2T protection */
    float i_off;      /**< Current level to disable I2T protection */
    float filt_coeff; /**< Filter coefficient for thermal time constant */

    float i_sq_filt; /**< Filtered current squared */
    float i_filt;    /**< Filtered current magnitude */
    EN_DIS_t state;  /**< I2T protection state (enabled/disabled) */
    float i_limit;   /**< Current limit */
} I2T_t;

/**
 * @brief Motor protection structure
 *
 * Protection limits specific to the motor.
 */
typedef struct
{
    I2T_t i2t; /**< Motor thermal (I2T) protection */
#if defined(CTRL_METHOD_SFO)
    float T_lmt; /**< Torque limit (SFO) */
#endif
} MOTOR_PROTECT_t;

/**
 * @brief Drive protection structure
 *
 * Protection limits specific to the drive electronics.
 */
typedef struct
{
    float place_holder; /**< Placeholder for future drive protection features */
} DRIVE_PROTECT_t;

/**
 * @brief Complete protection structure
 *
 * Aggregates motor and drive protection systems.
 */
typedef struct
{
    MOTOR_PROTECT_t motor; /**< Motor protection */
    DRIVE_PROTECT_t drive; /**< Drive protection */
} PROTECT_t;

extern FAULTS_t faults[MOTOR_CTRL_NO_OF_MOTOR];
extern PROTECT_t protect[MOTOR_CTRL_NO_OF_MOTOR];

/**
 * @brief Initialize fault and protection system
 * @param motor_ptr Pointer to motor structure
 */
void FAULT_PROTECT_Init(MOTOR_t *motor_ptr);

/**
 * @brief Reset fault and protection system
 * @param motor_ptr Pointer to motor structure
 */
void FAULT_PROTECT_Reset(MOTOR_t *motor_ptr);

/**
 * @brief Run fault protection (ISR0, fast loop)
 * @param motor_ptr Pointer to motor structure
 */
void FAULT_PROTECT_RunISR0(MOTOR_t *motor_ptr);

/**
 * @brief Run fault protection (ISR1, slow loop)
 * @param motor_ptr Pointer to motor structure
 */
void FAULT_PROTECT_RunISR1(MOTOR_t *motor_ptr);

/**
 * @brief Clear clearable faults
 * @param motor_ptr Pointer to motor structure
 */
void FAULT_PROTECT_ClearFaults(MOTOR_t *motor_ptr);

#if defined(CTRL_METHOD_SFO)
/**
 * @brief Run torque limit control (ISR1, SFO only)
 * @param motor_ptr Pointer to motor structure
 */
void FAULT_PROTECT_RunTrqLimitCtrlISR1(MOTOR_t *motor_ptr);
#endif
