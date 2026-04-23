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
 * @file Controller.h
 * @brief Motor controller main structure definitions and interface
 *
 * This file contains the main controller structures that integrate all control subsystems
 * including filters, speed control, current control, flux control, and voltage modulation.
 * It defines the CTRL_t structure which aggregates all control components and the MOTOR_t
 * structure which represents a complete motor instance with all its associated controllers,
 * sensors, and parameters.
 */

#pragma once

struct MOTOR_str;
typedef struct MOTOR_str MOTOR_t;

#include "Params.h"
#include "Biquad.h"
#include "CtrlFilts.h"
#include "CtrlVars.h"
#include "FaultProtect.h"
#include "FcnExeHandler.h"
#include "General.h"
#include "HallSensor.h"
#include "Observer.h"
#include "PLL.h"
#include "ResonantFilt.h"
#include "SensorIface.h"
#include "NotchFilt.h"
#include "SpeedCtrl.h"
#include "StateMachine.h"
#include "VoltCtrl.h"
#include "Trq.h"
#include "VoltMod.h"
#include "Profiler.h"
#include "AdapTrackLoop.h"
#include "IncEncoder.h"
#include "PositionCtrl.h"

/**
 * @brief Main controller structure containing all control subsystems
 *
 * This structure aggregates all control modules including filters, speed control,
 * current control (for RFO/TBC), flux control (for SFO), position control (for RFO),
 * and voltage modulation. The actual components included depend on the control method
 * selected at compile time (RFO, SFO, or TBC).
 */
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
#include "FluxWeaken.h"
#include "HighFreqInj.h"
#include "SixPulseInj.h"
#endif

#if defined(CTRL_METHOD_TBC)
#include "BlockComm.h"
#include "TrapComm.h"
#include "CurrentCtrl.h"
#elif defined(CTRL_METHOD_RFO)
#include "PhaseAdvance.h"
#include "CurrentCtrl.h"
#elif defined(CTRL_METHOD_SFO)
#include "DeltaCtrl.h"
#include "FluxCtrl.h"
#endif

typedef struct
{
    CTRL_FILTS_t filt;  /**< Control filters (speed and acceleration) */
    SPEED_CTRL_t speed; /**< Speed controller */
#if defined(CTRL_METHOD_RFO)
    POSITION_CTRL_t position; /**< Position controller (RFO only) */
    PHASE_ADV_t ph_adv;       /**< Phase advance (RFO only) */
#elif defined(CTRL_METHOD_SFO)
    TRQ_t trq;          /**< Torque controller (SFO only) */
    FLUX_CTRL_t flux;   /**< Flux controller (SFO only) */
    DELTA_CTRL_t delta; /**< Delta (load angle) controller (SFO only) */
#elif defined(CTRL_METHOD_TBC)
    BLOCK_COMM_t block_comm; /**< Block commutation (TBC only) */
    TRAP_COMM_t trap_comm;   /**< Trapezoidal commutation (TBC only) */
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    CURRENT_CTRL_t curr; /**< Current controller (RFO/TBC) */
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    FLUX_WEAKEN_t flux_weaken;     /**< Flux weakening (RFO/SFO) */
    SIX_PULSE_INJ_t six_pulse_inj; /**< Six pulse injection for sensorless startup (RFO/SFO) */
    HIGH_FREQ_INJ_t high_freq_inj; /**< High frequency injection for sensorless operation (RFO/SFO) */
#endif
    VOLT_MOD_t volt_mod; /**< Voltage modulator */
} CTRL_t;

extern CTRL_t ctrl[MOTOR_CTRL_NO_OF_MOTOR];

/**
 * @brief Hardware function interface structure
 *
 * Contains function pointers to hardware-specific implementations for
 * peripheral initialization, gate driver control, flash operations, and
 * critical section management. This abstraction allows the control library
 * to be hardware-agnostic.
 */
typedef struct
{                                                                            // hardware interface function pointers
    void (*HardwareIfaceInit)(uint8_t motor_id);                             /**< Initialize hardware interface */
    void (*EnterCriticalSection)();                                          /**< Enter critical section (disable interrupts) */
    void (*ExitCriticalSection)();                                           /**< Exit critical section (enable interrupts) */
    void (*GateDriverEnterHighZ)(uint8_t motor_id);                          /**< Put gate driver in high-Z state */
    void (*GateDriverExitHighZ)(uint8_t motor_id);                           /**< Exit gate driver high-Z state */
    void (*StartPeripherals)(uint8_t motor_id);                              /**< Start PWMs, ADCs, DMA, ISRs */
    void (*StopPeripherals)(uint8_t motor_id);                               /**< Stop PWMs, ADCs, DMA, ISRs */
    bool (*FlashRead)(uint8_t motor_id, PARAMS_ID_t id, PARAMS_t *ram_data); /**< Read parameters from flash */
    bool (*FlashWrite)(uint8_t motor_id, PARAMS_t *ram_data);                /**< Write parameters to flash */
    bool (*ArePhaseVoltagesMeasured)(uint8_t motor_id);                      /**< Check if phase voltages are measured (changes with high-z state) */
} HW_FCN_t;

/**
 * @brief Complete motor instance structure
 *
 * Represents a complete motor control instance with all associated controllers,
 * sensors, parameters, and state. Each motor instance maintains pointers to its
 * dedicated control structures, allowing multi-motor control.
 */
struct MOTOR_str
{
    const uint8_t motor_instance;           /**< Motor instance ID */
    STATE_MACHINE_t *const sm_ptr;          /**< Pointer to state machine */
    CTRL_VARS_t *const vars_ptr;            /**< Pointer to control variables */
    CTRL_t *const ctrl_ptr;                 /**< Pointer to controller structure */
    PARAMS_t *const params_ptr;             /**< Pointer to parameters */
    OBS_t *const obs_ptr;                   /**< Pointer to observer */
    FAULTS_t *const faults_ptr;             /**< Pointer to fault status */
    PROTECT_t *const protect_ptr;           /**< Pointer to protection limits */
    HALL_SENS_t *const hall_ptr;            /**< Pointer to Hall sensor */
    SENSOR_IFACE_t *const sensor_iface_ptr; /**< Pointer to sensor interface */
    INC_ENCODER_t *const inc_encoder_ptr;   /**< Pointer to incremental encoder */

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    PROFILER_t *const profiler_ptr; /**< Pointer to motor profiler (RFO/SFO) */
#endif
};

#include "MotorLibAPI.h"

extern HW_FCN_t hw_fcn;

extern MOTOR_t motor[MOTOR_CTRL_NO_OF_MOTOR];

/**
 * @brief Reset the internal speed command integrator
 * @param Motor_ptr Pointer to motor structure
 * @param w0 Initial speed setpoint in electrical coordinates
 */
void CTRL_ResetWcmdInt(MOTOR_t *Motor_ptr, const ELEC_t w0);

/**
 * @brief Update internal speed command with rate limiting (ISR1)
 * @param Motor_ptr Pointer to motor structure
 * @param w_target Target speed in electrical/mechanical coordinates
 * @param rate_lim Rate limit factor (applied with sampling time)
 */
void CTRL_UpdateWcmdIntISR1(MOTOR_t *Motor_ptr, const ELEC_MECH_t w_target, float rate_lim);
