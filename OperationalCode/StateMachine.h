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
 * @file StateMachine.h
 * @brief Motor control state machine definitions
 *
 * Defines the motor control state machine including all states (initialization, alignment,
 * profiling, open-loop, closed-loop control modes, and fault states) and state transitions.
 */

#pragma once

#include "General.h"

/**
 * @brief Motor control state descriptor
 *
 * Holds function pointers for all four hooks of a single state:
 * entry/exit called on transitions, RunISR0/RunISR1 called every interrupt period.
 */
typedef struct
{
    void (*Entry)();   /**< State entry hook — called once on transition into this state */
    void (*Exit)();    /**< State exit hook — called once on transition out of this state */
    void (*RunISR0)(); /**< Fast-rate ISR0 handler — executed every fast ISR period, can pre-empt ISR1 */
    void (*RunISR1)(); /**< Slow-rate ISR1 handler — executed every slow ISR period, can be pre-empted by ISR0 */
} STATE_t;

/**
 * @brief State machine state identifiers
 *
 * Enumerates every state in the motor control state machine.
 * @warning Do not change the order of states — state table indexing depends on it.
 */
typedef enum
{
    Init = 0U,  /**< Initialization: parameter loading and offset nulling */
    Brake_Boot, /**< Brake and bootstrap: gate-driver bootstrap charging */
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    Align,          /**< Align: apply rotor pre-positioning voltage pulse */
    Six_Pulse,      /**< Six-pulse injection: rotor angle detection via pulse response */
    High_Freq,      /**< High-frequency injection: HFI-based rotor locking */
    Speed_OL_To_CL, /**< Speed OL-to-CL: ramp-based transition to closed-loop speed */
    Dyno_Lock,      /**< Dyno lock: wait for observer to lock in dyno (back-driven) mode */
    Prof_Finished,  /**< Profiler finished: parameter estimation complete */
    Prof_Rot_Lock,  /**< Profiler rotor lock: hold rotor for impedance measurement */
    Prof_R,         /**< Profiler R: stator resistance estimation */
    Prof_Ld,        /**< Profiler Ld: d-axis inductance estimation */
    Prof_Lq,        /**< Profiler Lq: q-axis inductance estimation */
#endif
#if defined(CTRL_METHOD_RFO)
    Current_OL, /**< Open-loop current control (RFO) */
#endif
    Volt_OL,  /**< Open-loop voltage (V/Hz) control */
    Speed_CL, /**< Closed-loop speed control */
    Fault,    /**< Fault state: gate driver disabled, fault reaction active */
#if defined(CTRL_METHOD_SFO)
    Torque_CL, /**< Closed-loop torque control (SFO) */
#elif defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    Current_CL, /**< Closed-loop current control (RFO / TBC) */
#endif
#if defined(CTRL_METHOD_RFO)
    Position_CL, /**< Closed-loop position control (RFO encoder) */
    Catch_Spin,  /**< Catch-spin: synchronize to a free-running motor */
#endif
    State_ID_Max /**< Sentinel: total number of states */
} STATE_ID_t;

/**
 * @brief State variables for the Init state
 */
typedef struct
{
    bool param_init_done;  /**< True when parameter initialization is complete */
    bool offset_null_done; /**< True when current-sensor offset nulling is complete */
    TIMER_t timer;         /**< Init state duration timer */
} STATE_VARS_INIT_t;

/**
 * @brief State variables for the Brake_Boot state
 */
typedef struct
{
    TIMER_t timer;             /**< Bootstrap charge duration timer */
    uint32_t scheudle_counter; /**< Scheduler counter for enabling/disabling phases sequentially */
} STATE_VARS_BRBT_t;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

/**
 * @brief State variables for the Align state
 */
typedef struct
{
    TIMER_t timer; /**< Alignment voltage hold duration timer */
} STATE_VARS_ALIGN;

/**
 * @brief State variables for the High_Freq injection state
 */
typedef struct
{
    TIMER_t timer; /**< HFI lock duration timer */
    bool used;     /**< True if high-frequency injection has been activated */
} STATE_VARS_HIGHFREQ_t;

/**
 * @brief State variables for the Speed_OL_To_CL transition state
 */
typedef struct
{
    TIMER_t timer; /**< Open-loop-to-closed-loop transition hold timer */
} STATE_VARS_SPDOLTOCL_t;

/**
 * @brief State variables for the Dyno_Lock state
 */
typedef struct
{
    TIMER_t timer; /**< Observer lock wait timer */
} STATE_VARS_DYNOLOCK_t;

#endif
#if defined(CTRL_METHOD_RFO)
/**
 * @brief State variables for the Catch_Spin state
 */
typedef struct
{
    TIMER_t timer;         /**< Catch-spin attempt duration timer */
    bool bemf_measurement; /**< True when BEMF measurement phase is active */
    bool done;             /**< True when catch-spin sequence is complete */
} STATE_VARS_CATCHSPIN_t;
#endif
/**
 * @brief State variables for the Fault state
 */
typedef struct
{
    uint32_t clr_try_cnt;  /**< Number of fault-clear retry attempts made */
    TIMER_t clr_try_timer; /**< Timer between fault-clear retry attempts */
    bool clr_faults_prev;  /**< Previous value of clear-faults command (edge detection) */
    bool clr_success;      /**< True when fault was cleared successfully */
    bool clr_request;      /**< True when a fault-clear request is pending */
} STATE_VARS_FAULT_t;

/**
 * @brief Additional ISR callbacks for state machine flexibility
 *
 * Optional user-registered callbacks executed every ISR period in addition
 * to the active state's RunISR0 / RunISR1 handlers.
 */
typedef struct
{
    void (*RunISR0)(); /**< Additional ISR0 callback (may be NULL) */
    void (*RunISR1)(); /**< Additional ISR1 callback (may be NULL) */
} STATE_ADD_CALLBACK;

/**
 * @brief Aggregated state-specific variables for all states
 */
typedef struct
{
    bool speed_reset_required;    /**< True when speed controller needs reset on next state entry */
    STATE_VARS_INIT_t init;       /**< Variables for the Init state */
    STATE_VARS_BRBT_t brake_boot; /**< Variables for the Brake_Boot state */
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    STATE_VARS_ALIGN align;                /**< Variables for the Align state */
    STATE_VARS_HIGHFREQ_t high_freq;       /**< Variables for the High_Freq injection state */
    STATE_VARS_SPDOLTOCL_t speed_ol_to_cl; /**< Variables for the Speed_OL_To_CL transition */
    STATE_VARS_DYNOLOCK_t dyno_lock;       /**< Variables for the Dyno_Lock state */
#endif
#if defined(CTRL_METHOD_RFO)
    STATE_VARS_CATCHSPIN_t catch_spin; /**< Variables for the Catch_Spin state */

#endif
    STATE_VARS_FAULT_t fault; /**< Variables for the Fault state */
#if defined(PC_TEST)
    float *capture_channels[32]; /**< Pointers to variables captured at state transitions (PC test) */
    float capture_vals[32];      /**< Values of captured variables at state transitions (PC test) */
#endif
} STATE_VARS_t;

/**
 * @brief Motor control state machine instance
 *
 * Holds all state descriptors, current/next state IDs, per-state variables,
 * and optional additional ISR callbacks for one motor instance.
 */
typedef struct
{
    STATE_t states[State_ID_Max];    /**< Array of all state descriptors indexed by STATE_ID_t */
    STATE_ID_t current;              /**< Currently active state ID */
    STATE_ID_t next;                 /**< Requested next state (triggers transition when != current) */
    STATE_VARS_t vars;               /**< State-specific variable storage */
    STATE_ADD_CALLBACK add_callback; /**< Optional additional ISR callbacks */
} STATE_MACHINE_t;

extern STATE_MACHINE_t sm[MOTOR_CTRL_NO_OF_MOTOR];

/**
 * @brief Initialize all state machine instances
 *
 * Registers entry/exit/ISR function pointers for every state (Init, Brake_Boot,
 * Align, Speed_CL, Fault, etc.) and sets the initial state to Init for all
 * configured motor instances.
 */
void STATE_MACHINE_Init();

#if defined(PC_TEST)
/**
 * @brief Execute one ISR0 (fast) cycle for all motors (PC test build)
 *
 * Iterates over all motor instances and calls the active state's RunISR0
 * function pointer. State transitions are evaluated and entry/exit hooks
 * are invoked if the state changes.
 */
void STATE_MACHINE_RunISR0();

/**
 * @brief Execute one ISR1 (slow) cycle for all motors (PC test build)
 *
 * Iterates over all motor instances and calls the active state's RunISR1
 * function pointer.
 */
void STATE_MACHINE_RunISR1();
#else
/**
 * @brief Execute one ISR0 (fast) cycle for a single motor
 *
 * Calls the active state's RunISR0 function pointer for the given motor.
 * If a state transition is requested (current != next), executes the Exit
 * hook of the current state and the Entry hook of the next state.
 *
 * @param motor_ptr Pointer to motor instance
 */
void STATE_MACHINE_RunISR0(MOTOR_t *motor_ptr);

/**
 * @brief Execute one ISR1 (slow) cycle for a single motor
 *
 * Calls the active state's RunISR1 function pointer for the given motor.
 *
 * @param motor_ptr Pointer to motor instance
 */
void STATE_MACHINE_RunISR1(MOTOR_t *motor_ptr);
#endif

/**
 * @brief Reset all control modules for a motor
 *
 * Calls the Reset function of every active module (current controller,
 * speed controller, observer, sensor interface, etc.) to bring all
 * integrators and state variables to a known initial condition.
 *
 * @param motor_ptr Pointer to motor instance
 */
void STATE_MACHINE_ResetAllModules(MOTOR_t *motor_ptr);

/**
 * @brief Reset state-machine-level control variables
 *
 * Clears speed reset flags and other inter-state bookkeeping variables
 * without resetting individual module states.
 *
 * @param motor_ptr Pointer to motor instance
 */
void STATE_MACHINE_ResetVariable(MOTOR_t *motor_ptr);
/**
 * @brief Enable or disable runtime fault detection
 *
 * Controls whether soft-coded fault checks (over-current, over-voltage, etc.)
 * are evaluated during ISR execution. Used to temporarily suppress faults
 * during state transitions or profiling sequences.
 *
 * @param motor_ptr Pointer to motor instance
 * @param en        En to enable fault detection, Dis to disable
 */
void SM_RuntimeFaultEnDis(MOTOR_t *motor_ptr, EN_DIS_t en);
