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

#pragma once

#include "General.h"

typedef struct
{
    void (*Entry)();	// Entry function
    void (*Exit)();		// Exit function
    void (*RunISR0)();	// ISR0, fast, can pre-empt ISR1
    void (*RunISR1)();	// ISR1, slow, can be pre-empted ISR0
} STATE_t;

// Caution: do not change the order of states!
typedef enum
{
    Init = 0U,		// Initialization
    Brake_Boot,     // Brake and Bootstrap
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    Align,          // Aligning (pre-positioning)
    Six_Pulse,		// Six pulse injection
    High_Freq,		// High-frequency-injection locking
    Speed_OL_To_CL,	// Transition from open-loop to closed-loop
    Dyno_Lock,		// Waiting for obsever lock to start up in dyno mode
    Prof_Finished,  // Profiler, finished
    Prof_Rot_Lock,  // Profiler, rotor locking
    Prof_R,         // Profiler, stator resistance estimation
    Prof_Ld,        // Profiler, d-axis inductance estimation
    Prof_Lq,        // Profiler, q-axis inductance estimation
#endif
#if defined(CTRL_METHOD_RFO)
    Current_OL,     // Open-loop current control
#endif
    Volt_OL,		// Open-loop voltage control
    Speed_CL,		// Closed-loop speed control
    Fault,			// Fault
#if defined(CTRL_METHOD_SFO)
    Torque_CL,		// Closed-loop torque control
#elif defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    Current_CL,		// Closed-loop current control
#endif
#if defined(CTRL_METHOD_RFO)
    Position_CL,     // Closed-loop position control
#endif
    State_ID_Max
} STATE_ID_t;

typedef struct
{
    bool param_init_done;
    bool offset_null_done;
    TIMER_t timer;
} STATE_VARS_INIT_t;

typedef struct
{
    TIMER_t timer;
} STATE_VARS_BRBT_t;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

typedef struct
{
    TIMER_t timer;
} STATE_VARS_ALIGN;

typedef struct
{
    TIMER_t timer;
    bool used;		// true = using high freqeuncy injection
} STATE_VARS_HIGHFREQ_t;

typedef struct
{
    TIMER_t timer;
} STATE_VARS_SPDOLTOCL_t;

typedef struct
{
    TIMER_t timer;
} STATE_VARS_DYNOLOCK_t;

#endif

typedef struct
{
    uint32_t clr_try_cnt;
    bool clr_faults_prev;
    bool clr_success;
    bool clr_request;
} STATE_VARS_FAULT_t;

typedef struct
{
    void (*RunISR0)();
    void (*RunISR1)();
} STATE_ADD_CALLBACK;   // additional ISR callbacks for more flexibility

typedef struct
{
    bool speed_reset_required;
    STATE_VARS_INIT_t init;
    STATE_VARS_BRBT_t brake_boot;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    STATE_VARS_ALIGN align;
    STATE_VARS_HIGHFREQ_t high_freq;
    STATE_VARS_SPDOLTOCL_t speed_ol_to_cl;
    STATE_VARS_DYNOLOCK_t dyno_lock;
#endif
    STATE_VARS_FAULT_t fault;
#if defined(PC_TEST)
    float* capture_channels[32];	// for capturing variable in state transitions
    float capture_vals[32];			// for capturing variable in state transitions
#endif
} STATE_VARS_t;

typedef struct
{
    STATE_t states[State_ID_Max];
    STATE_ID_t current;
    STATE_ID_t next;
    STATE_VARS_t vars;
    STATE_ADD_CALLBACK add_callback;
} STATE_MACHINE_t;

extern STATE_MACHINE_t sm[MOTOR_CTRL_NO_OF_MOTOR];

void STATE_MACHINE_Init();
#if defined(PC_TEST)
void STATE_MACHINE_RunISR0();
void STATE_MACHINE_RunISR1();
#else
void STATE_MACHINE_RunISR0(MOTOR_t* motor_ptr);
void STATE_MACHINE_RunISR1(MOTOR_t* motor_ptr);
#endif
void STATE_MACHINE_ResetAllModules(MOTOR_t* motor_ptr);


void STATE_MACHINE_ResetVariable(MOTOR_t *motor_ptr);
