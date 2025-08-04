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


typedef union
{
    struct
    {
        union	// 32 bits, software faults space
        {
            struct
            {
                uint32_t oc : 1;			// over-current
                uint32_t ot_ps : 1;			// over-temperature from power-stage (possibilities: motor, power-stage, gate-driver, ucontroller)
                uint32_t ov_vdc : 1;		// over-voltage from vdc
                uint32_t uv_vdc : 1;		// under-voltage from vdc
                uint32_t os : 1;			// over-speed
                uint32_t hall : 1;			// hall-sensor fault, invalid input
                uint32_t params : 1;	    // parameter fault (incompatibilities between parameters)
                uint32_t brk : 1;			// brake switch, enabled
                uint32_t em_stop : 1;		// emergency stop
                uint32_t encoder : 1;       // encoder
                uint32_t reserved : 22;		// reserved
            };
            uint32_t reg;
        } sw;
        union	// 32 bits, hardware faults space
        {
            struct
            {
                uint32_t cs_ocp : 3;		// current-sense over-current, phase {w,v,u}, enabled
                uint32_t cp : 1;			// charge pump, enabled
                uint32_t dvdd_ocp : 1;		// digital vdd over-current, enabled
                uint32_t dvdd_uv : 1;		// digital vdd under-voltage, enabled
                uint32_t dvdd_ov : 1;		// digital vdd over-voltage, enabled
                uint32_t bk_ocp : 1;		// buck over-current, enabled
                uint32_t ots : 1;			// over-temperature shutdown, enabled
                uint32_t otw : 1;			// over-temperature warning, enabled
                uint32_t rlock : 1;			// rotor lock, disabled
                uint32_t wd : 1;			// watchdog, disabled
                uint32_t otp : 1;			// one-time-programmable memory, enabled
                uint32_t reserved : 19;		// reserved
            };
            uint32_t reg;
        } hw;
    };
    uint64_t all;
} FAULT_FLAGS_t;

typedef enum
{
    No_Reaction = 0U,
    High_Z,
    Short_Motor,
    Num_Reactions
} FAULT_REACTION_t;

typedef struct
{
    float oc_thresh;
    TIMER_t vdc_ov_timer;
    TIMER_t vdc_uv_timer;
} FAULT_VARS_t;

typedef struct
{
    FAULT_FLAGS_t flags;
    FAULT_FLAGS_t flags_latched;
    FAULT_FLAGS_t react_mask[Num_Reactions];
    FAULT_FLAGS_t unclearable_mask;
    FAULT_REACTION_t reaction;
    FAULT_VARS_t vars;
} FAULTS_t;

typedef struct
{
    float i_on;
    float i_off;
    float filt_coeff;

    float i_sq_filt;
    float i_filt;
    EN_DIS_t state;
    float i_limit;
} I2T_t;

typedef struct
{
    I2T_t i2t;
#if defined(CTRL_METHOD_SFO)
    float T_lmt;
#endif
} MOTOR_PROTECT_t;	// Protecting the motor

typedef struct
{
    float place_holder;
} DRIVE_PROTECT_t;	// Protecting the drive

typedef struct
{
    MOTOR_PROTECT_t motor;
    DRIVE_PROTECT_t drive;
} PROTECT_t;

extern FAULTS_t faults[MOTOR_CTRL_NO_OF_MOTOR];
extern PROTECT_t protect[MOTOR_CTRL_NO_OF_MOTOR];

void FAULT_PROTECT_Init(MOTOR_t *motor_ptr);
void FAULT_PROTECT_Reset(MOTOR_t *motor_ptr);
void FAULT_PROTECT_RunISR0(MOTOR_t *motor_ptr);
void FAULT_PROTECT_RunISR1(MOTOR_t *motor_ptr);
void FAULT_PROTECT_ClearFaults(MOTOR_t *motor_ptr);

#if defined(CTRL_METHOD_SFO)
void FAULT_PROTECT_RunTrqLimitCtrlISR1(MOTOR_t *motor_ptr);
#endif
