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
#include "Controller.h"

// Motor Library Interface APIs
// These APIs are intended for use by the application layer to configure
// motor control behavior at runtime.

/**
 * @brief Enable or disable the motor drive
 *
 * When disabled, the state machine will not start the drive.
 * Must be called after STATE_MACHINE_Init.
 *
 * @param motor_ptr Pointer to the motor instance
 * @param en        True to enable the drive, false to disable
 */
static inline void MOTOR_CTRL_SetDriveEnable(MOTOR_t *motor_ptr, bool en)
{
    motor_ptr->vars_ptr->en = en;
}

/**
 * @brief Assign fault bits exclusively to one reaction type
 *
 * Clears the specified fault bits from all reaction masks, then assigns them
 * to the chosen reaction, ensuring each fault maps to exactly one reaction.
 * Setting @p reaction = No_Reaction removes the fault(s) from all active masks.
 *
 * @note Must be called AFTER STATE_MACHINE_Init / FAULT_PROTECT_Init, as
 *       FAULT_PROTECT_Init overwrites the react_mask with defaults on first call.
 *
 * @param motor_ptr Pointer to the motor instance
 * @param reaction  Target reaction type (No_Reaction, High_Z, Short_Motor)
 * @param fault     FAULT_FLAGS_t with the desired fault bit(s) set
 */
static inline void MOTOR_CTRL_SetFaultReactMask(MOTOR_t *motor_ptr, FAULT_REACTION_t reaction, FAULT_FLAGS_t fault)
{
    FAULTS_t *faults_ptr = motor_ptr->faults_ptr;

    if (reaction < Num_Reactions)
    {
        // Clear the fault bits from all reaction masks
        faults_ptr->react_mask[No_Reaction].all &= ~fault.all;
        faults_ptr->react_mask[High_Z].all &= ~fault.all;
        faults_ptr->react_mask[Short_Motor].all &= ~fault.all;

        // Assign exclusively to the requested reaction.
        // When reaction == No_Reaction, the fault bits are tracked in react_mask[No_Reaction]
        // to record explicitly disabled faults, while remaining cleared from all active masks.
        faults_ptr->react_mask[reaction].all |= fault.all;
    }
}
