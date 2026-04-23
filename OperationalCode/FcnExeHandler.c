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
 * @file FcnExeHandler.c
 * @brief Function execution handler implementation
 *
 * Implements request/acknowledge mechanism for executing special functions from GUI.
 * Functions are executed based on motor state and priority. Supports multi-motor
 * configurations with motor ID encoded in upper byte of request register.
 *
 * Command format: fcn_exe_handler.req = [MotorID(31-24) | CMD(0-23)]
 * Examples:
 * - Auto_Calc_Params: 0x1 (both motors)
 * - Set_Tuning_Params_Slow Motor 0: 0x1000004
 * - Set_Tuning_Params_Slow Motor 1: 0x2000004
 */

#include "Controller.h"

/** Extract motor ID from request register */
#define GET_MOTOR_ID ((fcn_exe_handler.req >> 24) >> 1)

/** Extract motor ID bitmask from request register */
#define GET_MOTOR_IDs (fcn_exe_handler.req >> 24) & 0x3

FCN_EXE_HANDLER_t fcn_exe_handler;

/**
 * @brief Check if parameter updates are allowed
 *
 * Parameter updates are only allowed when all motors are in Init or Fault state.
 *
 * @return true if parameter updates allowed, false otherwise
 */
bool ParameterUpdatedAllowed(void)
{
    bool ret_val = 1;
    for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
    {
        if (!((motor[count].sm_ptr->current == Init) || (motor[count].sm_ptr->current == Fault))) /*Parameter update allowed only when motor is in init or fault state*/
        {
            ret_val = 0;
        }
    }
    return (ret_val);
}

/**
 * @brief Create bitmask for a function label
 * @param fcn_label Function label to mask
 * @return Bitmask with bit set for the specified function label
 */
inline FCN_EXE_REG_t FCN_EXE_HANDLER_Mask(FCN_EXE_LABEL_t fcn_label)
{
    return (((FCN_EXE_REG_t)(0b1)) << (uint8_t)(fcn_label));
}

/**
 * @brief Wrapper for auto-calculate parameters function
 *
 * Executes parameter auto-calculation for all motors if allowed.
 * Sets fault flag if executed when not in Init state.
 */
static void AutoCalcParamsWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StopPeripherals(count);
        }
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            PARAMS_InitAutoCalc(motor[count].params_ptr);
        }
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StartPeripherals(count);
        }
    }
    else
    {
        fcn_exe_handler.fault_flag.auto_cal_fault = true;
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Auto_Calc_Params);
}

/**
 * @brief Wrapper for reset modules function
 *
 * Resets all control modules for all motors if allowed.
 */
static void ResetModulesWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init or fault state, otherwise ignore
    {
        /* For any motor currently in Fault, clear faults and force the state
         * directly to Init. STATE_MACHINE_ResetAllModules does not reset
         * sm_ptr->current, so we do it here. clr_faults is NOT reset by
         * ResetAllModules either, so FAULT_PROTECT_ClearFaults is called
         * synchronously instead of relying on the FaultISR1 path. */
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            STATE_MACHINE_t *sm_ptr = motor[count].sm_ptr;
            if (sm_ptr->current == Fault)
            {
                FAULT_PROTECT_ClearFaults(&motor[count]);
                sm_ptr->vars.fault.clr_request = false;
                sm_ptr->vars.fault.clr_success = false;
                sm_ptr->vars.speed_reset_required = false; // mirrors InitEntry (Entry() skipped since current==next after direct assignment)
                sm_ptr->current = Init;
                sm_ptr->next = Init; // must equal current so state machine stays in Init and runs offset nulling via InitRunISR1
            }
        }
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StopPeripherals(count);
        }
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.HardwareIfaceInit(count);
        }
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            STATE_MACHINE_ResetAllModules(&motor[count]);
            StopWatchInit(&motor[count].sm_ptr->vars.init.timer, motor[count].params_ptr->sys.analog.offset_null_time, motor[count].params_ptr->sys.samp.ts0);
            motor[count].sm_ptr->vars.init.offset_null_done = false;
        }
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StartPeripherals(count);
        }
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Reset_Modules);
}

/**
 * @brief Wrapper for flash write parameters function
 *
 * Writes current parameters to non-volatile flash storage for all motors if allowed.
 * Sets fault flag if executed when not in Init state.
 */
static void FlashParamsWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StopPeripherals(count);
        }
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.FlashWrite(count, motor[count].params_ptr);
        }
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StartPeripherals(count);
        };
    }
    else
    {
        fcn_exe_handler.fault_flag.write_flash_fault = true;
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Flash_Params);
}

/**
 * @brief Wrapper for slow tuning parameters function
 *
 * Applies the Slow bandwidth preset to all relevant controller and observer
 * parameters for the specified motor ID, then recalculates derived parameters.
 */
static void SetTuningParamsSlowWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StopPeripherals(count);
        }

        PROFILER_SetTuningParams(motor[GET_MOTOR_ID].params_ptr, Slow);
        PARAMS_InitAutoCalc(motor[GET_MOTOR_ID].params_ptr);

        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StartPeripherals(count);
        }
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Set_Tuning_Params_Slow);
}

/**
 * @brief Wrapper for moderate tuning parameters function
 *
 * Applies the Moderate bandwidth preset to all relevant controller and observer
 * parameters for the specified motor ID, then recalculates derived parameters.
 */
static void SetTuningParamsModerateWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StopPeripherals(count);
        }

        PROFILER_SetTuningParams(motor[GET_MOTOR_ID].params_ptr, Moderate);
        PARAMS_InitAutoCalc(motor[GET_MOTOR_ID].params_ptr);

        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StartPeripherals(count);
        }
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Set_Tuning_Params_Moderate);
}

/**
 * @brief Wrapper for fast tuning parameters function
 *
 * Applies the Fast bandwidth preset to all relevant controller and observer
 * parameters for the specified motor ID, then recalculates derived parameters.
 */
static void SetTuningParamsFastWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StopPeripherals(count);
        }

        PROFILER_SetTuningParams(motor[GET_MOTOR_ID].params_ptr, Fast);
        PARAMS_InitAutoCalc(motor[GET_MOTOR_ID].params_ptr);

        for (int count = 0; count < MOTOR_CTRL_NO_OF_MOTOR; count++)
        {
            hw_fcn.StartPeripherals(count);
        }
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Set_Tuning_Params_Fast);
}

/** @brief Initialise function execution handler: register all deferred-function callback
 *         wrappers (AutoCalcParams, ResetModules, FlashParams, SetTuningParams Slow/Moderate/Fast)
 *         into the handler's callback table for later dispatch by RunISR1. */
void FCN_EXE_HANDLER_Init()
{
    // Init Function callbacks
    fcn_exe_handler.callback[Auto_Calc_Params] = AutoCalcParamsWrapper;
    fcn_exe_handler.callback[Reset_Modules] = ResetModulesWrapper;
    fcn_exe_handler.callback[Flash_Params] = FlashParamsWrapper;
    fcn_exe_handler.callback[Set_Tuning_Params_Slow] = SetTuningParamsSlowWrapper;
    fcn_exe_handler.callback[Set_Tuning_Params_Moderate] = SetTuningParamsModerateWrapper;
    fcn_exe_handler.callback[Set_Tuning_Params_Fast] = SetTuningParamsFastWrapper;
}

/**
 * @brief Reset function execution handler state
 *
 * Clears the acknowledge and done bit-fields so that any in-flight
 * function requests are treated as not-yet-started on the next ISR1.
 */
void FCN_EXE_HANDLER_Reset()
{
    fcn_exe_handler.ack = 0U;
    fcn_exe_handler.done = 0U;
}

/**
 * @brief Process deferred function execution requests at slow ISR rate (ISR1)
 *
 * Iterates over all registered function slots (highest-priority first) and
 * dispatches at most one callback per ISR1 cycle:
 * - If a slot is requested (!ack) and no callback has been triggered yet,
 *   sets ack, clears done, fires the registered callback, then locks out
 *   further callbacks for this ISR cycle.
 * - Once the callback sets the done flag and the request is withdrawn (!req),
 *   ack is cleared and the fault flag is reset, completing the handshake.
 *
 * This serialised dispatch prevents multiple state-changing callbacks from
 * running in the same ISR1 tick.
 */
void FCN_EXE_HANDLER_RunISR1()
{
    bool callback_triggered = false;

    for (uint8_t fcn_index = 0U; fcn_index < Max_Fcn_Count; ++fcn_index)
    {
        FCN_EXE_REG_t fcn_mask = FCN_EXE_HANDLER_Mask((FCN_EXE_LABEL_t)(fcn_index));
        bool req = fcn_exe_handler.req & fcn_mask;
        bool ack = fcn_exe_handler.ack & fcn_mask;
        bool done = fcn_exe_handler.done & fcn_mask;

        if (req && !ack && !callback_triggered)
        {
            fcn_exe_handler.ack |= fcn_mask;
            fcn_exe_handler.done &= ~fcn_mask;
            fcn_exe_handler.callback[fcn_index](); // done is set in callback
            callback_triggered = true;             // only trigger one callback (highest priority) per each ISR
        }
        else if (!req && ack && done)
        {
            fcn_exe_handler.ack &= ~fcn_mask;
            fcn_exe_handler.fault_flag.word_access = 0;
        }
    }
}
