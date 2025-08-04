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

/*
 * 	                    fcn_exe_handler.req (MotorID: [31-24],CMD: [0-23])
CMD	                    Both Motor	 	Motor 0	       	Motor 1
Auto_Calc_Params	       	0x1	      		NA				NA
Reset_Modules	     		0x2				NA				NA
Flash_Params				0x3				NA				NA
Set_Tuning_Params_Slow		0x4[M0]		0x1000004 		0x2000004
Set_Tuning_Params_Moderate	0x5[M0] 	0x1000005 		0x2000005
Set_Tuning_Params_Fast		0x6[M0]		0x1000006 		0x2000006
 *
 */
#include "Controller.h"

#define GET_MOTOR_ID  ((fcn_exe_handler.req>>24)>>1)/*MSB[8 bit] contains motor id, motor[0] =>1, motor[1] =>2 */

#define GET_MOTOR_IDs  (fcn_exe_handler.req>>24)& 0x3

FCN_EXE_HANDLER_t fcn_exe_handler;

bool ParameterUpdatedAllowed(void)
{
    bool ret_val =1;
    for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
    {
        if (!( (motor[count].sm_ptr->current == Init) ||(motor[count].sm_ptr->current == Fault))) /*Parameter update allowed only when motor is in init or fault state*/
        {
            ret_val =0;
        }
    }
    return(ret_val);
}
inline FCN_EXE_REG_t FCN_EXE_HANDLER_Mask(FCN_EXE_LABEL_t fcn_label)
{
    return (((FCN_EXE_REG_t)(0b1)) << (uint8_t)(fcn_label));
}

static void AutoCalcParamsWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
            hw_fcn.StopPeripherals(count);
        }
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
            PARAMS_InitAutoCalc(motor[count].params_ptr);
        }
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
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

static void ResetModulesWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
         hw_fcn.StopPeripherals(count);
        }
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          hw_fcn.HardwareIfaceInit(count);
        }
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          STATE_MACHINE_ResetAllModules(&motor[count]);
          StopWatchInit(&motor[count].sm_ptr->vars.init.timer, motor[count].params_ptr->sys.analog.offset_null_time, motor[count].params_ptr->sys.samp.ts0);
          motor[count].sm_ptr->vars.init.offset_null_done = false;
        }

    	for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
    	{
          hw_fcn.StartPeripherals(count);
    	}
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Reset_Modules);
}

static void FlashParamsWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          hw_fcn.StopPeripherals(count);
        }
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          hw_fcn.FlashWrite(count,motor[count].params_ptr);
        }
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
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

static void SetTuningParamsSlowWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          hw_fcn.StopPeripherals(count);
        }

        PROFILER_SetTuningParams(motor[GET_MOTOR_ID].params_ptr,Slow);
        PARAMS_InitAutoCalc(motor[GET_MOTOR_ID].params_ptr);
        
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
            hw_fcn.StartPeripherals(count);
        }
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Set_Tuning_Params_Slow);
}

static void SetTuningParamsModerateWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          hw_fcn.StopPeripherals(count);
        }

        PROFILER_SetTuningParams(motor[GET_MOTOR_ID].params_ptr,Moderate);
        PARAMS_InitAutoCalc(motor[GET_MOTOR_ID].params_ptr);

        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          hw_fcn.StartPeripherals(count);
        }
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Set_Tuning_Params_Moderate);
}

static void SetTuningParamsFastWrapper()
{
    if (ParameterUpdatedAllowed()) // only in init state, otherwise ignore
    {
        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          hw_fcn.StopPeripherals(count);
        }

        PROFILER_SetTuningParams(motor[GET_MOTOR_ID].params_ptr,Fast);
        PARAMS_InitAutoCalc(motor[GET_MOTOR_ID].params_ptr);

        for(int count=0; count<MOTOR_CTRL_NO_OF_MOTOR; count++ )
        {
          hw_fcn.StartPeripherals(count);
        }
    }
    fcn_exe_handler.done |= FCN_EXE_HANDLER_Mask(Set_Tuning_Params_Fast);
}

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

void FCN_EXE_HANDLER_Reset()
{
    fcn_exe_handler.ack = 0U;
    fcn_exe_handler.done = 0U;
}

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
            fcn_exe_handler.callback[fcn_index]();    // done is set in callback
            callback_triggered = true;  // only trigger one callback (highest priority) per each ISR
        }
        else if (!req && ack && done)
        {
            fcn_exe_handler.ack &= ~fcn_mask;
            fcn_exe_handler.fault_flag.word_access =0;
        }

    }
}
