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


#include "Controller.h"

static bool DummyFlashWrite(uint8_t  motor_id, PARAMS_t* ram_data) { return true; }
static bool DummyFlashRead(uint8_t   motor_id, PARAMS_ID_t params_id, PARAMS_t* ram_data) { return false; };

MC_INFO_t mc_info ={
    .parameter_version = PARAMS_VER,
    .firmware_version  = FIRMWARE_VER,
    .build_config_id   = BUILD_CONFIG_ID,
};

PARAMS_t params[MOTOR_CTRL_NO_OF_MOTOR]= { 0 };

HW_FCN_t hw_fcn = {
    .HardwareIfaceInit = EmptyFcn_OneArgument,
    .EnterCriticalSection = EmptyFcn,
    .ExitCriticalSection = EmptyFcn,
    .GateDriverEnterHighZ = EmptyFcn_OneArgument,
    .GateDriverExitHighZ = EmptyFcn_OneArgument,
    .StartPeripherals = EmptyFcn_OneArgument,
    .StopPeripherals = EmptyFcn_OneArgument,
    .FlashRead = DummyFlashRead,
    .FlashWrite = DummyFlashWrite,
    .ArePhaseVoltagesMeasured = AlwaysTrue,
};

LUT_PARAMS_t params_lut;

void PARAMS_UpdateLookupTable(void)
{
    // LUT Parameters:
    params_lut.sin.th_step = PI_OVER_TWO / (TRIG_LUT_WIDTH - 1); // [Ra]
    params_lut.sin.th_step_inv = 1.0f / params_lut.sin.th_step; // [1/Ra]
    for (uint32_t index = 0; index < TRIG_LUT_WIDTH; ++index)
    {
        params_lut.sin.val[index] = sinf(index * params_lut.sin.th_step); // [#]
    }
    params_lut.atan.step = 1.0f / (INV_TRIG_LUT_WIDTH - 1); // [#]
    params_lut.atan.step_inv = 1.0f / params_lut.atan.step; // [#]
    params_lut.asin.step = params_lut.atan.step; // [#]
    params_lut.asin.step_inv = params_lut.atan.step_inv; // [#]
    for (uint32_t index = 0; index < INV_TRIG_LUT_WIDTH; ++index)
    {
        params_lut.atan.val[index] = atanf(index * params_lut.atan.step); // [Ra]
        params_lut.asin.val[index] = asinf(index * params_lut.asin.step); // [Ra]
    }

}


