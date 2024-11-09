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

void SPEED_CTRL_Init()
{
#if defined(CTRL_METHOD_RFO)
    PI_UpdateParams(&ctrl.speed.pi, params.ctrl.speed.kp, params.ctrl.speed.ki * params.sys.samp.ts0, -params.motor.i_peak, params.motor.i_peak);
#elif defined(CTRL_METHOD_SFO)
    PI_UpdateParams(&ctrl.speed.pi, params.ctrl.speed.kp, params.ctrl.speed.ki * params.sys.samp.ts0, -params.motor.T_max, params.motor.T_max);
#elif defined(CTRL_METHOD_TBC)
    ctrl.speed.pi.output_max = (params.ctrl.curr.bypass == false) ? SQRT_TWO * params.motor.i_peak : SQRT_TWO * params.ctrl.curr.v_max / params.motor.r;
    ctrl.speed.pi.output_min = -ctrl.speed.pi.output_max;
    PI_UpdateParams(&ctrl.speed.pi, params.ctrl.speed.kp, params.ctrl.speed.ki * params.sys.samp.ts0, ctrl.speed.pi.output_min, ctrl.speed.pi.output_max);
#endif
}

void SPEED_CTRL_Reset()
{
    PI_Reset(&ctrl.speed.pi);
}

void SPEED_CTRL_CalcFeedForwards()
{
    ctrl.speed.ff_inertia = params.ctrl.speed.ff_k_inertia * vars.acc_cmd_int.elec;
    ctrl.speed.ff_friction = params.ctrl.speed.ff_k_friction * SIGN(vars.w_cmd_int.elec);
    ctrl.speed.ff_viscous = params.ctrl.speed.ff_k_viscous * vars.w_cmd_int.elec;
    ctrl.speed.ff_total = ctrl.speed.ff_inertia + ctrl.speed.ff_friction + ctrl.speed.ff_viscous;
}

void SPEED_CTRL_IntegBackCalc(const float cmd)
{
    SPEED_CTRL_CalcFeedForwards();
    PI_IntegBackCalc(&ctrl.speed.pi, cmd, vars.w_cmd_int.elec - vars.w_final_filt.elec, ctrl.speed.ff_total);

}

RAMFUNC_BEGIN
void SPEED_CTRL_RunISR0()
{
    // Feed forwards
    SPEED_CTRL_CalcFeedForwards();
    // PI
    PI_Run(&ctrl.speed.pi, vars.w_cmd_int.elec, vars.w_final_filt.elec, ctrl.speed.ff_total);
    // Output
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    vars.i_cmd_spd = ctrl.speed.pi.output;
#elif defined(CTRL_METHOD_SFO)
    vars.T_cmd_spd = ctrl.speed.pi.output;
#endif
}
RAMFUNC_END
