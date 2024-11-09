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


#if defined(CTRL_METHOD_SFO)
#include "Controller.h"

void DELTA_CTRL_Init()
{
    PI_UpdateParams(&ctrl.delta.pi, 0.0f, 0.0f, -params.ctrl.delta.vq_max, params.ctrl.delta.vq_max);
    ctrl.delta.bw_red_coeff = 1.0f;
}

void DELTA_CTRL_Reset()
{
    PI_Reset(&ctrl.trq.pi);
}

RAMFUNC_BEGIN
void DELTA_CTRL_RunISR0()
{
    float w_abs_sat_elec = SAT(params.ctrl.delta.bw_mult_wl.elec, params.ctrl.delta.bw_mult_wh.elec, vars.w_final_filt_abs.elec);
    ctrl.delta.bw_ratio = SlopeIntercept(params.ctrl.delta.bw_mult_slope, params.ctrl.delta.bw_mult_inter, w_abs_sat_elec);
    ctrl.delta.bw = params.ctrl.delta.bw * ctrl.delta.bw_red_coeff * (1.0f + ctrl.delta.bw_ratio * (params.ctrl.delta.bw_mult - 1.0f));

    ctrl.delta.pi.kp = ctrl.delta.bw * params.ctrl.delta.pole_sep * vars.la_cmd_final;
    ctrl.delta.pi.ki = POW_TWO(ctrl.delta.bw) * (params.ctrl.delta.pole_sep - 1.0f) * vars.la_cmd_final * params.sys.samp.ts0;

    PI_Run(&ctrl.delta.pi, vars.delta_cmd.elec, vars.delta_est.elec, 0.0f);
    vars.v_qd_s_cmd.q = ctrl.delta.pi.output;
}
RAMFUNC_END

#endif
