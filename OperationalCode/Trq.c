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

#if defined(CTRL_METHOD_SFO)

void TRQ_Init()
{
    PI_UpdateParams(&ctrl.trq.pi, params.ctrl.trq.kp, params.ctrl.trq.ki * params.sys.samp.ts0, -params.ctrl.trq.delta_max.elec, params.ctrl.trq.delta_max.elec);
}

RAMFUNC_BEGIN
void TRQ_RunCtrlISR0()
{
    vars.T_cmd_mtpv = LUT1DInterp(&params.motor.mtpv_lut, vars.la_cmd_final);
    vars.T_cmd_final = SAT(-vars.T_cmd_mtpv, vars.T_cmd_mtpv, vars.T_cmd_int);
    PI_Run(&ctrl.trq.pi, vars.T_cmd_final, vars.T_est, 0.0f);
    vars.delta_cmd.elec = ctrl.trq.pi.output;
}
RAMFUNC_END

void TRQ_CalcTrq(QD_t* i_qd_r, float* trq)
{
    *trq = 0.75f * params.motor.P * (params.motor.lam + (params.motor.ld - params.motor.lq) * i_qd_r->d) * i_qd_r->q;
}

#endif

void TRQ_Reset()
{
#if defined(CTRL_METHOD_SFO)
    PI_Reset(&ctrl.trq.pi);
#endif
    vars.T_est_filt = 0.0f;
}

RAMFUNC_BEGIN
void TRQ_RunObsISR0()
{
#if defined(CTRL_METHOD_SFO)
    if (!sm.vars.high_freq.used) // handled by high freqeuncy injection module due to required frequency spectrum separation
    {
        ParkInit(vars.th_s_est.elec, &vars.park_s);
        ParkTransform(&vars.i_ab_fb, &vars.park_s, &vars.i_qd_s_fb);
    }
    vars.T_est = 0.75f * params.motor.P * vars.la_qd_s_est.d * vars.i_qd_s_fb.q;

#elif defined(CTRL_METHOD_RFO)
    float trq_flux = (params.sys.fb.mode == Sensorless) ? obs.pll_r.mag : (params.motor.lam + (params.motor.ld - params.motor.lq) * vars.i_qd_r_fb.d);
    vars.T_est = 0.75f * params.motor.P * trq_flux * vars.i_qd_r_fb.q;

#elif defined(CTRL_METHOD_TBC)
    vars.T_est = ONE_OVER_SQRT_TWO * params.motor.P * params.motor.lam * vars.i_s_fb;

#endif

    vars.T_est_filt += (vars.T_est - vars.T_est_filt) * (params.filt.trq_w0 * params.sys.samp.ts0);

#if defined(PC_TEST)
    vars.test[33] = vars.T_est_filt;
#endif
}
RAMFUNC_END