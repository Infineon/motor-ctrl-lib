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

void TRQ_Init(MOTOR_t *motor_ptr)
{
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;

    PI_UpdateParams(&ctrl_ptr->trq.pi, params_ptr->ctrl.trq.kp, params_ptr->ctrl.trq.ki * params_ptr->sys.samp.ts0, -params_ptr->ctrl.trq.delta_max.elec, params_ptr->ctrl.trq.delta_max.elec);
}

RAMFUNC_BEGIN
void TRQ_RunCtrlISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;


    vars_ptr->T_cmd_mtpv = LUT1DInterp(&params_ptr->motor.mtpv_lut, vars_ptr->la_cmd_final);
    vars_ptr->T_cmd_final = SAT(-vars_ptr->T_cmd_mtpv, vars_ptr->T_cmd_mtpv, vars_ptr->T_cmd_int);
    PI_Run(&ctrl_ptr->trq.pi, vars_ptr->T_cmd_final, vars_ptr->T_est, 0.0f);
    vars_ptr->delta_cmd.elec = ctrl_ptr->trq.pi.output;
}
RAMFUNC_END

void TRQ_CalcTrq(MOTOR_t *motor_ptr, QD_t* i_qd_r, float* trq)
{
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    *trq = 0.75f * params_ptr->motor.P * (params_ptr->motor.lam + (params_ptr->motor.ld - params_ptr->motor.lq) * i_qd_r->d) * i_qd_r->q;
}

#endif

void TRQ_Reset(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
#if defined(CTRL_METHOD_SFO)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
    PI_Reset(&ctrl_ptr->trq.pi);
#endif
    vars_ptr->T_est_filt = 0.0f;
}

RAMFUNC_BEGIN
void TRQ_RunObsISR0(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_TBC)
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
#endif
#if defined(CTRL_METHOD_SFO)
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
#endif
#if defined(CTRL_METHOD_RFO) && !defined(MOTOR_CTRL_DISABLE_ADDON_FEATURES)
	OBS_t* obs_ptr= motor_ptr->obs_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
#endif


#if defined(CTRL_METHOD_SFO)
    if (!sm_ptr->vars.high_freq.used) // handled by high freqeuncy injection module due to required frequency spectrum separation
    {
        ParkInit(vars_ptr->th_s_est.elec, &vars_ptr->park_s);
        ParkTransform(&vars_ptr->i_ab_fb, &vars_ptr->park_s, &vars_ptr->i_qd_s_fb);
    }
    vars_ptr->T_est = 0.75f * params_ptr->motor.P * vars_ptr->la_qd_s_est.d * vars_ptr->i_qd_s_fb.q;
    vars_ptr->T_est_filt += (vars_ptr->T_est - vars_ptr->T_est_filt) * (params_ptr->filt.trq_w0 * params_ptr->sys.samp.ts0);

#elif defined(CTRL_METHOD_RFO) && !defined(MOTOR_CTRL_DISABLE_ADDON_FEATURES)
    float trq_flux = (params_ptr->sys.fb.mode == Sensorless) ? obs_ptr->pll_r.mag : (params_ptr->motor.lam + (params_ptr->motor.ld - params_ptr->motor.lq) * vars_ptr->i_qd_r_fb.d);
    vars_ptr->T_est = 0.75f * params_ptr->motor.P * trq_flux * vars_ptr->i_qd_r_fb.q;
    vars_ptr->T_est_filt += (vars_ptr->T_est - vars_ptr->T_est_filt) * (params_ptr->filt.trq_w0 * params_ptr->sys.samp.ts0);

#elif defined(CTRL_METHOD_TBC)
    vars_ptr->T_est = ONE_OVER_SQRT_TWO * params_ptr->motor.P * params_ptr->motor.lam * vars_ptr->i_s_fb;
    vars_ptr->T_est_filt += (vars_ptr->T_est - vars_ptr->T_est_filt) * (params_ptr->filt.trq_w0 * params_ptr->sys.samp.ts0);

#endif


#if defined(PC_TEST)
    vars_ptr->test[33] = vars_ptr->T_est_filt;
#endif
}
RAMFUNC_END
