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

FAULTS_t faults[MOTOR_CTRL_NO_OF_MOTOR]= { 0 };;
PROTECT_t protect[MOTOR_CTRL_NO_OF_MOTOR]= { 0 };;

void FAULT_PROTECT_Init(MOTOR_t *motor_ptr)
{
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;
    FAULTS_t* faults_ptr = motor_ptr->faults_ptr;
    // Protections:
    // Motor I2T
    protect_ptr->motor.i2t.i_on = params_ptr->motor.i2t.on_level * params_ptr->motor.i_cont;
    protect_ptr->motor.i2t.i_off = params_ptr->motor.i2t.off_level * params_ptr->motor.i_cont;
    protect_ptr->motor.i2t.filt_coeff = params_ptr->sys.samp.ts0 / params_ptr->motor.i2t.therm_tau;

    // Faults:
    // Detection parameters:
    faults_ptr->vars.oc_thresh = params_ptr->sys.faults.oc_thresh * params_ptr->motor.i_cont;
    DebounceFiltInit(&faults_ptr->vars.vdc_ov_timer, params_ptr->sys.faults.vdc_time, params_ptr->sys.samp.ts1);
    DebounceFiltInit(&faults_ptr->vars.vdc_uv_timer, params_ptr->sys.faults.vdc_time, params_ptr->sys.samp.ts1);
    // Fault reaction bitmasks:
    faults_ptr->react_mask[No_Reaction] = (FAULT_FLAGS_t){ 0U };

    faults_ptr->react_mask[High_Z] = (FAULT_FLAGS_t){ 0U };
    faults_ptr->react_mask[High_Z].sw.oc = 0b1;
    faults_ptr->react_mask[High_Z].sw.ot_ps = 0b1;
    faults_ptr->react_mask[High_Z].sw.brk = 0b1;
    faults_ptr->react_mask[High_Z].sw.em_stop = 0b1;
    faults_ptr->react_mask[High_Z].sw.encoder = 0b1;
    faults_ptr->react_mask[High_Z].hw.cs_ocp = 0b111;
    faults_ptr->react_mask[High_Z].hw.cp = 0b1;
    faults_ptr->react_mask[High_Z].hw.dvdd_ocp = 0b1;
    faults_ptr->react_mask[High_Z].hw.dvdd_uv = 0b1;
    faults_ptr->react_mask[High_Z].hw.dvdd_ov = 0b1;
    faults_ptr->react_mask[High_Z].hw.bk_ocp = 0b1;
    faults_ptr->react_mask[High_Z].hw.ots = 0b1;
    //faults_ptr->react_mask[High_Z].hw.otw = 1U;
    faults_ptr->react_mask[High_Z].hw.rlock = 0b1;
    faults_ptr->react_mask[High_Z].hw.wd = 0b1;
    faults_ptr->react_mask[High_Z].hw.otp = 0b1;

    faults_ptr->react_mask[Short_Motor] = (FAULT_FLAGS_t){ 0U };
    faults_ptr->react_mask[Short_Motor].sw.ov_vdc = 0b1;
    faults_ptr->react_mask[Short_Motor].sw.uv_vdc = 0b1;
    faults_ptr->react_mask[Short_Motor].sw.os = 0b1;
    faults_ptr->react_mask[Short_Motor].sw.hall = 0b1;
    faults_ptr->react_mask[Short_Motor].sw.params = 0b1;

    // Unclearable faults bitmask:
    faults_ptr->unclearable_mask = (FAULT_FLAGS_t){ 0U };
    faults_ptr->unclearable_mask.sw.params = 0b1;

}

void FAULT_PROTECT_Reset(MOTOR_t *motor_ptr)
{
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;
    FAULTS_t* faults_ptr = motor_ptr->faults_ptr;

    // Clear all faults
    faults_ptr->flags.all = 0U;
    faults_ptr->flags_latched.all = 0U;
    faults_ptr->reaction = No_Reaction;

    // Unclearable faults
    // Parameter incompatibilities:
    // - Single shunt sensing is not possible when using trapezoidal commutation, use block commutation instead
#if defined(CTRL_METHOD_TBC)
    faults_ptr->flags.sw.params |= (params_ptr->ctrl.tbc.mode == Trapezoidal_Commutation) && (params_ptr->sys.analog.shunt.type == Single_Shunt);
#endif

    // Motor I2T
    protect_ptr->motor.i2t.i_sq_filt = 0.0f;
    protect_ptr->motor.i2t.state = Dis;
    protect_ptr->motor.i2t.i_limit = params_ptr->motor.i_peak;

#if defined(CTRL_METHOD_SFO)
    protect_ptr->motor.T_lmt = params_ptr->motor.T_max;
#endif
}

RAMFUNC_BEGIN
void FAULT_PROTECT_RunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
#if defined(CTRL_METHOD_TBC)
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
#endif
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;

    // Motor I2T
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
	vars_ptr->i_s_fb_sq = POW_TWO(vars_ptr->i_ab_fb_tot.alpha) + POW_TWO(vars_ptr->i_ab_fb_tot.beta);
#elif defined(CTRL_METHOD_TBC)
    if (params_ptr->ctrl.mode == Volt_Mode_Open_Loop)
    {
    	vars_ptr->i_s_fb_sq = POW_TWO(vars_ptr->i_ab_fb_tot.alpha) + POW_TWO(vars_ptr->i_ab_fb_tot.beta);
    }
    else
    {
    	vars_ptr->i_s_fb_sq = POW_TWO(vars_ptr->i_s_fb);
    }
#endif
    protect_ptr->motor.i2t.i_sq_filt += (vars_ptr->i_s_fb_sq - protect_ptr->motor.i2t.i_sq_filt) * protect_ptr->motor.i2t.filt_coeff;

}
RAMFUNC_END

void FAULT_PROTECT_RunISR1(MOTOR_t *motor_ptr)
{
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;
	FAULTS_t* faults_ptr = motor_ptr->faults_ptr;

    // Protections ..............................................................................................
    // Motor I2T
    protect_ptr->motor.i2t.i_filt = sqrtf(protect_ptr->motor.i2t.i_sq_filt);
#if defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_RFO)
    vars_ptr->i_s_fb = protect_ptr->motor.i2t.i_filt;
#endif

    if ((protect_ptr->motor.i2t.i_filt >= protect_ptr->motor.i2t.i_on) && (protect_ptr->motor.i2t.state == Dis))
    {
        protect_ptr->motor.i2t.state = En;
        protect_ptr->motor.i2t.i_limit = params_ptr->motor.i_cont;
    }
    else if ((protect_ptr->motor.i2t.i_filt < protect_ptr->motor.i2t.i_off) && (protect_ptr->motor.i2t.state == En))
    {
        protect_ptr->motor.i2t.state = Dis;
        protect_ptr->motor.i2t.i_limit = params_ptr->motor.i_peak;
    }

    // Fault Detections .........................................................................................
    // OC
    faults_ptr->flags.sw.oc = (protect_ptr->motor.i2t.i_filt >= faults_ptr->vars.oc_thresh);

    // Vdc OV and UV
    if (vars_ptr->v_dc >= params_ptr->sys.faults.vdc_thresh.max)
    {
        DebounceFiltInc(&faults_ptr->vars.vdc_ov_timer);
    }
    else if (vars_ptr->v_dc <= params_ptr->sys.faults.vdc_thresh.min)
    {
        DebounceFiltInc(&faults_ptr->vars.vdc_uv_timer);
    }
    else
    {
        DebounceFiltDec(&faults_ptr->vars.vdc_ov_timer);
        DebounceFiltDec(&faults_ptr->vars.vdc_uv_timer);
    }
    faults_ptr->flags.sw.ov_vdc = DebounceFiltIsSet(&faults_ptr->vars.vdc_ov_timer);
    faults_ptr->flags.sw.uv_vdc = DebounceFiltIsSet(&faults_ptr->vars.vdc_uv_timer);

    // OT
    faults_ptr->flags.sw.ot_ps = (vars_ptr->temp_ps >= params_ptr->sys.faults.temp_ps_thresh);

    // OS
    faults_ptr->flags.sw.os = (vars_ptr->w_final_filt_abs.elec >= params_ptr->sys.faults.w_thresh.elec);
    faults_ptr->flags.sw.brk = vars_ptr->brk;
    faults_ptr->flags.sw.em_stop = vars_ptr->em_stop;

    // Fault Latching ...........................................................................................
    faults_ptr->flags_latched.all |= faults_ptr->flags.all;

    // Fault Reactions ..........................................................................................
    if (faults_ptr->flags_latched.all & faults_ptr->react_mask[Short_Motor].all)
    {
        // Method indicated by params_ptr->sys.faults_ptr->short_method
        faults_ptr->reaction = Short_Motor;
    }
    else if (faults_ptr->flags_latched.all & faults_ptr->react_mask[High_Z].all) // note that short condition is checked first
    {
        faults_ptr->reaction = High_Z;
    }

#if defined(PC_TEST)
    vars_ptr->test[21] = protect_ptr->motor.i2t.i_on;
    vars_ptr->test[22] = protect_ptr->motor.i2t.i_off;
    vars_ptr->test[23] = protect_ptr->motor.i2t.i_filt;
    vars_ptr->test[24] = protect_ptr->motor.i2t.i_limit;
    vars_ptr->test[25] = POW_TWO(protect_ptr->motor.i2t.i_limit);
    vars_ptr->test[26] = vars_ptr->i_s_fb_sq;
#if defined(CTRL_METHOD_SFO)
    vars_ptr->test[27] = protect_ptr->motor.T_lmt;
#endif
#endif

}

void FAULT_PROTECT_ClearFaults(MOTOR_t *motor_ptr)
{
	FAULTS_t* faults_ptr = motor_ptr->faults_ptr;

    faults_ptr->flags.all &= faults_ptr->unclearable_mask.all;
    faults_ptr->flags_latched.all &= faults_ptr->unclearable_mask.all;
    faults_ptr->reaction = No_Reaction;
}

#if defined(CTRL_METHOD_SFO)
RAMFUNC_BEGIN
void FAULT_PROTECT_RunTrqLimitCtrlISR1(MOTOR_t *motor_ptr)	// Sliding-mode current limiter in SFO
{
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

    float T_lmt = protect_ptr->motor.T_lmt + params_ptr->ctrl.trq.curr_lmt_ki * SIGN(POW_TWO(protect_ptr->motor.i2t.i_limit) - vars_ptr->i_s_fb_sq);
    protect_ptr->motor.T_lmt = SAT(0.0f, params_ptr->motor.T_max, T_lmt);
}
RAMFUNC_END
#endif
