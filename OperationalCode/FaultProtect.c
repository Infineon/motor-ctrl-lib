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

FAULTS_t faults;
PROTECT_t protect;

void FAULT_PROTECT_Init()
{
    // Protections:
    // Motor I2T
    protect.motor.i2t.i_on = params.motor.i2t.on_level * params.motor.i_cont;
    protect.motor.i2t.i_off = params.motor.i2t.off_level * params.motor.i_cont;
    protect.motor.i2t.filt_coeff = params.sys.samp.ts0 / params.motor.i2t.therm_tau;

    // Faults:
    // Detection parameters:
    faults.vars.oc_thresh = params.sys.faults.oc_thresh * params.motor.i_cont;
    DebounceFiltInit(&faults.vars.vdc_ov_timer, params.sys.faults.vdc_time, params.sys.samp.ts1);
    DebounceFiltInit(&faults.vars.vdc_uv_timer, params.sys.faults.vdc_time, params.sys.samp.ts1);
    // Fault reaction bitmasks:
    faults.react_mask[No_Reaction] = (FAULT_FLAGS_t){ 0U };

    faults.react_mask[High_Z] = (FAULT_FLAGS_t){ 0U };
    faults.react_mask[High_Z].sw.oc = 0b1;
    faults.react_mask[High_Z].sw.ot_ps = 0b1;
    faults.react_mask[High_Z].sw.brk = 0b1;
    faults.react_mask[High_Z].sw.em_stop = 0b1;
    faults.react_mask[High_Z].sw.encoder = 0b1;
    faults.react_mask[High_Z].hw.cs_ocp = 0b111;
    faults.react_mask[High_Z].hw.cp = 0b1;
    faults.react_mask[High_Z].hw.dvdd_ocp = 0b1;
    faults.react_mask[High_Z].hw.dvdd_uv = 0b1;
    faults.react_mask[High_Z].hw.dvdd_ov = 0b1;
    faults.react_mask[High_Z].hw.bk_ocp = 0b1;
    faults.react_mask[High_Z].hw.ots = 0b1;
    //faults.react_mask[High_Z].hw.otw = 1U;
    faults.react_mask[High_Z].hw.rlock = 0b1;
    faults.react_mask[High_Z].hw.wd = 0b1;
    faults.react_mask[High_Z].hw.otp = 0b1;

    faults.react_mask[Short_Motor] = (FAULT_FLAGS_t){ 0U };
    faults.react_mask[Short_Motor].sw.ov_vdc = 0b1;
    faults.react_mask[Short_Motor].sw.uv_vdc = 0b1;
    faults.react_mask[Short_Motor].sw.os = 0b1;
    faults.react_mask[Short_Motor].sw.hall = 0b1;
    faults.react_mask[Short_Motor].sw.params = 0b1;

    // Unclearable faults bitmask:
    faults.unclearable_mask = (FAULT_FLAGS_t){ 0U };
    faults.unclearable_mask.sw.params = 0b1;

}

void FAULT_PROTECT_Reset()
{
    // Clear all faults
    faults.flags.all = 0U;
    faults.flags_latched.all = 0U;
    faults.reaction = No_Reaction;

    // Unclearable faults
    // Parameter incompatibilities:
    // - Single shunt sensing is not possible when using trapezoidal commutation, use block commutation instead
#if defined(CTRL_METHOD_TBC)
    faults.flags.sw.params |= (params.ctrl.tbc.mode == Trapezoidal_Commutation) && (params.sys.analog.shunt.type == Single_Shunt);
#endif

    // Motor I2T
    protect.motor.i2t.i_sq_filt = 0.0f;
    protect.motor.i2t.state = Dis;
    protect.motor.i2t.i_limit = params.motor.i_peak;

#if defined(CTRL_METHOD_SFO)
    protect.motor.T_lmt = params.motor.T_max;
#endif
}

RAMFUNC_BEGIN
void FAULT_PROTECT_RunISR0()
{
    // Motor I2T
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    vars.i_s_fb_sq = POW_TWO(vars.i_ab_fb_tot.alpha) + POW_TWO(vars.i_ab_fb_tot.beta);
#elif defined(CTRL_METHOD_TBC)
    if (params.ctrl.mode == Volt_Mode_Open_Loop)
    {
        vars.i_s_fb_sq = POW_TWO(vars.i_ab_fb_tot.alpha) + POW_TWO(vars.i_ab_fb_tot.beta);
    }
    else
    {
        vars.i_s_fb_sq = POW_TWO(vars.i_s_fb);
    }
#endif
    protect.motor.i2t.i_sq_filt += (vars.i_s_fb_sq - protect.motor.i2t.i_sq_filt) * protect.motor.i2t.filt_coeff;

}
RAMFUNC_END

void FAULT_PROTECT_RunISR1()
{
    // Protections ..............................................................................................
    // Motor I2T
    protect.motor.i2t.i_filt = sqrtf(protect.motor.i2t.i_sq_filt);
#if defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_RFO)
    vars.i_s_fb = protect.motor.i2t.i_filt;
#endif

    if ((protect.motor.i2t.i_filt >= protect.motor.i2t.i_on) && (protect.motor.i2t.state == Dis))
    {
        protect.motor.i2t.state = En;
        protect.motor.i2t.i_limit = params.motor.i_cont;
    }
    else if ((protect.motor.i2t.i_filt < protect.motor.i2t.i_off) && (protect.motor.i2t.state == En))
    {
        protect.motor.i2t.state = Dis;
        protect.motor.i2t.i_limit = params.motor.i_peak;
    }

    // Fault Detections .........................................................................................
    // OC
    faults.flags.sw.oc = (protect.motor.i2t.i_filt >= faults.vars.oc_thresh);

    // Vdc OV and UV
    if (vars.v_dc >= params.sys.faults.vdc_thresh.max)
    {
        DebounceFiltInc(&faults.vars.vdc_ov_timer);
    }
    else if (vars.v_dc <= params.sys.faults.vdc_thresh.min)
    {
        DebounceFiltInc(&faults.vars.vdc_uv_timer);
    }
    else
    {
        DebounceFiltDec(&faults.vars.vdc_ov_timer);
        DebounceFiltDec(&faults.vars.vdc_uv_timer);
    }
    faults.flags.sw.ov_vdc = DebounceFiltIsSet(&faults.vars.vdc_ov_timer);
    faults.flags.sw.uv_vdc = DebounceFiltIsSet(&faults.vars.vdc_uv_timer);

    // OT
    faults.flags.sw.ot_ps = (vars.temp_ps >= params.sys.faults.temp_ps_thresh);

    // OS
    faults.flags.sw.os = (vars.w_final_filt_abs.elec >= params.sys.faults.w_thresh.elec);
    faults.flags.sw.brk = vars.brk;
    faults.flags.sw.em_stop = vars.em_stop;

    // Fault Latching ...........................................................................................
    faults.flags_latched.all |= faults.flags.all;

    // Fault Reactions ..........................................................................................
    if (faults.flags_latched.all & faults.react_mask[Short_Motor].all)
    {
        // Method indicated by params.sys.faults.short_method
        faults.reaction = Short_Motor;
    }
    else if (faults.flags_latched.all & faults.react_mask[High_Z].all) // note that short condition is checked first
    {
        faults.reaction = High_Z;
    }

#if defined(PC_TEST)
    vars.test[21] = protect.motor.i2t.i_on;
    vars.test[22] = protect.motor.i2t.i_off;
    vars.test[23] = protect.motor.i2t.i_filt;
    vars.test[24] = protect.motor.i2t.i_limit;
    vars.test[25] = POW_TWO(protect.motor.i2t.i_limit);
    vars.test[26] = vars.i_s_fb_sq;
#if defined(CTRL_METHOD_SFO)
    vars.test[27] = protect.motor.T_lmt;
#endif
#endif

}

void FAULT_PROTECT_ClearFaults()
{
    faults.flags.all &= faults.unclearable_mask.all;
    faults.flags_latched.all &= faults.unclearable_mask.all;
    faults.reaction = No_Reaction;
}

#if defined(CTRL_METHOD_SFO)
RAMFUNC_BEGIN
void FAULT_PROTECT_RunTrqLimitCtrlISR0()	// Sliding-mode current limiter in SFO
{
    float T_lmt = protect.motor.T_lmt + params.ctrl.trq.curr_lmt_ki * SIGN(POW_TWO(protect.motor.i2t.i_limit) - vars.i_s_fb_sq);
    protect.motor.T_lmt = SAT(0.0f, params.motor.T_max, T_lmt);
}
RAMFUNC_END
#endif
