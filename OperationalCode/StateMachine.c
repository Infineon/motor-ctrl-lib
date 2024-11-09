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

STATE_MACHINE_t sm;

static void (*CommonISR0Wrap)() = EmptyFcn;
static void (*CommonISR1Wrap)() = EmptyFcn;
static void (*FeedbackISR0Wrap)() = OBS_RunISR0;

static inline bool Mode(CTRL_MODE_t ctrl_mode)
{
    return (params.ctrl.mode == ctrl_mode);
}

RAMFUNC_BEGIN
static void CommonISR0()
{
    SENSOR_IFACE_RunISR0();
    FAULT_PROTECT_RunISR0();
}
RAMFUNC_END

static void CommonISR1()
{
    SENSOR_IFACE_RunISR1();
    FAULT_PROTECT_RunISR1();
    FCN_EXE_HANDLER_RunISR1();
}

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
RAMFUNC_BEGIN
static void SensorlessFeedbackISR0()
{
    // Using threshold hysteresis
    if ((sm.vars.high_freq.used) && ABS_ABOVE_LIM(vars.w_final_filt.elec, params.obs.w_thresh.elec))
    {
        sm.vars.high_freq.used = false;
#if defined(CTRL_METHOD_RFO)
        CURRENT_CTRL_Init(1.0f);
#elif defined(CTRL_METHOD_SFO)
        FLUX_CTRL_Init(1.0f);
        ctrl.delta.bw_red_coeff = 1.0f;
#endif
    }

    if (!sm.vars.high_freq.used)
    {
        OBS_RunISR0();
    }
    else // running both observer and high frequency injection
    {
        HIGH_FREQ_INJ_RunFiltISR0();
        OBS_RunISR0();
        HIGH_FREQ_INJ_RunCtrlISR0();
    }

#if defined(PC_TEST)
    vars.test[34] = sm.vars.high_freq.used;
    vars.test[35] = obs.pll_r.th.elec;
    vars.test[36] = ctrl.high_freq_inj.integ_pll_r.integ;
    vars.test[37] = obs.pll_r.w.elec;
    vars.test[38] = ctrl.high_freq_inj.pi_pll_r.output;
#endif
}
RAMFUNC_END
#endif

void STATE_MACHINE_ResetAllModules()
{
    SENSOR_IFACE_Init();
    FAULT_PROTECT_Init();
    OBS_Init();
    CTRL_FILTS_Init();
    SPEED_CTRL_Init();
    VOLT_MOD_Init();
#if defined(CTRL_METHOD_RFO)
    PHASE_ADV_Init();
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Init(1.0f);
    TRQ_Init();
    DELTA_CTRL_Init();
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    SIX_PULSE_INJ_Init();
    HIGH_FREQ_INJ_Init();
    PROFILER_Init();
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    HALL_SENSOR_Init();
    INC_ENCODER_Init();
    CURRENT_CTRL_Init(1.0f);
#endif
    // Reset CommonISR modules first:
    SENSOR_IFACE_Reset();
    FAULT_PROTECT_Reset();
    CommonISR0Wrap = CommonISR0;
    CommonISR1Wrap = CommonISR1;
#if defined(CTRL_METHOD_RFO)
    if (Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup) ||
        Mode(Curr_Mode_FOC_Sensorless_HighFreq_Startup))
    {
        FeedbackISR0Wrap = SensorlessFeedbackISR0;
        sm.vars.high_freq.used = true;
    }
    else if (params.sys.fb.mode == Hall)
    {
        HALL_SENSOR_Reset();
        FeedbackISR0Wrap = HALL_SENSOR_RunISR0;
        sm.vars.high_freq.used = false;
    }
    else if (params.sys.fb.mode == AqB_Enc)
    {
        FeedbackISR0Wrap = INC_ENCODER_RunISR0;
        sm.vars.high_freq.used = false;
    }
    else
    {
        FeedbackISR0Wrap = OBS_RunISR0;
        sm.vars.high_freq.used = false;
    }
#elif defined(CTRL_METHOD_SFO)
    if (Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup) ||
        Mode(Trq_Mode_FOC_Sensorless_HighFreq_Startup))
    {
        FeedbackISR0Wrap = SensorlessFeedbackISR0;
        sm.vars.high_freq.used = true;
    }
    else
    {
        FeedbackISR0Wrap = OBS_RunISR0;
        sm.vars.high_freq.used = false;
    }
#elif defined(CTRL_METHOD_TBC)
    if (params.sys.fb.mode == Hall)
    {
        HALL_SENSOR_Reset();
        FeedbackISR0Wrap = HALL_SENSOR_RunISR0;
        BLOCK_COMM_Init();
        TRAP_COMM_Init();
    }
#endif
}
void STATE_MACHINE_ResetVariable(void)
{
    ELEC_t w0 = Elec_Zero;

	CTRL_FILTS_Reset();
    SENSOR_IFACE_Reset();
    SPEED_CTRL_Reset();
    VOLT_CTRL_Reset();
    CTRL_ResetWcmdInt(w0);

	vars.v_qd_r_cmd.d = 0.0f;
	vars.v_qd_r_cmd.q = 0.0f;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    ELEC_t th0 = { PI_OVER_TWO };
    if (params.sys.fb.mode == Sensorless)
    {
       AB_t la_ab_lead = (AB_t){ (params.motor.lam + params.motor.ld * params.ctrl.align.voltage / params.motor.r), 0.0f };
       OBS_Reset(&la_ab_lead, &w0, &th0);
    }
    SIX_PULSE_INJ_Reset();
    TRQ_Reset();
    HIGH_FREQ_INJ_Reset(Elec_Zero, ctrl.six_pulse_inj.th_r_est);
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
	if (params.sys.fb.mode == Hall)
	{
	  HALL_SENSOR_Reset();
	}
	CURRENT_CTRL_Reset();
    vars.i_cmd_int = 0.0f;

#endif
#if defined(CTRL_METHOD_SFO)
	FLUX_CTRL_Reset();
	DELTA_CTRL_Reset();
    vars.T_cmd_int = 0.0f;
	vars.delta_cmd.elec =0.0f;
	vars.T_cmd_final = 0.0f;
	vars.la_cmd_final =0.0f;
	vars.la_qd_s_est.d = 0.0f;
	vars.v_qd_s_cmd.d = 0.0f;
	vars.v_qd_s_cmd.q = 0.0f;
#endif
#if defined(CTRL_METHOD_RFO)
    if (params.sys.fb.mode == AqB_Enc)
    {
      INC_ENCODER_Reset(th0);
    }
	FLUX_WEAKEN_Reset();
	vars.i_qd_r_cmd.d = 0.0f;
	vars.i_qd_r_cmd.q = 0.0f;

	vars.i_qd_r_fb.d  = 0.0f;
	vars.i_qd_r_fb.q  = 0.0f;

#endif
#if defined(CTRL_METHOD_TBC)
	TRAP_COMM_Reset();
	vars.v_s_cmd.rad= 0.0f;
#endif


}
static void InitEntry()
{
    hw_fcn.GateDriverEnterHighZ();

    if (!sm.vars.init.param_init_done)	// only after booting up
    {
        hw_fcn.StopPeripherals();		// disable all ISRs, PWMs, ADCs, etc.

        CommonISR0Wrap = EmptyFcn;
        CommonISR1Wrap = EmptyFcn;
        FeedbackISR0Wrap = OBS_RunISR0;
#if !defined(PC_TEST)
        PARAMS_Init();
#endif
        hw_fcn.HardwareIfaceInit();		// all peripherals must stop before re-initializing
        STATE_MACHINE_ResetAllModules();
        sm.vars.init.param_init_done = true;
        vars.en = true;

        hw_fcn.StartPeripherals();		// enable all ISRs, PWMs, ADCs, etc.
    }
   	STATE_MACHINE_ResetVariable();

    // TBD: check offset nulling timer
    StopWatchInit(&sm.vars.init.timer, params.sys.analog.offset_null_time, params.sys.samp.ts0);
    sm.vars.speed_reset_required = false;
    sm.vars.init.offset_null_done = false;

}

RAMFUNC_BEGIN
static void InitISR0()
{
    if (sm.vars.init.param_init_done)
    {
        StopWatchRun(&sm.vars.init.timer);
        if (StopWatchIsDone(&sm.vars.init.timer))
        {
            sm.vars.init.offset_null_done = true;
        }
        else
        {
            SENSOR_IFACE_OffsetNullISR0();
        }
    }
}
RAMFUNC_END

static void BrakeBootEntry()
{
    StopWatchInit(&sm.vars.brake_boot.timer, params.sys.boot_time, params.sys.samp.ts0);

    hw_fcn.EnterCriticalSection();	// --------------------
    // atomic operations needed for struct writes and no more modification until next state
    CTRL_FILTS_Reset();
    vars.d_uvw_cmd = UVW_Zero;
    vars.w_cmd_int.elec = 0.0f;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    vars.i_cmd_int = 0.0f;
#elif defined(CTRL_METHOD_SFO)
    vars.T_cmd_int = 0.0f;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    if (Mode(Profiler_Mode))
    {
        sm.add_callback.RunISR0 = EmptyFcn;
    }
#endif
    sm.current = sm.next; // must be in critical section for brake-boot entry
    hw_fcn.GateDriverExitHighZ();
    hw_fcn.ExitCriticalSection();	// --------------------
}

RAMFUNC_BEGIN
static void BrakeBootISR0()
{
    StopWatchRun(&sm.vars.brake_boot.timer);
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    if (params.sys.fb.mode == Hall)
    {
        FeedbackISR0Wrap();
        vars.w_final.elec = vars.w_hall.elec;
        vars.th_r_final.elec = vars.th_r_hall.elec;
    }
#endif
}
RAMFUNC_END

static void BrakeBootISR1()
{
    if (sm.vars.speed_reset_required && ABS_BELOW_LIM(vars.w_cmd_ext.elec, params.ctrl.volt.w_thresh.elec - params.ctrl.volt.w_hyst.elec))
    {
        sm.vars.speed_reset_required = false;
    }
}

static void VoltHzOLEntry()
{
    CTRL_ResetWcmdInt(params.ctrl.volt.w_thresh);
    vars.v_qd_r_cmd.d = 0.0f;
    VOLT_CTRL_Reset();
    hw_fcn.EnterCriticalSection();	// --------------------
    if (params.sys.analog.shunt.type == Single_Shunt)
    {
        VOLT_MOD_EnDisHybMod(Dis);
    }
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    if (Mode(Profiler_Mode))
    {
        sm.add_callback.RunISR0 = PROFILER_RunISR0;
    }
#endif
    sm.current = sm.next; // must be in critical section
    hw_fcn.ExitCriticalSection();	// --------------------
}

static void VoltHzOLExit()
{
    if (params.sys.analog.shunt.type == Single_Shunt)
    {
        VOLT_MOD_EnDisHybMod(En);
    }
}

RAMFUNC_BEGIN
static void VoltHzOLISR0()
{
    CTRL_UpdateWcmdIntISR0(vars.w_cmd_ext);
    vars.v_qd_r_cmd.q = MAX(params.ctrl.volt.v_min + ABS(vars.w_cmd_int.elec) * params.ctrl.volt.v_to_f_ratio, 0.0f) * vars.dir;
    VOLT_CTRL_RunISR0();
    VOLT_MOD_RunISR0();
    CTRL_FILTS_RunAllISR0();
}
RAMFUNC_END

static void SpeedCLEntry()
{
#if defined(CTRL_METHOD_RFO)
    if ((sm.current == Align) || (sm.current == Six_Pulse) || (sm.current == High_Freq) || (sm.current == Brake_Boot))
    {
        ELEC_t w0 = (((sm.current == Align) || (sm.current == Six_Pulse)) && (params.sys.fb.mode == Sensorless)) ? params.obs.w_thresh : params.ctrl.volt.w_thresh;
        CTRL_ResetWcmdInt(w0);
        SPEED_CTRL_Reset();
        vars.i_cmd_int = 0.0f;
        FLUX_WEAKEN_Reset();
    }
#elif defined(CTRL_METHOD_SFO)
    if ((sm.current == Align) || (sm.current == Six_Pulse) || (sm.current == High_Freq))
    {
        CTRL_ResetWcmdInt((sm.current == High_Freq) ? params.ctrl.volt.w_thresh : params.obs.w_thresh);
        SPEED_CTRL_Reset();
        vars.T_cmd_int = 0.0f;
    }
#elif defined(CTRL_METHOD_TBC)
    if (sm.current == Brake_Boot)
    {
        CTRL_ResetWcmdInt(params.ctrl.volt.w_thresh);
        SPEED_CTRL_Reset();
        vars.i_cmd_int = 0.0f;
        CURRENT_CTRL_Reset();
        TRAP_COMM_Reset();
    }
#endif
}

#if defined(CTRL_METHOD_SFO)
RAMFUNC_BEGIN
static inline void AddHighFreqVoltsIfNeeded()
{	// adding high frequency excitation voltage components if needed
    if (sm.vars.high_freq.used)
    {
        vars.v_ab_cmd_tot.alpha += ctrl.high_freq_inj.v_ab_cmd.alpha;
        vars.v_ab_cmd_tot.beta += ctrl.high_freq_inj.v_ab_cmd.beta;
    }
}
RAMFUNC_END
#endif

RAMFUNC_BEGIN
static void SpeedCLISR0()
{
    CTRL_UpdateWcmdIntISR0(vars.w_cmd_ext);
    FeedbackISR0Wrap();
    switch (params.sys.fb.mode)
    {
    default:
    case Sensorless:
        vars.w_final.elec = vars.w_est.elec;
        vars.th_r_final.elec = vars.th_r_est.elec;
        break;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    case Hall:
        vars.w_final.elec = vars.w_hall.elec;
        vars.th_r_final.elec = vars.th_r_hall.elec;
        break;
    case AqB_Enc:
        vars.w_final.elec = vars.w_enc.elec;
        vars.th_r_final.elec = vars.th_r_enc.elec;
        break;
#endif
    }
    CTRL_FILTS_RunAllISR0();
    SPEED_CTRL_RunISR0();
#if defined(CTRL_METHOD_RFO)
    vars.i_cmd_prot = SAT(-protect.motor.i2t.i_limit, protect.motor.i2t.i_limit, vars.i_cmd_spd);
    vars.i_cmd_int = RateLimit(params.sys.rate_lim.i_cmd * params.sys.samp.ts0, vars.i_cmd_prot, vars.i_cmd_int);
    PHASE_ADV_RunISR0();
    FLUX_WEAKEN_RunISR0();
    CURRENT_CTRL_RunISR0();
    TRQ_RunObsISR0();
    VOLT_MOD_RunISR0();
#elif defined(CTRL_METHOD_SFO)
    FAULT_PROTECT_RunTrqLimitCtrlISR0();
    vars.T_cmd_prot = SAT(-protect.motor.T_lmt, protect.motor.T_lmt, vars.T_cmd_spd);
    vars.T_cmd_int = RateLimit(params.sys.rate_lim.T_cmd * params.sys.samp.ts0, vars.T_cmd_prot, vars.T_cmd_int);
    vars.la_cmd_mtpa = LUT1DInterp(&params.motor.mtpa_lut, ABS(vars.T_cmd_int));
    FLUX_WEAKEN_RunISR0();
    FLUX_CTRL_RunISR0();
    TRQ_RunObsISR0();
    TRQ_RunCtrlISR0();
    DELTA_CTRL_RunISR0();
    ParkTransformInv(&vars.v_qd_s_cmd, &vars.park_s, &vars.v_ab_cmd);
    vars.v_ab_cmd_tot = vars.v_ab_cmd;
    AddHighFreqVoltsIfNeeded();
    VOLT_MOD_RunISR0();
#elif defined(CTRL_METHOD_TBC)
    vars.i_cmd_prot = (params.ctrl.curr.bypass == false) ? SAT(-protect.motor.i2t.i_limit, protect.motor.i2t.i_limit, vars.i_cmd_spd) : vars.i_cmd_spd;
    vars.i_cmd_int = RateLimit(params.sys.rate_lim.i_cmd * params.sys.samp.ts0, vars.i_cmd_prot, vars.i_cmd_int);
    switch (params.ctrl.tbc.mode)
    {
    case Block_Commutation:
    default:
        CURRENT_CTRL_RunISR0();
        BLOCK_COMM_RunVoltModISR0();
        break;
    case Trapezoidal_Commutation:
        TRAP_COMM_RunISR0();
        break;
    }
    TRQ_RunObsISR0();
#endif
}
RAMFUNC_END

static void FaultEntry()
{
    UVW_t d_uvw_cmd = UVW_Zero;
    if (faults.reaction == Short_Motor)
    {
        hw_fcn.GateDriverExitHighZ();
#if defined(CTRL_METHOD_TBC)
        ctrl.block_comm.exit_high_z_flag.u = true;
        ctrl.block_comm.exit_high_z_flag.v = true;
        ctrl.block_comm.exit_high_z_flag.w = true;
#endif
        switch (params.sys.faults.short_method)
        {
        case Low_Side_Short:
            d_uvw_cmd = UVW_Zero;
            break;
        case High_Side_Short:
            d_uvw_cmd = UVW_One;
            break;
        case Alternate_Short:
        default:
            d_uvw_cmd = UVW_Half;
            break;
        }
    }
    else if (faults.reaction == High_Z)
    {
        hw_fcn.GateDriverEnterHighZ();
#if defined(CTRL_METHOD_TBC)
        ctrl.block_comm.enter_high_z_flag.w = true;
        ctrl.block_comm.enter_high_z_flag.v = true;
        ctrl.block_comm.enter_high_z_flag.w = true;
#endif
    } // "No_Reaction" is not reachable

    sm.vars.fault.clr_success = false;
    sm.vars.fault.clr_request = false;

    hw_fcn.EnterCriticalSection();	// --------------------
    CTRL_FILTS_Reset();
    vars.d_uvw_cmd = d_uvw_cmd;
    sm.current = sm.next;			// must be in critical section
    hw_fcn.ExitCriticalSection();	// --------------------
}

static void FaultISR1()
{
    bool clr_faults_and_count = RISE_EDGE(sm.vars.fault.clr_faults_prev, vars.clr_faults) && (!faults.flags_latched.sw.brk) && (!faults.flags_latched.sw.em_stop);
    bool clr_faults_no_count = ((!faults.flags.sw.brk) && (faults.flags_latched.sw.brk)) || ((!faults.flags.sw.em_stop) && (faults.flags_latched.sw.em_stop));
    sm.vars.fault.clr_faults_prev = vars.clr_faults;

    if (sm.vars.fault.clr_request)
    {
        sm.vars.fault.clr_request = false;
        FAULT_PROTECT_ClearFaults();
        sm.vars.fault.clr_success = true;
    }
    else if (clr_faults_and_count && (sm.vars.fault.clr_try_cnt < params.sys.faults.max_clr_tries))
    {
        sm.vars.fault.clr_request = true;
        ++sm.vars.fault.clr_try_cnt;
    }
    else if (clr_faults_no_count)
    {
        sm.vars.fault.clr_request = true;
    }

}

static void FaultExit()
{
    hw_fcn.GateDriverExitHighZ();
#if defined(CTRL_METHOD_TBC)
    ctrl.block_comm.exit_high_z_flag.u = true;
    ctrl.block_comm.exit_high_z_flag.v = true;
    ctrl.block_comm.exit_high_z_flag.w = true;
#endif
}

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

static void AlignEntry()
{
    VOLT_CTRL_Reset();
    vars.v_qd_r_cmd = (QD_t){ params.ctrl.align.voltage, 0.0f };
    StopWatchInit(&sm.vars.align.timer, params.ctrl.align.time, params.sys.samp.ts0);
}

RAMFUNC_BEGIN
static void AlignISR0()
{
    VOLT_CTRL_RunISR0();
    VOLT_MOD_RunISR0();
    StopWatchRun(&sm.vars.align.timer);
}
RAMFUNC_END

static void AlignExit()
{
    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = { PI_OVER_TWO };

#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Reset();
    if (params.sys.fb.mode == AqB_Enc)
    {
        INC_ENCODER_Reset(th0);
    }
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Reset();
    TRQ_Reset();
    DELTA_CTRL_Reset();
#endif
    if (params.sys.fb.mode == Sensorless)
    {
        AB_t la_ab_lead = (AB_t){ (params.motor.lam + params.motor.ld * params.ctrl.align.voltage / params.motor.r), 0.0f };
        OBS_Reset(&la_ab_lead, &w0, &th0);
    }
}

static void SixPulseEntry()
{
    SIX_PULSE_INJ_Reset();
}

RAMFUNC_BEGIN
static void SixPulseISR0()
{
    SIX_PULSE_INJ_RunISR0();
}
RAMFUNC_END

static void SixPulseISR1()
{
    SIX_PULSE_INJ_RunISR1();
}

static void SixPulseExit()
{
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Reset();
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Reset();
    TRQ_Reset();
    DELTA_CTRL_Reset();
#endif

    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = ctrl.six_pulse_inj.th_r_est;
    PARK_t park_r;
    ParkInit(th0.elec, &park_r);

    QD_t la_qd_r = (QD_t){ 0.0f, params.motor.lam };
    AB_t la_ab_lead;
    ParkTransformInv(&la_qd_r, &park_r, &la_ab_lead);

    OBS_Reset(&la_ab_lead, &w0, &th0);
}

static void HighFreqEntry()
{
    HIGH_FREQ_INJ_Reset(Elec_Zero, ctrl.six_pulse_inj.th_r_est);
    StopWatchInit(&sm.vars.high_freq.timer, params.ctrl.high_freq_inj.lock_time, params.sys.samp.ts0);
    sm.vars.high_freq.used = true;
    vars.v_ab_cmd = AB_Zero;
}

RAMFUNC_BEGIN
static void HighFreqISR0()
{
    StopWatchRun(&sm.vars.high_freq.timer);
    FeedbackISR0Wrap();
    vars.w_final.elec = vars.w_est.elec;
    vars.th_r_final.elec = vars.th_r_est.elec;
    CTRL_FILTS_RunSpeedISR0();
    vars.v_ab_cmd_tot = ctrl.high_freq_inj.v_ab_cmd;
    VOLT_MOD_RunISR0();
}
RAMFUNC_END

static void HighFreqExit()
{
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Init(params.ctrl.high_freq_inj.bw_red_coeff);
    CURRENT_CTRL_Reset();
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Init(params.ctrl.high_freq_inj.bw_red_coeff);
    ctrl.delta.bw_red_coeff = params.ctrl.high_freq_inj.bw_red_coeff;
#endif
}

static void SpeedOLToCLEntry()
{
    AB_t la_ab_lead = AB_Zero;
    ELEC_t w0 = (ELEC_t){ params.obs.w_thresh.elec * vars.dir };
    ELEC_t th0 = Elec_Zero;
    OBS_Reset(&la_ab_lead, &w0, &th0);
    TRQ_Reset();
    StopWatchInit(&sm.vars.speed_ol_to_cl.timer, params.obs.lock_time, params.sys.samp.ts0);
}

RAMFUNC_BEGIN
static void SpeedOLToCLISR0()
{
    CTRL_UpdateWcmdIntISR0(vars.w_cmd_ext);
    FeedbackISR0Wrap();
    ELEC_t th_r_trq = { vars.th_r_est.elec };
    PARK_t park_r_trq;
    ParkInit(th_r_trq.elec, &park_r_trq);
    ParkTransform(&vars.i_ab_fb, &park_r_trq, &vars.i_qd_r_fb);
    TRQ_RunObsISR0();

    vars.v_qd_r_cmd.q = MAX(params.ctrl.volt.v_min + ABS(vars.w_cmd_int.elec) * params.ctrl.volt.v_to_f_ratio, 0.0f) * vars.dir;
    VOLT_CTRL_RunISR0();
    VOLT_MOD_RunISR0();
    CTRL_FILTS_RunAllISR0();

    StopWatchRun(&sm.vars.speed_ol_to_cl.timer);
}
RAMFUNC_END

static void SpeedOLToCLExit()
{
    SPEED_CTRL_Reset();

    PARK_t park_r;
    QD_t i_qd_r_fb;

    hw_fcn.EnterCriticalSection();	// --------------------
    // atomic operations needed for struct copies and all data must have the same time stamp
    AB_t i_ab_fb = vars.i_ab_fb;
    float th_r_est = vars.th_r_est.elec;
    hw_fcn.ExitCriticalSection();	// --------------------

    ParkInit(th_r_est, &park_r);
    ParkTransform(&i_ab_fb, &park_r, &i_qd_r_fb);

#if defined(CTRL_METHOD_RFO)
    float i_cmd_spd;
    PHASE_ADV_CalcOptIs(&i_qd_r_fb, &i_cmd_spd);
    vars.i_cmd_int = i_cmd_spd * params.ctrl.speed.ol_cl_tr_coeff;
    SPEED_CTRL_IntegBackCalc(vars.i_cmd_int);

#if defined(PC_TEST)
    vars.test[30] = i_qd_r_fb.q;
    vars.test[31] = i_qd_r_fb.d;
    vars.test[32] = i_cmd_spd;
#endif

    FLUX_WEAKEN_Reset();
    CURRENT_CTRL_Reset();

#elif defined(CTRL_METHOD_SFO)
    float T_cmd_spd;
    TRQ_CalcTrq(&i_qd_r_fb, &T_cmd_spd);
    vars.T_cmd_int = T_cmd_spd * params.ctrl.speed.ol_cl_tr_coeff;
    SPEED_CTRL_IntegBackCalc(vars.T_cmd_int);

    FLUX_CTRL_Reset();
    DELTA_CTRL_Reset();

#endif
}

static void DynoLockEntry()
{
    StopWatchInit(&sm.vars.dyno_lock.timer, params.sys.dyno_lock_time, params.sys.samp.ts0);

    hw_fcn.EnterCriticalSection();	// --------------------
    // atomic operations needed for struct writes and no more modification until next state

    hw_fcn.GateDriverEnterHighZ();
    vars.d_uvw_cmd = UVW_Zero;

    vars.v_ab_obs = &vars.v_ab_fb;
    AB_t la_ab_lead = AB_Zero;
    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = Elec_Zero;
    OBS_Reset(&la_ab_lead, &w0, &th0);
    CTRL_FILTS_Reset();

    sm.current = sm.next; // must be in critical section for dyno_lock entry

    hw_fcn.ExitCriticalSection();	// --------------------
}

RAMFUNC_BEGIN
static void DynoLockISR0()
{
    StopWatchRun(&sm.vars.dyno_lock.timer);
    FeedbackISR0Wrap();
    switch (params.sys.fb.mode)
    {
    default:
    case Sensorless:
        vars.w_final.elec = vars.w_est.elec;
        vars.th_r_final.elec = vars.th_r_est.elec;
        break;
#if defined(CTRL_METHOD_RFO)
    case Hall:
        vars.w_final.elec = vars.w_hall.elec;
        vars.th_r_final.elec = vars.th_r_hall.elec;
        break;
#endif
    }
    CTRL_FILTS_RunSpeedISR0();
}
RAMFUNC_END

static void DynoLockExit()
{
    TRQ_Reset();
#if defined(CTRL_METHOD_RFO)
    //	Ideal case in RFO:
    //		1) i_qd_r_cmd == i_qd_r_fb == {0.0f,0.0f}
    //		2) v_qd_r_cmd == v_qd_r_fb == {lam*w, 0.0f}
    //		3) v_ab_cmd == v_ab_fb == ParkInv(v_qd_r_fb, th_r)
    //		4) curr.pi_q.error == curr.pi_d.error == curr.pi_q.output == curr.pi_d.output == 0.0f (when curr.ff_coef == 1.0f)
    vars.i_cmd_int = 0.0f;
    CURRENT_CTRL_Reset();
#elif defined(CTRL_METHOD_SFO)
    //	Ideal case in SFO:
    //		1) i_qd_s_cmd == i_qd_s_fb == {0.0f,0.0f}
    //		2) delta == 0.0f, th_s = th_r
    //		3) v_qd_s_cmd == v_qd_s_fb == {lam*w, 0.0f}
    //		4) v_ab_cmd == v_ab_fb == ParkInv(v_qd_s_fb, th_s)
    //		5) trq.pi.error == trq.pi.output == flux.pi.error == flux.pi.output == 0.0f
    //		6) delta.pi.output = v_qd_s_cmd.q = lam*w
    vars.T_cmd_int = 0.0f;
    FLUX_CTRL_Reset();
    DELTA_CTRL_Reset();
    ctrl.delta.pi.output = params.motor.lam * vars.w_final_filt.elec;
    PI_IntegBackCalc(&ctrl.delta.pi, ctrl.delta.pi.output, 0.0f, 0.0f);
#endif
    hw_fcn.EnterCriticalSection();		// --------------------
    vars.v_ab_cmd = vars.v_ab_fb;	// atomic operations needed
    vars.v_ab_obs = &vars.v_ab_cmd;
    hw_fcn.GateDriverExitHighZ();
    hw_fcn.ExitCriticalSection();		// --------------------
}

static void MotorProfEntry()
{
    hw_fcn.EnterCriticalSection();		// --------------------
    PROFILER_Entry();
    sm.current = sm.next; // must be in critical section
    hw_fcn.ExitCriticalSection();		// --------------------
}

static void MotorProfISR0()
{
    PROFILER_RunISR0();
    VOLT_MOD_RunISR0();
}

static void MotorProfExit()
{
    PROFILER_Exit();
}

#endif

#if defined(CTRL_METHOD_SFO)

static void TorqueCLEntry()
{
    vars.T_cmd_int = params.ctrl.trq.T_cmd_thresh * vars.dir;
}

RAMFUNC_BEGIN
static void TorqueCLISR0()
{
    FeedbackISR0Wrap();
    vars.w_final.elec = vars.w_est.elec;
    vars.th_r_final.elec = vars.th_r_est.elec;
    CTRL_FILTS_RunSpeedISR0();
    FAULT_PROTECT_RunTrqLimitCtrlISR0();
    vars.T_cmd_prot = SAT(-protect.motor.T_lmt, protect.motor.T_lmt, vars.T_cmd_ext);
    vars.T_cmd_int = RateLimit(params.sys.rate_lim.T_cmd * params.sys.samp.ts0, vars.T_cmd_prot, vars.T_cmd_int);
    vars.la_cmd_mtpa = LUT1DInterp(&params.motor.mtpa_lut, ABS(vars.T_cmd_int));
    vars.la_cmd_final = vars.la_cmd_mtpa;		// No flux weakening in torque control mode
    FLUX_CTRL_RunISR0();
    TRQ_RunObsISR0();
    TRQ_RunCtrlISR0();
    DELTA_CTRL_RunISR0();
    ParkTransformInv(&vars.v_qd_s_cmd, &vars.park_s, &vars.v_ab_cmd);
    vars.v_ab_cmd_tot = vars.v_ab_cmd;
    AddHighFreqVoltsIfNeeded();
    VOLT_MOD_RunISR0();
}
RAMFUNC_END

#elif defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)

static void CurrentCLEntry()
{
    vars.i_cmd_int = params.ctrl.curr.i_cmd_thresh * vars.dir;
}

RAMFUNC_BEGIN
static void CurrentCLISR0()
{
    FeedbackISR0Wrap();
    switch (params.sys.fb.mode)
    {
    default:
    case Sensorless:
        vars.w_final.elec = vars.w_est.elec;
        vars.th_r_final.elec = vars.th_r_est.elec;
        break;
    case Hall:
        vars.w_final.elec = vars.w_hall.elec;
        vars.th_r_final.elec = vars.th_r_hall.elec;
        break;
    case AqB_Enc:
        vars.w_final.elec = vars.w_enc.elec;
        vars.th_r_final.elec = vars.th_r_enc.elec;
        break;
    }
    CTRL_FILTS_RunSpeedISR0();
    vars.i_cmd_prot = SAT(-protect.motor.i2t.i_limit, protect.motor.i2t.i_limit, vars.i_cmd_ext);
    vars.i_cmd_int = RateLimit(params.sys.rate_lim.i_cmd * params.sys.samp.ts0, vars.i_cmd_prot, vars.i_cmd_int);
#if defined(CTRL_METHOD_RFO)
    PHASE_ADV_RunISR0();
    vars.i_qd_r_cmd = vars.i_qd_r_ref;		// No flux weakening in current control mode
    CURRENT_CTRL_RunISR0();
    VOLT_MOD_RunISR0();
#elif defined(CTRL_METHOD_TBC)
    switch (params.ctrl.tbc.mode)
    {
    case Block_Commutation:
    default:
        CURRENT_CTRL_RunISR0();
        BLOCK_COMM_RunVoltModISR0();
        break;
    case Trapezoidal_Commutation:
        TRAP_COMM_RunISR0();
        break;
    }
#endif
    TRQ_RunObsISR0();
}
RAMFUNC_END

#endif

static void ConditionCheck()
{
    STATE_ID_t current = sm.current;
    STATE_ID_t next = current;


    bool fault_trigger = (faults.reaction != No_Reaction);
    bool no_speed_reset_required = !sm.vars.speed_reset_required;

    float w_cmd_ext_abs = ABS(vars.w_cmd_ext.elec);
    float w_cmd_int_abs = ABS(vars.w_cmd_int.elec);
    float w_thresh_above_low = params.ctrl.volt.w_thresh.elec;
    float w_thresh_below_low = params.ctrl.volt.w_thresh.elec - params.ctrl.volt.w_hyst.elec;
    float w_thresh_above_high = params.obs.w_thresh.elec;
    float w_thresh_below_high = params.obs.w_thresh.elec - params.obs.w_hyst.elec;

    bool w_cmd_ext_above_thresh_low = w_cmd_ext_abs > w_thresh_above_low;
    bool w_cmd_ext_below_thresh_low = w_cmd_ext_abs < w_thresh_below_low;
    bool w_cmd_int_above_thresh_low = w_cmd_int_abs > w_thresh_above_low;
    bool w_cmd_int_below_thresh_low = w_cmd_int_abs < w_thresh_below_low;

    bool w_cmd_ext_above_thresh_high = w_cmd_ext_abs > w_thresh_above_high;
    bool w_cmd_ext_below_thresh_high = w_cmd_ext_abs < w_thresh_below_high;
    bool w_cmd_int_above_thresh_high = w_cmd_int_abs > w_thresh_above_high;
    bool w_cmd_int_below_thresh_high = w_cmd_int_abs < w_thresh_below_high;

    (void)w_cmd_ext_above_thresh_low;
    (void)w_cmd_ext_below_thresh_low;
    (void)w_cmd_int_above_thresh_low;
    (void)w_cmd_int_below_thresh_low;
    (void)w_cmd_ext_above_thresh_high;
    (void)w_cmd_ext_below_thresh_high;
    (void)w_cmd_int_above_thresh_high;
    (void)w_cmd_int_below_thresh_high;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    float i_cmd_ext_abs = ABS(vars.i_cmd_ext);
    float i_cmd_int_abs = ABS(vars.i_cmd_int);
    float i_thresh_above = params.ctrl.curr.i_cmd_thresh;
    float i_thresh_below = params.ctrl.curr.i_cmd_thresh - params.ctrl.curr.i_cmd_hyst;

    bool i_cmd_ext_above_thresh = i_cmd_ext_abs > i_thresh_above;
    bool i_cmd_ext_below_thresh = i_cmd_ext_abs < i_thresh_below;
    bool i_cmd_int_above_thresh = i_cmd_int_abs > i_thresh_above;
    bool i_cmd_int_below_thresh = i_cmd_int_abs < i_thresh_below;

    (void)i_cmd_ext_above_thresh;
    (void)i_cmd_ext_below_thresh;
    (void)i_cmd_int_above_thresh;
    (void)i_cmd_int_below_thresh;

#elif defined(CTRL_METHOD_SFO)
    float T_cmd_ext_abs = ABS(vars.T_cmd_ext);
    float T_cmd_int_abs = ABS(vars.T_cmd_int);
    float T_thresh_above = params.ctrl.trq.T_cmd_thresh;
    float T_thresh_below = params.ctrl.trq.T_cmd_thresh - params.ctrl.trq.T_cmd_hyst;

    bool T_cmd_ext_above_thresh = T_cmd_ext_abs > T_thresh_above;
    bool T_cmd_ext_below_thresh = T_cmd_ext_abs < T_thresh_below;
    bool T_cmd_int_above_thresh = T_cmd_int_abs > T_thresh_above;
    bool T_cmd_int_below_thresh = T_cmd_int_abs < T_thresh_below;

    (void)T_cmd_ext_above_thresh;
    (void)T_cmd_ext_below_thresh;
    (void)T_cmd_int_above_thresh;
    (void)T_cmd_int_below_thresh;
#endif

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    float cmd_thresh_above = params.profiler.cmd_thresh;
    float cmd_thresh_below = params.profiler.cmd_thresh - params.profiler.cmd_hyst;

    bool cmd_above_thresh = vars.cmd_final > cmd_thresh_above;
    bool cmd_below_thresh = vars.cmd_final < cmd_thresh_below;

    (void)cmd_above_thresh;
    (void)cmd_below_thresh;
#endif

    switch (current)
    {
    default:
    case Init:
        if (fault_trigger)
        {
            next = Fault;
        }
        else if (sm.vars.init.param_init_done && sm.vars.init.offset_null_done && vars.en)
        {
#if defined(CTRL_METHOD_RFO)
            if (Mode(Curr_Mode_FOC_Sensorless_Dyno))
            {
                next = Dyno_Lock;
            }
            else
            {
                next = Brake_Boot;
            }
#elif defined(CTRL_METHOD_SFO)
            if (Mode(Trq_Mode_FOC_Sensorless_Dyno))
            {
                next = Dyno_Lock;
            }
            else
            {
                next = Brake_Boot;
            }
#elif defined(CTRL_METHOD_TBC)
            next = Brake_Boot;
#endif
        }
        break;
    case Brake_Boot:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm.vars.brake_boot.timer))
        {
#if defined(CTRL_METHOD_RFO)
            if (cmd_above_thresh && Mode(Profiler_Mode))
            {
                next = Prof_Rot_Lock;
            }
            else if (w_cmd_ext_above_thresh_low && (Mode(Volt_Mode_Open_Loop) || (Mode(Speed_Mode_FOC_Sensorless_Volt_Startup) && no_speed_reset_required)))
            {
                next = Volt_Hz_OL;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(Speed_Mode_FOC_Encoder_Align_Startup))) || 
                (i_cmd_ext_above_thresh && (Mode(Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(Curr_Mode_FOC_Encoder_Align_Startup))))
            {
                next = Align;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (i_cmd_ext_above_thresh && (Mode(Curr_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Curr_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Six_Pulse;
            }
            else if (i_cmd_ext_above_thresh && Mode(Curr_Mode_FOC_Hall))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(Speed_Mode_FOC_Hall))
            {
                next = Speed_CL;
            }
#elif defined(CTRL_METHOD_SFO)
            if (cmd_above_thresh && Mode(Profiler_Mode))
            {
                next = Prof_Rot_Lock;
            }
            else if (w_cmd_ext_above_thresh_low && (Mode(Volt_Mode_Open_Loop) || (Mode(Speed_Mode_FOC_Sensorless_Volt_Startup) && no_speed_reset_required)))
            {
                next = Volt_Hz_OL;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(Speed_Mode_FOC_Sensorless_Align_Startup)) || (T_cmd_ext_above_thresh && Mode(Trq_Mode_FOC_Sensorless_Align_Startup)))
            {
                next = Align;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (T_cmd_ext_above_thresh && (Mode(Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Trq_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Six_Pulse;
            }
#elif defined(CTRL_METHOD_TBC)
            if (w_cmd_ext_above_thresh_low && Mode(Volt_Mode_Open_Loop))
            {
                next = Volt_Hz_OL;
            }
            else if (i_cmd_ext_above_thresh && Mode(Curr_Mode_Block_Comm_Hall))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(Speed_Mode_Block_Comm_Hall))
            {
                next = Speed_CL;
            }
#endif
        }
        break;
    case Volt_Hz_OL:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
        else if ((w_cmd_int_below_thresh_low && (Mode(Volt_Mode_Open_Loop) || Mode(Speed_Mode_FOC_Sensorless_Volt_Startup)))
            || (cmd_below_thresh && Mode(Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (w_cmd_int_above_thresh_high && (Mode(Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(Profiler_Mode)))
        {
            sm.vars.speed_reset_required = true;
            next = Speed_OL_To_CL;
        }
#elif defined(CTRL_METHOD_TBC)
        else if (w_cmd_int_below_thresh_low && Mode(Volt_Mode_Open_Loop))
        {
            next = Brake_Boot;
        }
#endif
        break;

#if defined(CTRL_METHOD_RFO)
    case Align:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm.vars.align.timer))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(Speed_Mode_FOC_Encoder_Align_Startup))) || 
                (i_cmd_ext_below_thresh && (Mode(Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(Curr_Mode_FOC_Encoder_Align_Startup))))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && (Mode(Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(Curr_Mode_FOC_Encoder_Align_Startup)))
            {
                next = Current_CL;
            }
            else if ((w_cmd_ext_above_thresh_high && Mode(Speed_Mode_FOC_Sensorless_Align_Startup)) || (w_cmd_ext_above_thresh_low && Mode(Speed_Mode_FOC_Encoder_Align_Startup)))
            {
                sm.vars.speed_reset_required = (Mode(Speed_Mode_FOC_Sensorless_Align_Startup) == true);
                next = Speed_CL; // S061987
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Align:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm.vars.align.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(Speed_Mode_FOC_Sensorless_Align_Startup)) || (T_cmd_ext_below_thresh && Mode(Trq_Mode_FOC_Sensorless_Align_Startup)))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(Trq_Mode_FOC_Sensorless_Align_Startup))
            {
                next = Torque_CL;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(Speed_Mode_FOC_Sensorless_Align_Startup))
            {
                sm.vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Six_Pulse:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((ctrl.six_pulse_inj.state == Finished_Success) || (ctrl.six_pulse_inj.state == Finished_Ambiguous))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (i_cmd_ext_below_thresh && (Mode(Curr_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Curr_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && Mode(Curr_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                next = Current_CL;
            }
            else if (i_cmd_ext_above_thresh && Mode(Curr_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(Speed_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                sm.vars.speed_reset_required = true;
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Six_Pulse:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((ctrl.six_pulse_inj.state == Finished_Success) || (ctrl.six_pulse_inj.state == Finished_Ambiguous))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (T_cmd_ext_below_thresh && (Mode(Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Trq_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(Trq_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                next = Torque_CL;
            }
            else if (T_cmd_ext_above_thresh && Mode(Trq_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(Speed_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                sm.vars.speed_reset_required = true;
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case High_Freq:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm.vars.high_freq.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup)) || (i_cmd_ext_below_thresh && Mode(Curr_Mode_FOC_Sensorless_HighFreq_Startup)))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && Mode(Curr_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                sm.vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case High_Freq:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm.vars.high_freq.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup)) || (T_cmd_ext_below_thresh && Mode(Trq_Mode_FOC_Sensorless_HighFreq_Startup)))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(Trq_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = Torque_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                sm.vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Speed_OL_To_CL:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((w_cmd_int_below_thresh_high && Mode(Speed_Mode_FOC_Sensorless_Volt_Startup))
            || (cmd_below_thresh && Mode(Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (StopWatchIsDone(&sm.vars.speed_ol_to_cl.timer))
        {
            next = Speed_CL;
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Speed_OL_To_CL:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((w_cmd_int_below_thresh_high && Mode(Speed_Mode_FOC_Sensorless_Volt_Startup))
            || (cmd_below_thresh && Mode(Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (StopWatchIsDone(&sm.vars.speed_ol_to_cl.timer))
        {
            next = Speed_CL;
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Dyno_Lock:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm.vars.dyno_lock.timer))
        {
            if (i_cmd_ext_above_thresh && Mode(Curr_Mode_FOC_Sensorless_Dyno))
            {
                next = Current_CL;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Dyno_Lock:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm.vars.dyno_lock.timer))
        {
            if (T_cmd_ext_above_thresh && Mode(Trq_Mode_FOC_Sensorless_Dyno))
            {
                next = Torque_CL;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    case Prof_Rot_Lock:
    case Prof_R:
    case Prof_Ld:
    case Prof_Lq:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        if (cmd_below_thresh)
        {
            next = Brake_Boot;
        }
        else if (StopWatchIsDone(&profiler.timer))  // timer's period is dynamically set
        {
            next = (STATE_ID_t)(((uint8_t)(current)) + 1U);
        }
        break;
    case Prof_Finished:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        if (cmd_below_thresh)
        {
            next = Brake_Boot;
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Current_CL:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (i_cmd_int_below_thresh && (Mode(Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(Curr_Mode_FOC_Sensorless_SixPulse_Startup) || 
            Mode(Curr_Mode_FOC_Sensorless_HighFreq_Startup) || Mode(Curr_Mode_FOC_Encoder_Align_Startup) || Mode(Curr_Mode_FOC_Hall)))
        {
            next = Brake_Boot;
        }
        else if (i_cmd_int_below_thresh && Mode(Curr_Mode_FOC_Sensorless_Dyno))
        {
            next = Dyno_Lock;
        }
        break;
#elif defined(CTRL_METHOD_TBC)
    case Current_CL:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (i_cmd_int_below_thresh && Mode(Curr_Mode_Block_Comm_Hall))
        {
            next = Brake_Boot;
        }
        break;
#endif
#if defined(CTRL_METHOD_SFO)
    case Torque_CL:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (T_cmd_int_below_thresh && (Mode(Trq_Mode_FOC_Sensorless_Align_Startup) || Mode(Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Trq_Mode_FOC_Sensorless_HighFreq_Startup)))
        {
            next = Brake_Boot;
        }
        else if (T_cmd_int_below_thresh && Mode(Trq_Mode_FOC_Sensorless_Dyno))
        {
            next = Dyno_Lock;
        }
        break;
#endif
    case Speed_CL:
        if (!vars.en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
#if defined(CTRL_METHOD_RFO)
        else if ((w_cmd_int_below_thresh_high && (Mode(Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Speed_Mode_FOC_Sensorless_Volt_Startup)))
            || ((sm.vars.high_freq.used ? w_cmd_int_below_thresh_low : w_cmd_int_below_thresh_high) && Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            || (w_cmd_int_below_thresh_low && (Mode(Speed_Mode_FOC_Encoder_Align_Startup) || Mode(Speed_Mode_FOC_Hall)))
            || (cmd_below_thresh && Mode(Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if ((profiler.ramp_down_status == Task_Finished) && Mode(Profiler_Mode))
        {
            next = Prof_Finished;
        }
        break;
#elif defined(CTRL_METHOD_SFO)
        else if ((w_cmd_int_below_thresh_high && (Mode(Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(Speed_Mode_FOC_Sensorless_Volt_Startup)))
            || ((sm.vars.high_freq.used ? w_cmd_int_below_thresh_low : w_cmd_int_below_thresh_high) && Mode(Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            || (cmd_below_thresh && Mode(Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if ((profiler.ramp_down_status == Task_Finished) && Mode(Profiler_Mode))
        {
            next = Prof_Finished;
        }
        break;
#elif defined(CTRL_METHOD_TBC)
        else if (w_cmd_int_below_thresh_low && Mode(Speed_Mode_Block_Comm_Hall))
        {
            next = Brake_Boot;
        }
        break;
#endif
    case Fault:
        if ((!vars.en) || sm.vars.fault.clr_success)
        {
            next = Init;
        }
        break;
    }
    sm.next = next;

}

void STATE_MACHINE_Init()
{
#if defined(PC_TEST)
    hw_fcn.HardwareIfaceInit = EmptyFcn;
    hw_fcn.EnterCriticalSection = EmptyFcn;
    hw_fcn.ExitCriticalSection = EmptyFcn;
    hw_fcn.GateDriverEnterHighZ = EmptyFcn;
    hw_fcn.GateDriverExitHighZ = EmptyFcn;
    hw_fcn.StartPeripherals = EmptyFcn;
    hw_fcn.StopPeripherals = EmptyFcn;
    hw_fcn.ArePhaseVoltagesMeasured = AlwaysTrue;
#endif

    sm.add_callback.RunISR0 = EmptyFcn;
    sm.add_callback.RunISR1 = EmptyFcn;

    //									       Entry(),				Exit(),				    RunISR0(),			    RunISR1()
    sm.states[Init]             = (STATE_t){ &InitEntry,			&EmptyFcn,			    &InitISR0,			    &EmptyFcn       };
    sm.states[Brake_Boot]       = (STATE_t){ &BrakeBootEntry,		&EmptyFcn,			    &BrakeBootISR0,		    &BrakeBootISR1  };
    sm.states[Volt_Hz_OL]       = (STATE_t){ &VoltHzOLEntry,		&VoltHzOLExit,	        &VoltHzOLISR0,		    &EmptyFcn       };
    sm.states[Speed_CL]         = (STATE_t){ &SpeedCLEntry,		    &EmptyFcn,			    &SpeedCLISR0,		    &EmptyFcn       };
    sm.states[Fault]            = (STATE_t){ &FaultEntry,			&FaultExit,			    &EmptyFcn,			    &FaultISR1      };
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    sm.states[Align]            = (STATE_t){ &AlignEntry,			&AlignExit,			    &AlignISR0,			    &EmptyFcn       };
    sm.states[Six_Pulse]        = (STATE_t){ &SixPulseEntry,		&SixPulseExit,		    &SixPulseISR0,		    &SixPulseISR1   };
    sm.states[High_Freq]        = (STATE_t){ &HighFreqEntry,		&HighFreqExit,		    &HighFreqISR0,		    &EmptyFcn       };
    sm.states[Speed_OL_To_CL]   = (STATE_t){ &SpeedOLToCLEntry,	    &SpeedOLToCLExit,	    &SpeedOLToCLISR0,	    &EmptyFcn       };
    sm.states[Dyno_Lock]        = (STATE_t){ &DynoLockEntry,		&DynoLockExit,		    &DynoLockISR0,		    &EmptyFcn       };
    sm.states[Prof_Rot_Lock]    = (STATE_t){ &MotorProfEntry,       &MotorProfExit,	        &MotorProfISR0,         &EmptyFcn       };
    sm.states[Prof_R]           = (STATE_t){ &MotorProfEntry,       &MotorProfExit,	        &MotorProfISR0,         &EmptyFcn       };
    sm.states[Prof_Ld]          = (STATE_t){ &MotorProfEntry,       &MotorProfExit,	        &MotorProfISR0,         &EmptyFcn       };
    sm.states[Prof_Lq]          = (STATE_t){ &MotorProfEntry,       &MotorProfExit,	        &MotorProfISR0,         &EmptyFcn       };
    sm.states[Prof_Finished]    = (STATE_t){ &MotorProfEntry,       &MotorProfExit,	        &MotorProfISR0,         &EmptyFcn       };
#endif
#if defined(CTRL_METHOD_SFO)
    sm.states[Torque_CL]        = (STATE_t){ &TorqueCLEntry,		&EmptyFcn,			    &TorqueCLISR0,		    &EmptyFcn       };
#elif defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    sm.states[Current_CL]       = (STATE_t){ &CurrentCLEntry,		&EmptyFcn,			    &CurrentCLISR0,		    &EmptyFcn       };
#endif

    sm.current = Init;
    sm.vars.init.param_init_done = false;
    sm.states[sm.current].Entry();

    sm.vars.fault.clr_try_cnt = 0U;

    // Must not be STATE_MACHINE_ResetAllModules because that is one of the functions requested by gui:
    FCN_EXE_HANDLER_Init();
    FCN_EXE_HANDLER_Reset();

#if defined(PC_TEST)
    for (uint32_t index = 0; index < sizeof(sm.vars.capture_vals) / sizeof(float); ++index)
    {
        sm.vars.capture_vals[index] = 0.0f;
        sm.vars.capture_channels[index] = &sm.vars.capture_vals[index];
    }
#endif
}

RAMFUNC_BEGIN
void STATE_MACHINE_RunISR0()
{
    CommonISR0Wrap();

    sm.states[sm.current].RunISR0();

    sm.add_callback.RunISR0();
}
RAMFUNC_END

void STATE_MACHINE_RunISR1()
{
    CommonISR1Wrap();

    sm.states[sm.current].RunISR1();

    sm.add_callback.RunISR1();

    ConditionCheck();
    if (sm.next != sm.current)
    {
#if defined(PC_TEST)
        for (uint32_t index = 0; index < sizeof(sm.vars.capture_vals) / sizeof(float); ++index)
        {
            sm.vars.capture_vals[index] = (*sm.vars.capture_channels[index]);
        }
#endif
        sm.states[sm.current].Exit();
        sm.states[sm.next].Entry();
        sm.current = sm.next;
        // This instruction (sm.current update) should be the last one ue to data integrity reasons (between ISR0/ISR1)
    }

}