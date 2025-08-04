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



STATE_MACHINE_t sm[MOTOR_CTRL_NO_OF_MOTOR] = { 0 };

static void (*CommonISR0Wrap)() = EmptyFcn;
static void (*CommonISR1Wrap)() = EmptyFcn;
#if defined(PC_TEST)
static void (*FeedbackISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = { OBS_RunISR0 };
#else
static void (*FeedbackISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() ={[0 ... (MOTOR_CTRL_NO_OF_MOTOR-1)] = OBS_RunISR0};
#endif

#if defined(CTRL_METHOD_RFO)||defined(CTRL_METHOD_SFO)||defined(CTRL_METHOD_TBC)
static inline bool Mode(PARAMS_t* params_ptr, CTRL_MODE_t ctrl_mode)
{
    return (params_ptr->ctrl.mode == ctrl_mode);
}
#endif
RAMFUNC_BEGIN
static void CommonISR0(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    SENSOR_IFACE_RunISR0(motor_ptr);
    if(sm_ptr->vars.init.offset_null_done) /*Current calculation  is not executed, if current offset is not done*/
    {
      FAULT_PROTECT_RunISR0(motor_ptr);
    }
}
RAMFUNC_END

static void CommonISR1(MOTOR_t *motor_ptr)
{
    SENSOR_IFACE_RunISR1(motor_ptr);
    FAULT_PROTECT_RunISR1(motor_ptr);

    if(motor_ptr->motor_instance == 0)
    {
      FCN_EXE_HANDLER_RunISR1();
    }
}

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
RAMFUNC_BEGIN
static void SensorlessFeedbackISR0(MOTOR_t *motor_ptr)
{
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

#if defined(CTRL_METHOD_SFO)|| defined(PC_TEST)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
#if defined(PC_TEST)
    OBS_t* obs_ptr = motor_ptr->obs_ptr;
#endif
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
    // Using threshold hysteresis
    if ((sm_ptr->vars.high_freq.used) && ABS_ABOVE_LIM(vars_ptr->w_final_filt.elec, params_ptr->obs.w_thresh.elec))
    {
        sm_ptr->vars.high_freq.used = false;
#if defined(CTRL_METHOD_RFO)
        CURRENT_CTRL_Init(motor_ptr,1.0f);
#elif defined(CTRL_METHOD_SFO)
        FLUX_CTRL_Init(motor_ptr,1.0f);
        ctrl_ptr->delta.bw_red_coeff = 1.0f;
#endif
    }

    if (!sm_ptr->vars.high_freq.used)
    {
        OBS_RunISR0(motor_ptr);
    }
    else // running both observer and high frequency injection
    {
        HIGH_FREQ_INJ_RunFiltISR0(motor_ptr);
        OBS_RunISR0(motor_ptr);
        HIGH_FREQ_INJ_RunCtrlISR0(motor_ptr);
    }

#if defined(PC_TEST)
    vars_ptr->test[34] = sm_ptr->vars.high_freq.used;
    vars_ptr->test[35] = obs_ptr->pll_r.th.elec;
    vars_ptr->test[36] = ctrl_ptr->high_freq_inj.integ_pll_r.integ;
    vars_ptr->test[37] = obs_ptr->pll_r.w.elec;
    vars_ptr->test[38] = ctrl_ptr->high_freq_inj.pi_pll_r.output;
#endif
}
RAMFUNC_END
#endif

void STATE_MACHINE_ResetAllModules(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_RFO)||defined(CTRL_METHOD_SFO)||defined(CTRL_METHOD_TBC)
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
#endif
    SENSOR_IFACE_Init(motor_ptr);
    FAULT_PROTECT_Init(motor_ptr);
    OBS_Init(motor_ptr);
    CTRL_FILTS_Init(motor_ptr);
    SPEED_CTRL_Init(motor_ptr);
    VOLT_MOD_Init(motor_ptr);
#if defined(CTRL_METHOD_RFO)
    PHASE_ADV_Init(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Init(motor_ptr,1.0f);
    TRQ_Init(motor_ptr);
    DELTA_CTRL_Init(motor_ptr);
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    SIX_PULSE_INJ_Init(motor_ptr);
    HIGH_FREQ_INJ_Init(motor_ptr);
    PROFILER_Init(motor_ptr);
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    HALL_SENSOR_Init(motor_ptr);
    INC_ENCODER_Init(motor_ptr);
#endif
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Init(motor_ptr, 1.0f);
#endif
#if defined(CTRL_METHOD_TBC)
    CURRENT_CTRL_Init(motor_ptr);
#endif
    // Reset CommonISR modules first:
    SENSOR_IFACE_Reset(motor_ptr);
    FAULT_PROTECT_Reset(motor_ptr);
    CommonISR0Wrap = CommonISR0;
    CommonISR1Wrap = CommonISR1;
#if defined(CTRL_METHOD_RFO)
    if (Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup) ||
        Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup))
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance] = SensorlessFeedbackISR0;
        sm_ptr->vars.high_freq.used = true;
    }
    else if (params_ptr->sys.fb.mode == Hall)
    {
        HALL_SENSOR_Reset(motor_ptr);
        FeedbackISR0Wrap[motor_ptr->motor_instance] = HALL_SENSOR_RunISR0;
        sm_ptr->vars.high_freq.used = false;
    }
    else if (params_ptr->sys.fb.mode == AqB_Enc)
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance] = INC_ENCODER_RunISR0;
        sm_ptr->vars.high_freq.used = false;
    }
    else if (params_ptr->sys.fb.mode == Direct)
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance] = EmptyFcn_PtrArgument;
        sm_ptr->vars.high_freq.used = false;

    }
    else /*Sensor-less*/
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance] = OBS_RunISR0;
        sm_ptr->vars.high_freq.used = false;

    }
#elif defined(CTRL_METHOD_SFO)
    if (Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup) ||
        Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup))
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance] = SensorlessFeedbackISR0;
        sm_ptr->vars.high_freq.used = true;
    }
    else
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance] = OBS_RunISR0;
        sm_ptr->vars.high_freq.used = false;
    }
#elif defined(CTRL_METHOD_TBC)
    if (params_ptr->sys.fb.mode == Hall)
    {
        HALL_SENSOR_Reset(motor_ptr);
        FeedbackISR0Wrap[motor_ptr->motor_instance] = HALL_SENSOR_RunISR0;
        BLOCK_COMM_Init(motor_ptr);
        TRAP_COMM_Init(motor_ptr);
    }
#endif
}
void STATE_MACHINE_ResetVariable(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
#if defined(CTRL_METHOD_RFO)||defined(CTRL_METHOD_SFO)||defined(CTRL_METHOD_TBC)
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
#endif
    ELEC_t w0 = Elec_Zero;

	CTRL_FILTS_Reset(motor_ptr);
    FAULT_PROTECT_Reset(motor_ptr);
    SPEED_CTRL_Reset(motor_ptr);
    VOLT_CTRL_Reset(motor_ptr);
    CTRL_ResetWcmdInt(motor_ptr,w0);

    vars_ptr->v_qd_r_cmd.d = 0.0f;
    vars_ptr->v_qd_r_cmd.q = 0.0f;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    ELEC_t th0 = { PI_OVER_TWO };
    if (params_ptr->sys.fb.mode == Sensorless)
    {
       AB_t la_ab_lead = (AB_t){ (params_ptr->motor.lam + params_ptr->motor.ld * params_ptr->ctrl.align.voltage / params_ptr->motor.r), 0.0f };
       OBS_Reset(motor_ptr,&la_ab_lead, &w0, &th0);
    }
    SIX_PULSE_INJ_Reset(motor_ptr);
    TRQ_Reset(motor_ptr);
    HIGH_FREQ_INJ_Reset(motor_ptr,Elec_Zero, motor_ptr->ctrl_ptr->six_pulse_inj.th_r_est);
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
	if (params_ptr->sys.fb.mode == Hall)
	{
	  HALL_SENSOR_Reset(motor_ptr);
	}
	CURRENT_CTRL_Reset(motor_ptr);
    vars_ptr->i_cmd_int = 0.0f;

#endif
#if defined(CTRL_METHOD_SFO)
	FLUX_CTRL_Reset(motor_ptr);
	DELTA_CTRL_Reset(motor_ptr);
    vars_ptr->T_cmd_int = 0.0f;
	vars_ptr->delta_cmd.elec =0.0f;
	vars_ptr->T_cmd_final = 0.0f;
	vars_ptr->la_cmd_final =0.0f;
	vars_ptr->la_qd_s_est.d = 0.0f;
	vars_ptr->v_qd_s_cmd.d = 0.0f;
	vars_ptr->v_qd_s_cmd.q = 0.0f;
#endif
#if defined(CTRL_METHOD_RFO)
    if (params_ptr->sys.fb.mode == AqB_Enc)
    {
      INC_ENCODER_Reset(motor_ptr,th0);
    }
	FLUX_WEAKEN_Reset(motor_ptr);
	vars_ptr->i_qd_r_cmd.d = 0.0f;
	vars_ptr->i_qd_r_cmd.q = 0.0f;

	vars_ptr->i_qd_r_fb.d  = 0.0f;
	vars_ptr->i_qd_r_fb.q  = 0.0f;

#endif
#if defined(CTRL_METHOD_TBC)
	TRAP_COMM_Reset(motor_ptr);
	vars_ptr->v_s_cmd.rad= 0.0f;
#endif


}
static void InitEntry(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance);

    if (!sm_ptr->vars.init.param_init_done)	// only after booting up
    {
        hw_fcn.StopPeripherals(motor_ptr->motor_instance);		// disable all ISRs, PWMs, ADCs, etc.

        CommonISR0Wrap = EmptyFcn;
        CommonISR1Wrap = EmptyFcn;
        FeedbackISR0Wrap[motor_ptr->motor_instance] = OBS_RunISR0;
#if !defined(PC_TEST)
        PARAMS_Init(motor_ptr);
#endif
        hw_fcn.HardwareIfaceInit(motor_ptr->motor_instance);		// all peripherals must stop before re-initializing
        STATE_MACHINE_ResetAllModules(motor_ptr);
        sm_ptr->vars.init.param_init_done = true;
        vars_ptr->en = true;

    }
   	STATE_MACHINE_ResetVariable(motor_ptr);

    // TBD: check offset nulling timer
    StopWatchInit(&sm_ptr->vars.init.timer, params_ptr->sys.analog.offset_null_time, params_ptr->sys.samp.ts0);
    sm_ptr->vars.speed_reset_required = false;
    sm_ptr->vars.init.offset_null_done = false;

}

RAMFUNC_BEGIN
static void InitISR0(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    if (sm_ptr->vars.init.param_init_done)
    {
        StopWatchRun(&sm_ptr->vars.init.timer);
        if (StopWatchIsDone(&sm_ptr->vars.init.timer))
        {
            sm_ptr->vars.init.offset_null_done = true;
        }
        else
        {
            SENSOR_IFACE_OffsetNullISR0(motor_ptr);
        }
    }
}
RAMFUNC_END

static void BrakeBootEntry(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    StopWatchInit(&sm_ptr->vars.brake_boot.timer, params_ptr->sys.boot_time, params_ptr->sys.samp.ts0);

    hw_fcn.EnterCriticalSection();	// --------------------
    // atomic operations needed for struct writes and no more modification until next state
    CTRL_FILTS_Reset(motor_ptr);
    vars_ptr->d_uvw_cmd = UVW_Zero;
    vars_ptr->w_cmd_int.elec = 0.0f;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    vars_ptr->i_cmd_int = 0.0f;
#elif defined(CTRL_METHOD_SFO)
    vars_ptr->T_cmd_int = 0.0f;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    if (Mode(params_ptr,Profiler_Mode))
    {
        sm_ptr->add_callback.RunISR0 = EmptyFcn;
    }
#endif
    sm_ptr->current = sm_ptr->next; // must be in critical section for brake-boot entry
    hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
    hw_fcn.ExitCriticalSection();	// --------------------
}

RAMFUNC_BEGIN
static void BrakeBootISR0(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
#endif
	StopWatchRun(&sm_ptr->vars.brake_boot.timer);
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    if (params_ptr->sys.fb.mode == Hall)
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
        vars_ptr->w_final.elec = vars_ptr->w_hall.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
    }
#endif
}
RAMFUNC_END

static void BrakeBootISR1(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    if (sm_ptr->vars.speed_reset_required && ABS_BELOW_LIM(vars_ptr->w_cmd_ext.elec, params_ptr->ctrl.volt.w_thresh.elec - params_ptr->ctrl.volt.w_hyst.elec))
    {
        sm_ptr->vars.speed_reset_required = false;
    }
}

static void VoltOLEntry(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    CTRL_ResetWcmdInt(motor_ptr,params_ptr->ctrl.volt.w_thresh);
    vars_ptr->v_qd_r_cmd.d = 0.0f;
    VOLT_CTRL_Reset(motor_ptr);
    hw_fcn.EnterCriticalSection();	// --------------------
    if (params_ptr->sys.analog.shunt.type == Single_Shunt)
    {
        VOLT_MOD_EnDisHybMod(motor_ptr,Dis);
    }
#if defined(CTRL_METHOD_SFO)
    if (Mode(params_ptr,Profiler_Mode))
    {
        sm_ptr->add_callback.RunISR0 = PROFILER_RunISR0;
    }
#endif
    sm_ptr->current = sm_ptr->next; // must be in critical section
    hw_fcn.ExitCriticalSection();	// --------------------
}

static void VoltOLExit(MOTOR_t *motor_ptr)
{
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    if (params_ptr->sys.analog.shunt.type == Single_Shunt)
    {
        VOLT_MOD_EnDisHybMod(motor_ptr,En);
    }
}

RAMFUNC_BEGIN
static void VoltOLISR0(MOTOR_t *motor_ptr)
{

    VOLT_CTRL_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END
static void VoltOLISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;

    CTRL_UpdateWcmdIntISR1(motor_ptr,vars_ptr->w_cmd_ext);

    vars_ptr->v_qd_r_cmd.q = MAX(params_ptr->ctrl.volt.v_min + ABS(vars_ptr->w_cmd_int.elec) * params_ptr->ctrl.volt.v_to_f_ratio, 0.0f) * vars_ptr->dir;
    CTRL_FILTS_RunAllISR1(motor_ptr);
}

static void SpeedCLEntry(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_RFO)||defined(CTRL_METHOD_SFO)||defined(CTRL_METHOD_TBC)
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
#endif


#if defined(CTRL_METHOD_RFO)
    if ((sm_ptr->current == Align) || (sm_ptr->current == Six_Pulse) || (sm_ptr->current == High_Freq) || (sm_ptr->current == Brake_Boot))
    {
        ELEC_t w0 = (((sm_ptr->current == Align) || (sm_ptr->current == Six_Pulse)) && (params_ptr->sys.fb.mode == Sensorless)) ? params_ptr->obs.w_thresh : params_ptr->ctrl.volt.w_thresh;
        CTRL_ResetWcmdInt(motor_ptr,w0);
        SPEED_CTRL_Reset(motor_ptr);
        vars_ptr->i_cmd_int = 0.0f;
        FLUX_WEAKEN_Reset(motor_ptr);
    }
#elif defined(CTRL_METHOD_SFO)
    if ((sm_ptr->current == Align) || (sm_ptr->current == Six_Pulse) || (sm_ptr->current == High_Freq))
    {
        CTRL_ResetWcmdInt(motor_ptr,(sm_ptr->current == High_Freq) ? params_ptr->ctrl.volt.w_thresh : params_ptr->obs.w_thresh);
        SPEED_CTRL_Reset(motor_ptr);
        vars_ptr->T_cmd_int = 0.0f;
    }
#elif defined(CTRL_METHOD_TBC)
    if (sm_ptr->current == Brake_Boot)
    {
        CTRL_ResetWcmdInt(motor_ptr,params_ptr->ctrl.volt.w_thresh);
        SPEED_CTRL_Reset(motor_ptr);
        vars_ptr->i_cmd_int = 0.0f;
        CURRENT_CTRL_Reset(motor_ptr);
        TRAP_COMM_Reset(motor_ptr);
    }
#endif
}

#if defined(CTRL_METHOD_SFO)
RAMFUNC_BEGIN
static inline void AddHighFreqVoltsIfNeeded(MOTOR_t *motor_ptr)
{	// adding high frequency excitation voltage components if needed
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
	if (sm_ptr->vars.high_freq.used)
    {
        vars_ptr->v_ab_cmd_tot.alpha += ctrl_ptr->high_freq_inj.v_ab_cmd.alpha;
        vars_ptr->v_ab_cmd_tot.beta += ctrl_ptr->high_freq_inj.v_ab_cmd.beta;
    }
}
RAMFUNC_END
#endif

RAMFUNC_BEGIN
static void SpeedCLISR0(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    switch (params_ptr->sys.fb.mode)
    {
    default:
    case Sensorless:
    	vars_ptr->w_final.elec = vars_ptr->w_est.elec;
    	vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
        break;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    case Hall:
    	vars_ptr->w_final.elec =vars_ptr->w_hall.elec;
    	vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
        break;
    case AqB_Enc:
    	vars_ptr->w_final.elec = vars_ptr->w_enc.elec;
    	vars_ptr->th_r_final.elec = vars_ptr->th_r_enc.elec;
        break;
    case Direct:
    	vars_ptr->w_final.elec = vars_ptr->w_fb.elec;
    	vars_ptr->th_r_final.elec = vars_ptr->th_r_fb.elec;
        break;
#endif
    }
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_RunISR0(motor_ptr);
    TRQ_RunObsISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_RunISR0(motor_ptr);
    TRQ_RunObsISR0(motor_ptr);
    TRQ_RunCtrlISR0(motor_ptr);
    DELTA_CTRL_RunISR0(motor_ptr);
    ParkTransformInv(&vars_ptr->v_qd_s_cmd, &vars_ptr->park_s, &vars_ptr->v_ab_cmd);
    vars_ptr->v_ab_cmd_tot = vars_ptr->v_ab_cmd;
    AddHighFreqVoltsIfNeeded(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);

#elif defined(CTRL_METHOD_TBC)
    switch (params_ptr->ctrl.tbc.mode)
    {
    case Block_Commutation:
    default:
        CURRENT_CTRL_RunISR0(motor_ptr);
        BLOCK_COMM_RunVoltModISR0(motor_ptr);
        break;
    case Trapezoidal_Commutation:
        TRAP_COMM_RunISR0(motor_ptr);
        break;
    }
    TRQ_RunObsISR0(motor_ptr);
#endif

}
RAMFUNC_END
static void SpeedCLISR1(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
#if defined(CTRL_METHOD_RFO)||defined(CTRL_METHOD_SFO)||defined(CTRL_METHOD_TBC)
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;
#endif

    CTRL_UpdateWcmdIntISR1(motor_ptr,vars_ptr->w_cmd_ext);
    CTRL_FILTS_RunAllISR1(motor_ptr);
    SPEED_CTRL_RunISR1(motor_ptr);
#if defined(CTRL_METHOD_RFO)
    vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_spd);
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
    PHASE_ADV_RunISR1(motor_ptr);
    FLUX_WEAKEN_RunISR1(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FAULT_PROTECT_RunTrqLimitCtrlISR1(motor_ptr);
    vars_ptr->T_cmd_prot = SAT(-protect_ptr->motor.T_lmt, protect_ptr->motor.T_lmt, vars_ptr->T_cmd_spd);
    vars_ptr->T_cmd_int = RateLimit(params_ptr->sys.rate_lim.T_cmd * params_ptr->sys.samp.ts1, vars_ptr->T_cmd_prot, vars_ptr->T_cmd_int);
    vars_ptr->la_cmd_mtpa = LUT1DInterp(&params_ptr->motor.mtpa_lut, ABS(vars_ptr->T_cmd_int));
    FLUX_WEAKEN_RunISR1(motor_ptr);
#elif defined(CTRL_METHOD_TBC)
    vars_ptr->i_cmd_prot = (params_ptr->ctrl.curr.bypass == false) ? SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_spd) : vars_ptr->i_cmd_spd;
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
#endif

}

static void FaultEntry(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	FAULTS_t* faults_ptr = motor_ptr->faults_ptr;

#if defined(CTRL_METHOD_TBC)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
    UVW_t d_uvw_cmd = UVW_Zero;
    if (faults_ptr->reaction == Short_Motor)
    {
        hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
#if defined(CTRL_METHOD_TBC)
        ctrl_ptr->block_comm.exit_high_z_flag.u = true;
        ctrl_ptr->block_comm.exit_high_z_flag.v = true;
        ctrl_ptr->block_comm.exit_high_z_flag.w = true;
#endif
        switch (params_ptr->sys.faults.short_method)
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
    else if (faults_ptr->reaction == High_Z)
    {
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance);
#if defined(CTRL_METHOD_TBC)
        ctrl_ptr->block_comm.enter_high_z_flag.w = true;
        ctrl_ptr->block_comm.enter_high_z_flag.v = true;
        ctrl_ptr->block_comm.enter_high_z_flag.w = true;
#endif
    } // "No_Reaction" is not reachable

    sm_ptr->vars.fault.clr_success = false;
    sm_ptr->vars.fault.clr_request = false;

    hw_fcn.EnterCriticalSection();	// --------------------
    CTRL_FILTS_Reset(motor_ptr);
    vars_ptr->d_uvw_cmd = d_uvw_cmd;
    sm_ptr->current = sm_ptr->next;			// must be in critical section
    hw_fcn.ExitCriticalSection();	// --------------------
}

static void FaultISR1(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	FAULTS_t* faults_ptr = motor_ptr->faults_ptr;

    bool clr_faults_and_count = RISE_EDGE(sm_ptr->vars.fault.clr_faults_prev, vars_ptr->clr_faults) && (!faults_ptr->flags_latched.sw.brk) && (!faults_ptr->flags_latched.sw.em_stop);

    /* Move SM from Fault to init state, while remove the emergency stop or brake*/
    bool clr_faults_no_count =  ((!faults_ptr->flags.sw.brk) && (faults_ptr->flags_latched.sw.brk)) || ((!faults_ptr->flags.sw.em_stop) && (faults_ptr->flags_latched.sw.em_stop));

    sm_ptr->vars.fault.clr_faults_prev = vars_ptr->clr_faults;

    if (sm_ptr->vars.fault.clr_request)
    {
        sm_ptr->vars.fault.clr_request = false;
        FAULT_PROTECT_ClearFaults(motor_ptr);
        sm_ptr->vars.fault.clr_success = true;
    }
    else if (clr_faults_and_count && ((sm_ptr->vars.fault.clr_try_cnt < params_ptr->sys.faults.max_clr_tries) || (params_ptr->sys.faults.max_clr_tries == 0xFFFFFFFF)))
    {
        sm_ptr->vars.fault.clr_request = true;
        ++sm_ptr->vars.fault.clr_try_cnt;
    }
    else if (clr_faults_no_count)
    {
        sm_ptr->vars.fault.clr_request = true;
    }

}

static void FaultExit(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_TBC)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
#endif

    hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
#if defined(CTRL_METHOD_TBC)
    ctrl_ptr->block_comm.exit_high_z_flag.u = true;
    ctrl_ptr->block_comm.exit_high_z_flag.v = true;
    ctrl_ptr->block_comm.exit_high_z_flag.w = true;
#endif
}

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

static void AlignEntry(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    VOLT_CTRL_Reset(motor_ptr);
    vars_ptr->v_qd_r_cmd = (QD_t){ params_ptr->ctrl.align.voltage, 0.0f };
    StopWatchInit(&sm_ptr->vars.align.timer, params_ptr->ctrl.align.time, params_ptr->sys.samp.ts0);
}

RAMFUNC_BEGIN
static void AlignISR0(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    VOLT_CTRL_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
    StopWatchRun(&sm_ptr->vars.align.timer);
}
RAMFUNC_END

static void AlignExit(MOTOR_t *motor_ptr)
{

	PARAMS_t* params_ptr = motor_ptr->params_ptr;
    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = { PI_OVER_TWO };

#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Reset(motor_ptr);
    if (params_ptr->sys.fb.mode == AqB_Enc)
    {
        INC_ENCODER_Reset(motor_ptr,th0);
    }
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Reset(motor_ptr);
    TRQ_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);
#endif
    if (params_ptr->sys.fb.mode == Sensorless)
    {
        AB_t la_ab_lead = (AB_t){ (params_ptr->motor.lam + params_ptr->motor.ld * params_ptr->ctrl.align.voltage / params_ptr->motor.r), 0.0f };
        OBS_Reset(motor_ptr,&la_ab_lead, &w0, &th0);
    }
}

static void SixPulseEntry(MOTOR_t *motor_ptr)
{
    SIX_PULSE_INJ_Reset(motor_ptr);
}

RAMFUNC_BEGIN
static void SixPulseISR0(MOTOR_t *motor_ptr)
{
    SIX_PULSE_INJ_RunISR0(motor_ptr);
}
RAMFUNC_END

static void SixPulseISR1(MOTOR_t *motor_ptr)
{
    SIX_PULSE_INJ_RunISR1(motor_ptr);
}

static void SixPulseExit(MOTOR_t *motor_ptr)
{
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;

#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Reset(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Reset(motor_ptr);
    TRQ_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);
#endif

    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = ctrl_ptr->six_pulse_inj.th_r_est;
    PARK_t park_r;
    ParkInit(th0.elec, &park_r);

    QD_t la_qd_r = (QD_t){ 0.0f, params_ptr->motor.lam };
    AB_t la_ab_lead;
    ParkTransformInv(&la_qd_r, &park_r, &la_ab_lead);

    OBS_Reset(motor_ptr,&la_ab_lead, &w0, &th0);
}

static void HighFreqEntry(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;

    HIGH_FREQ_INJ_Reset(motor_ptr,Elec_Zero, ctrl_ptr->six_pulse_inj.th_r_est);
    StopWatchInit(&sm_ptr->vars.high_freq.timer, params_ptr->ctrl.high_freq_inj.lock_time, params_ptr->sys.samp.ts1);
    sm_ptr->vars.high_freq.used = true;
    vars_ptr->v_ab_cmd = AB_Zero;
}

RAMFUNC_BEGIN
static void HighFreqISR0(MOTOR_t *motor_ptr)
{

	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    vars_ptr->w_final.elec = vars_ptr->w_est.elec;
    vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
    vars_ptr->v_ab_cmd_tot = ctrl_ptr->high_freq_inj.v_ab_cmd;
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END

static void HighFreqISR1(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    StopWatchRun(&sm_ptr->vars.high_freq.timer);
    CTRL_FILTS_RunSpeedISR1(motor_ptr);
}
static void HighFreqExit(MOTOR_t *motor_ptr)
{
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
#if defined(CTRL_METHOD_SFO)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Init(motor_ptr, params_ptr->ctrl.high_freq_inj.bw_red_coeff);
    CURRENT_CTRL_Reset(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Init(motor_ptr, params_ptr->ctrl.high_freq_inj.bw_red_coeff);
    ctrl_ptr->delta.bw_red_coeff = params_ptr->ctrl.high_freq_inj.bw_red_coeff;
#endif
}

static void SpeedOLToCLEntry(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    AB_t la_ab_lead = AB_Zero;
    ELEC_t w0 = (ELEC_t){ params_ptr->obs.w_thresh.elec * vars_ptr->dir };
    ELEC_t th0 = Elec_Zero;
    OBS_Reset(motor_ptr,&la_ab_lead, &w0, &th0);
    TRQ_Reset(motor_ptr);
    StopWatchInit(&sm_ptr->vars.speed_ol_to_cl.timer, params_ptr->obs.lock_time, params_ptr->sys.samp.ts1);
}

RAMFUNC_BEGIN
static void SpeedOLToCLISR0(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    #if defined(CTRL_METHOD_RFO)
    bool prev_state_volt_ol = Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup);
    bool prev_state_curr_ol = Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr,Profiler_Mode);
#elif defined(CTRL_METHOD_SFO)
    bool prev_state_volt_ol = Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr,Profiler_Mode);
    // bool prev_state_curr_ol = false;
#endif

    if (prev_state_volt_ol)
    {
ELEC_t th_r_trq = { vars_ptr->th_r_est.elec };
    PARK_t park_r_trq;
    ParkInit(th_r_trq.elec, &park_r_trq);
    ParkTransform(&vars_ptr->i_ab_fb, &park_r_trq, &vars_ptr->i_qd_r_fb);

    VOLT_CTRL_RunISR0(motor_ptr);
    }
#if defined(CTRL_METHOD_RFO)
    else if (prev_state_curr_ol)
    {
    	vars_ptr->th_r_cmd.elec = Wrap2Pi(vars_ptr->th_r_cmd.elec + vars_ptr->w_cmd_int.elec * params_ptr->sys.samp.ts0);
        vars_ptr->th_r_final.elec = vars_ptr->th_r_cmd.elec;
        vars_ptr->w_final.elec = vars_ptr->w_cmd_int.elec;


        CURRENT_CTRL_RunISR0(motor_ptr);
    }
#endif
    TRQ_RunObsISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);

}
RAMFUNC_END
static void SpeedOLToCLISR1(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_RFO)
	PROTECT_t* protect_ptr = motor_ptr->protect_ptr;
#endif
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    CTRL_UpdateWcmdIntISR1(motor_ptr,vars_ptr->w_cmd_ext);

#if defined(CTRL_METHOD_RFO)
    bool prev_state_volt_ol = Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup);
    bool prev_state_curr_ol = Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr,Profiler_Mode);
#elif defined(CTRL_METHOD_SFO)
    bool prev_state_volt_ol = Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr, Profiler_Mode);
    // bool prev_state_curr_ol = false;
#endif

    if (prev_state_volt_ol)
    {
    vars_ptr->v_qd_r_cmd.q = MAX(params_ptr->ctrl.volt.v_min + ABS(vars_ptr->w_cmd_int.elec) * params_ptr->ctrl.volt.v_to_f_ratio, 0.0f) * vars_ptr->dir;
    }
#if defined(CTRL_METHOD_RFO)
    else if (prev_state_curr_ol)
    {
        vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_ext);
        vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
        vars_ptr->i_qd_r_ref = (QD_t){ vars_ptr->i_cmd_int, 0.0f };   // No phase advance in open loop
        vars_ptr->i_qd_r_cmd = vars_ptr->i_qd_r_ref;                  // No flux weakening in current control mode

    }
#endif
    CTRL_FILTS_RunAllISR1(motor_ptr);
    StopWatchRun(&sm_ptr->vars.speed_ol_to_cl.timer);
}

static void SpeedOLToCLExit(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
#if defined(CTRL_METHOD_RFO)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
    SPEED_CTRL_Reset(motor_ptr);

    PARK_t park_r;
    QD_t i_qd_r_fb;

    hw_fcn.EnterCriticalSection();	// --------------------
    // atomic operations needed for struct copies and all data must have the same time stamp
    AB_t i_ab_fb = vars_ptr->i_ab_fb;
    float th_r_est = vars_ptr->th_r_est.elec;
    hw_fcn.ExitCriticalSection();	// --------------------

    ParkInit(th_r_est, &park_r);
    ParkTransform(&i_ab_fb, &park_r, &i_qd_r_fb);

#if defined(CTRL_METHOD_RFO)
    float i_cmd_spd;
    PHASE_ADV_CalcOptIs(motor_ptr,&i_qd_r_fb, &i_cmd_spd);
    vars_ptr->i_cmd_int = i_cmd_spd * params_ptr->ctrl.speed.ol_cl_tr_coeff;
    SPEED_CTRL_IntegBackCalc(motor_ptr,vars_ptr->i_cmd_int);

#if defined(PC_TEST)
    vars_ptr->test[30] = i_qd_r_fb.q;
    vars_ptr->test[31] = i_qd_r_fb.d;
    vars_ptr->test[32] = i_cmd_spd;
#endif

    FLUX_WEAKEN_Reset(motor_ptr);
    if (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup))
    {
    CURRENT_CTRL_Reset(motor_ptr);
    }
    else if (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr,Profiler_Mode))
    {
        ctrl_ptr->curr.en_ff = true;
    }

#elif defined(CTRL_METHOD_SFO)
    float T_cmd_spd;
    TRQ_CalcTrq(motor_ptr,&i_qd_r_fb, &T_cmd_spd);
    vars_ptr->T_cmd_int = T_cmd_spd * params_ptr->ctrl.speed.ol_cl_tr_coeff;
    SPEED_CTRL_IntegBackCalc(motor_ptr,vars_ptr->T_cmd_int);

    FLUX_CTRL_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);

#endif
}

static void DynoLockEntry(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    StopWatchInit(&sm_ptr->vars.dyno_lock.timer, params_ptr->sys.dyno_lock_time, params_ptr->sys.samp.ts1);

    hw_fcn.EnterCriticalSection();	// --------------------
    // atomic operations needed for struct writes and no more modification until next state

    hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance);
    vars_ptr->d_uvw_cmd = UVW_Zero;

    vars_ptr->v_ab_obs = &vars_ptr->v_ab_fb;
    AB_t la_ab_lead = AB_Zero;
    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = Elec_Zero;
    OBS_Reset(motor_ptr,&la_ab_lead, &w0, &th0);
    CTRL_FILTS_Reset(motor_ptr);

    sm_ptr->current = sm_ptr->next; // must be in critical section for dyno_lock entry

    hw_fcn.ExitCriticalSection();	// --------------------
}

RAMFUNC_BEGIN
static void DynoLockISR0(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);

    switch (params_ptr->sys.fb.mode)
    {
    default:
    case Sensorless:
    	vars_ptr->w_final.elec = vars_ptr->w_est.elec;
    	vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
        break;
#if defined(CTRL_METHOD_RFO)
    case Hall:
    	vars_ptr->w_final.elec = vars_ptr->w_hall.elec;
    	vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
        break;
#endif
    }
}
RAMFUNC_END
static void DynoLockISR1(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    StopWatchRun(&sm_ptr->vars.dyno_lock.timer);
    CTRL_FILTS_RunSpeedISR1(motor_ptr);
}

static void DynoLockExit(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
#if defined(CTRL_METHOD_SFO)
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
#endif
#if defined(CTRL_METHOD_SFO)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
#endif

    TRQ_Reset(motor_ptr);
#if defined(CTRL_METHOD_RFO)
    //	Ideal case in RFO:
    //		1) i_qd_r_cmd == i_qd_r_fb == {0.0f,0.0f}
    //		2) v_qd_r_cmd == v_qd_r_fb == {lam*w, 0.0f}
    //		3) v_ab_cmd == v_ab_fb == ParkInv(v_qd_r_fb, th_r)
    //		4) curr.pi_q.error == curr.pi_d.error == curr.pi_q.output == curr.pi_d.output == 0.0f (when curr.ff_coef == 1.0f)
    vars_ptr->i_cmd_int = 0.0f;
    CURRENT_CTRL_Reset(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    //	Ideal case in SFO:
    //		1) i_qd_s_cmd == i_qd_s_fb == {0.0f,0.0f}
    //		2) delta == 0.0f, th_s = th_r
    //		3) v_qd_s_cmd == v_qd_s_fb == {lam*w, 0.0f}
    //		4) v_ab_cmd == v_ab_fb == ParkInv(v_qd_s_fb, th_s)
    //		5) trq.pi.error == trq.pi.output == flux.pi.error == flux.pi.output == 0.0f
    //		6) delta.pi.output = v_qd_s_cmd.q = lam*w
    vars_ptr->T_cmd_int = 0.0f;
    FLUX_CTRL_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);
    ctrl_ptr->delta.pi.output = params_ptr->motor.lam * vars_ptr->w_final_filt.elec;
    PI_IntegBackCalc(&ctrl_ptr->delta.pi, ctrl_ptr->delta.pi.output, 0.0f, 0.0f);
#endif
    hw_fcn.EnterCriticalSection();		// --------------------
    vars_ptr->v_ab_cmd = vars_ptr->v_ab_fb;	// atomic operations needed
    vars_ptr->v_ab_obs = &vars_ptr->v_ab_cmd;
    hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
    hw_fcn.ExitCriticalSection();		// --------------------
}

static void MotorProfEntry(MOTOR_t *motor_ptr)
{
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    hw_fcn.EnterCriticalSection();		// --------------------
    PROFILER_Entry(motor_ptr);
    sm_ptr->current = sm_ptr->next; // must be in critical section
    hw_fcn.ExitCriticalSection();		// --------------------
}

static void MotorProfISR0(MOTOR_t *motor_ptr)
{
    PROFILER_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}

static void MotorProfExit(MOTOR_t *motor_ptr)
{
  PROFILER_Exit(motor_ptr);
}

#endif

#if defined(CTRL_METHOD_SFO)

static void TorqueCLEntry(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    vars_ptr->T_cmd_int = params_ptr->ctrl.trq.T_cmd_thresh * vars_ptr->dir;
}

RAMFUNC_BEGIN
static void TorqueCLISR0(MOTOR_t *motor_ptr)
{
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    vars_ptr->w_final.elec = vars_ptr->w_est.elec;
    vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
    FLUX_CTRL_RunISR0(motor_ptr);
    TRQ_RunObsISR0(motor_ptr);
    TRQ_RunCtrlISR0(motor_ptr);
    DELTA_CTRL_RunISR0(motor_ptr);
    ParkTransformInv(&vars_ptr->v_qd_s_cmd, &vars_ptr->park_s, &vars_ptr->v_ab_cmd);
    vars_ptr->v_ab_cmd_tot = vars_ptr->v_ab_cmd;
    AddHighFreqVoltsIfNeeded(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END

static void TorqueCLISR1(MOTOR_t* motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;

    CTRL_FILTS_RunSpeedISR1(motor_ptr);
    FAULT_PROTECT_RunTrqLimitCtrlISR1(motor_ptr);
    vars_ptr->T_cmd_prot = SAT(-protect_ptr->motor.T_lmt, protect_ptr->motor.T_lmt, vars_ptr->T_cmd_ext);
    vars_ptr->T_cmd_int = RateLimit(params_ptr->sys.rate_lim.T_cmd * params_ptr->sys.samp.ts1, vars_ptr->T_cmd_prot, vars_ptr->T_cmd_int);
    vars_ptr->la_cmd_mtpa = LUT1DInterp(&params_ptr->motor.mtpa_lut, ABS(vars_ptr->T_cmd_int));
    vars_ptr->la_cmd_final = vars_ptr->la_cmd_mtpa;		// No flux weakening in torque control mode
}
#elif defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)

static void CurrentCLEntry(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;

    vars_ptr->i_cmd_int = params_ptr->ctrl.curr.i_cmd_thresh * vars_ptr->dir;
}

RAMFUNC_BEGIN
static void CurrentCLISR0(MOTOR_t *motor_ptr)
{
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    switch (params_ptr->sys.fb.mode)
    {
    default:
    case Sensorless:
        vars_ptr->w_final.elec = vars_ptr->w_est.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
        break;
    case Hall:
        vars_ptr->w_final.elec = vars_ptr->w_hall.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
        break;
    case AqB_Enc:
        vars_ptr->w_final.elec = vars_ptr->w_enc.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_enc.elec;
        break;
    case Direct:
    	vars_ptr->w_final.elec = vars_ptr->w_fb.elec;
    	vars_ptr->th_r_final.elec = vars_ptr->th_r_fb.elec;
        break;
    }
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
#elif defined(CTRL_METHOD_TBC)
    switch (params_ptr->ctrl.tbc.mode)
    {
    case Block_Commutation:
    default:
        CURRENT_CTRL_RunISR0(motor_ptr);
        BLOCK_COMM_RunVoltModISR0(motor_ptr);
        break;
    case Trapezoidal_Commutation:
        TRAP_COMM_RunISR0(motor_ptr);
        break;
    }
#endif
    TRQ_RunObsISR0(motor_ptr);
}
RAMFUNC_END

static void CurrentCLISR1(MOTOR_t *motor_ptr)
{
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;

    CTRL_FILTS_RunSpeedISR1(motor_ptr);
    vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_ext);
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
#if defined(CTRL_METHOD_RFO)
    PHASE_ADV_RunISR1(motor_ptr);
    vars_ptr->i_qd_r_cmd = vars_ptr->i_qd_r_ref;		// No flux weakening in current control mode
#endif
}
#endif

#if defined(CTRL_METHOD_RFO)

static void CurrentOLEntry(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
    STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;

    CTRL_ResetWcmdInt(motor_ptr,params_ptr->ctrl.volt.w_thresh);
    vars_ptr->th_r_cmd.elec = 0.0f;

    vars_ptr->i_cmd_ext = params_ptr->ctrl.curr.i_cmd_ol;
    vars_ptr->i_cmd_int = 0.0f;
    CURRENT_CTRL_Reset(motor_ptr);
    ctrl_ptr->curr.en_ff = false;
    hw_fcn.EnterCriticalSection();	// --------------------
    if (Mode(params_ptr,Profiler_Mode))
    {
        sm_ptr->add_callback.RunISR0 = PROFILER_RunISR0;
    }
    sm_ptr->current = sm_ptr->next; // must be in critical section
    hw_fcn.ExitCriticalSection();// --------------------
}

static void CurrentOLExit(MOTOR_t *motor_ptr)
{
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;

    if (Mode(params_ptr,Curr_Mode_Open_Loop))
    {
        ctrl_ptr->curr.en_ff = true;
    }
}

RAMFUNC_BEGIN
static void CurrentOLISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;


    vars_ptr->th_r_cmd.elec = Wrap2Pi(vars_ptr->th_r_cmd.elec + vars_ptr->w_cmd_int.elec * params_ptr->sys.samp.ts0);
    vars_ptr->th_r_final.elec = vars_ptr->th_r_cmd.elec;
    vars_ptr->w_final.elec = vars_ptr->w_cmd_int.elec;


    CURRENT_CTRL_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END

static void CurrentOLISR1(MOTOR_t *motor_ptr)
{
    PROTECT_t* protect_ptr = motor_ptr->protect_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;

    CTRL_UpdateWcmdIntISR1(motor_ptr, vars_ptr->w_cmd_ext);
    CTRL_FILTS_RunSpeedISR1(motor_ptr);
    vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_ext);
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
    vars_ptr->i_qd_r_ref = (QD_t){ vars_ptr->i_cmd_int, 0.0f };   // No phase advance in open loop
    vars_ptr->i_qd_r_cmd = vars_ptr->i_qd_r_ref;                  // No flux weakening in current control mode
}
#endif
static void ConditionCheck(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    FAULTS_t* faults_ptr = motor_ptr->faults_ptr;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    PROFILER_t* profiler_ptr = motor_ptr->profiler_ptr;
    CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
    STATE_ID_t current = sm_ptr->current;
    STATE_ID_t next = current;


    bool fault_trigger = (faults_ptr->reaction != No_Reaction);
#if defined(CTRL_METHOD_RFO)||defined(CTRL_METHOD_SFO)||defined(CTRL_METHOD_TBC)
    bool no_speed_reset_required = !sm_ptr->vars.speed_reset_required;
#endif
    float w_cmd_ext_abs = ABS(vars_ptr->w_cmd_ext.elec);
    float w_cmd_int_abs = ABS(vars_ptr->w_cmd_int.elec);
    float w_thresh_above_low = params_ptr->ctrl.volt.w_thresh.elec;
    float w_thresh_below_low = params_ptr->ctrl.volt.w_thresh.elec - params_ptr->ctrl.volt.w_hyst.elec;
    float w_thresh_above_high = params_ptr->obs.w_thresh.elec;
    float w_thresh_below_high = params_ptr->obs.w_thresh.elec - params_ptr->obs.w_hyst.elec;

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
    float i_cmd_ext_abs = ABS(vars_ptr->i_cmd_ext);
    float i_cmd_int_abs = ABS(vars_ptr->i_cmd_int);
    float i_thresh_above = params_ptr->ctrl.curr.i_cmd_thresh;
    float i_thresh_below = params_ptr->ctrl.curr.i_cmd_thresh - params_ptr->ctrl.curr.i_cmd_hyst;

    bool i_cmd_ext_above_thresh = i_cmd_ext_abs > i_thresh_above;
    bool i_cmd_ext_below_thresh = i_cmd_ext_abs < i_thresh_below;
    bool i_cmd_int_above_thresh = i_cmd_int_abs > i_thresh_above;
    bool i_cmd_int_below_thresh = i_cmd_int_abs < i_thresh_below;

    (void)i_cmd_ext_above_thresh;
    (void)i_cmd_ext_below_thresh;
    (void)i_cmd_int_above_thresh;
    (void)i_cmd_int_below_thresh;

#elif defined(CTRL_METHOD_SFO)
    float T_cmd_ext_abs = ABS(vars_ptr->T_cmd_ext);
    float T_cmd_int_abs = ABS(vars_ptr->T_cmd_int);
    float T_thresh_above = params_ptr->ctrl.trq.T_cmd_thresh;
    float T_thresh_below = params_ptr->ctrl.trq.T_cmd_thresh - params_ptr->ctrl.trq.T_cmd_hyst;

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
    float cmd_thresh_above = params_ptr->profiler.cmd_thresh;
    float cmd_thresh_below = params_ptr->profiler.cmd_thresh - params_ptr->profiler.cmd_hyst;

    bool cmd_above_thresh = vars_ptr->cmd_final > cmd_thresh_above;
    bool cmd_below_thresh = vars_ptr->cmd_final < cmd_thresh_below;

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
        else if (sm_ptr->vars.init.param_init_done && sm_ptr->vars.init.offset_null_done && vars_ptr->en)
        {
#if defined(CTRL_METHOD_RFO)
            if (Mode(params_ptr,Curr_Mode_FOC_Sensorless_Dyno))
            {
                next = Dyno_Lock;
            }
            else
            {
                next = Brake_Boot;
            }
#elif defined(CTRL_METHOD_SFO)
            if (Mode(params_ptr,Trq_Mode_FOC_Sensorless_Dyno))
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
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.brake_boot.timer))
        {
#if defined(CTRL_METHOD_RFO)
            if (cmd_above_thresh && Mode(params_ptr,Profiler_Mode))
            {
                next = Prof_Rot_Lock;
            }
            else if (w_cmd_ext_above_thresh_low && (Mode(params_ptr,Volt_Mode_Open_Loop) || (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup) && no_speed_reset_required)))
            {
                next = Volt_OL;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Encoder_Align_Startup))) ||
                (i_cmd_ext_above_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Curr_Mode_FOC_Encoder_Align_Startup))))
            {
                next = Align;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (i_cmd_ext_above_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Six_Pulse;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr,Curr_Mode_FOC_Hall))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(params_ptr,Speed_Mode_FOC_Hall))
            {
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && (Mode(params_ptr,Curr_Mode_Open_Loop) || (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup) && no_speed_reset_required)))
            {
                next = Current_OL;
            }
#elif defined(CTRL_METHOD_SFO)
            if (cmd_above_thresh && Mode(params_ptr,Profiler_Mode))
            {
                next = Prof_Rot_Lock;
            }
            else if (w_cmd_ext_above_thresh_low && (Mode(params_ptr,Volt_Mode_Open_Loop) || (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup) && no_speed_reset_required)))
            {
                next = Volt_OL;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup)) || (T_cmd_ext_above_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_Align_Startup)))
            {
                next = Align;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (T_cmd_ext_above_thresh && (Mode(params_ptr,Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Six_Pulse;
            }
#elif defined(CTRL_METHOD_TBC)
            if (w_cmd_ext_above_thresh_low && Mode(params_ptr,Volt_Mode_Open_Loop))
            {
                next = Volt_OL;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr,Curr_Mode_Block_Comm_Hall))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(params_ptr,Speed_Mode_Block_Comm_Hall))
            {
                next = Speed_CL;
            }
#endif
        }
        break;
    case Volt_OL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
#if defined(CTRL_METHOD_RFO)
        else if (w_cmd_int_below_thresh_low && (Mode(params_ptr,Volt_Mode_Open_Loop) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup)))
        {
            next = Brake_Boot;
        }
        else if (w_cmd_int_above_thresh_high && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup))
        {
            sm_ptr->vars.speed_reset_required = true;
            next = Speed_OL_To_CL;
        }
#elif defined(CTRL_METHOD_SFO)
        else if ((w_cmd_int_below_thresh_low && (Mode(params_ptr,Volt_Mode_Open_Loop) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup)))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (w_cmd_int_above_thresh_high && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr,Profiler_Mode)))
        {
            sm_ptr->vars.speed_reset_required = true;
            next = Speed_OL_To_CL;
        }
#elif defined(CTRL_METHOD_TBC)
        else if (w_cmd_int_below_thresh_low && Mode(params_ptr,Volt_Mode_Open_Loop))
        {
            next = Brake_Boot;
        }
#endif
        break;

#if defined(CTRL_METHOD_RFO)
    case Current_OL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((w_cmd_int_below_thresh_low && (Mode(params_ptr,Curr_Mode_Open_Loop) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup)))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (w_cmd_int_above_thresh_high && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr,Profiler_Mode)))
        {
            sm_ptr->vars.speed_reset_required = true;
            next = Speed_OL_To_CL;
        }
        break;
    case Align:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.align.timer))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Encoder_Align_Startup))) ||
                (i_cmd_ext_below_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Curr_Mode_FOC_Encoder_Align_Startup))))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Curr_Mode_FOC_Encoder_Align_Startup)))
            {
                next = Current_CL;
            }
            else if ((w_cmd_ext_above_thresh_high && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup)) || (w_cmd_ext_above_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Encoder_Align_Startup)))
            {
                sm_ptr->vars.speed_reset_required = (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup) == true);
                next = Speed_CL; // S061987
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Align:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.align.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup)) || (T_cmd_ext_below_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_Align_Startup)))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_Align_Startup))
            {
                next = Torque_CL;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Six_Pulse:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((ctrl_ptr->six_pulse_inj.state == Finished_Success) || (ctrl_ptr->six_pulse_inj.state == Finished_Ambiguous))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (i_cmd_ext_below_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr,Curr_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                next = Current_CL;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Six_Pulse:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((ctrl_ptr->six_pulse_inj.state == Finished_Success) || (ctrl_ptr->six_pulse_inj.state == Finished_Ambiguous))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (T_cmd_ext_below_thresh && (Mode(params_ptr,Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                next = Torque_CL;
            }
            else if (T_cmd_ext_above_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case High_Freq:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.high_freq.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup)) || (i_cmd_ext_below_thresh && Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup)))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case High_Freq:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.high_freq.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup)) || (T_cmd_ext_below_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup)))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = Torque_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Speed_OL_To_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((w_cmd_int_below_thresh_high && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup)))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.speed_ol_to_cl.timer))
        {
            next = Speed_CL;
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Speed_OL_To_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((w_cmd_int_below_thresh_high && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.speed_ol_to_cl.timer))
        {
            next = Speed_CL;
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Dyno_Lock:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.dyno_lock.timer))
        {
            if (i_cmd_ext_above_thresh && Mode(params_ptr,Curr_Mode_FOC_Sensorless_Dyno))
            {
                next = Current_CL;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Dyno_Lock:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.dyno_lock.timer))
        {
            if (T_cmd_ext_above_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_Dyno))
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
        if (!vars_ptr->en)
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
        else if (StopWatchIsDone(&profiler_ptr->timer))  // timer's period is dynamically set
        {
            next = (STATE_ID_t)(((uint8_t)(current)) + 1U);
        }
        break;
    case Prof_Finished:
        if (!vars_ptr->en)
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
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (i_cmd_int_below_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Curr_Mode_FOC_Sensorless_SixPulse_Startup) ||
            Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup) || Mode(params_ptr,Curr_Mode_FOC_Encoder_Align_Startup) || Mode(params_ptr,Curr_Mode_FOC_Hall)))
        {
            next = Brake_Boot;
        }
        else if (i_cmd_int_below_thresh && Mode(params_ptr,Curr_Mode_FOC_Sensorless_Dyno))
        {
            next = Dyno_Lock;
        }
        break;
#elif defined(CTRL_METHOD_TBC)
    case Current_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (i_cmd_int_below_thresh && Mode(params_ptr,Curr_Mode_Block_Comm_Hall))
        {
            next = Brake_Boot;
        }
        break;
#endif
#if defined(CTRL_METHOD_SFO)
    case Torque_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (T_cmd_int_below_thresh && (Mode(params_ptr,Trq_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup)))
        {
            next = Brake_Boot;
        }
        else if (T_cmd_int_below_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_Dyno))
        {
            next = Dyno_Lock;
        }
        break;
#endif
    case Speed_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
#if defined(CTRL_METHOD_RFO)
        else if ((w_cmd_int_below_thresh_high && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup)))
            || ((sm_ptr->vars.high_freq.used ? w_cmd_int_below_thresh_low : w_cmd_int_below_thresh_high) && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            || (w_cmd_int_below_thresh_low && (Mode(params_ptr, Speed_Mode_FOC_Encoder_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Hall)))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if ((profiler_ptr->ramp_down_status == Task_Finished) && Mode(params_ptr,Profiler_Mode))
        {
            next = Prof_Finished;
        }
        break;
#elif defined(CTRL_METHOD_SFO)
        else if ((w_cmd_int_below_thresh_high && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup)))
            || ((sm_ptr->vars.high_freq.used ? w_cmd_int_below_thresh_low : w_cmd_int_below_thresh_high) && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if ((profiler_ptr->ramp_down_status == Task_Finished) && Mode(params_ptr,Profiler_Mode))
        {
            next = Prof_Finished;
        }
        break;
#elif defined(CTRL_METHOD_TBC)
        else if (w_cmd_int_below_thresh_low && Mode(params_ptr,Speed_Mode_Block_Comm_Hall))
        {
            next = Brake_Boot;
        }
        break;
#else
        else
        {

        }
        break;
#endif
    case Fault:
    	if(sm_ptr->vars.fault.clr_success)
        {
            next = Init;
        }
        break;
    }
    sm_ptr->next = next;

}

void STATE_MACHINE_Init()
{
    PARAMS_UpdateLookupTable();

    for(uint32_t motor_ins =0; motor_ins<MOTOR_CTRL_NO_OF_MOTOR; motor_ins++)
    {
        motor[motor_ins].sm_ptr->add_callback.RunISR0 = EmptyFcn;
        motor[motor_ins].sm_ptr->add_callback.RunISR1 = EmptyFcn;

        FeedbackISR0Wrap[motor_ins] = OBS_RunISR0;

		//									                        Entry(),				Exit(),				    RunISR0(),			    RunISR1()

        motor[motor_ins].sm_ptr->states[Init]             = (STATE_t){ &InitEntry,          &EmptyFcn,             &InitISR0,              &EmptyFcn       };
        motor[motor_ins].sm_ptr->states[Brake_Boot]       = (STATE_t){ &BrakeBootEntry,     &EmptyFcn,             &BrakeBootISR0,         &BrakeBootISR1  };
        motor[motor_ins].sm_ptr->states[Volt_OL]          = (STATE_t){ &VoltOLEntry,        &VoltOLExit,           &VoltOLISR0,            &VoltOLISR1     };
        motor[motor_ins].sm_ptr->states[Speed_CL]         = (STATE_t){ &SpeedCLEntry,       &EmptyFcn,             &SpeedCLISR0,           &SpeedCLISR1    };
        motor[motor_ins].sm_ptr->states[Fault]            = (STATE_t){ &FaultEntry,         &FaultExit,            &EmptyFcn,              &FaultISR1      };
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
        motor[motor_ins].sm_ptr->states[Align]            = (STATE_t){ &AlignEntry,         &AlignExit,            &AlignISR0,             &EmptyFcn       };
        motor[motor_ins].sm_ptr->states[Six_Pulse]        = (STATE_t){ &SixPulseEntry,      &SixPulseExit,         &SixPulseISR0,          &SixPulseISR1   };
        motor[motor_ins].sm_ptr->states[High_Freq]        = (STATE_t){ &HighFreqEntry,      &HighFreqExit,         &HighFreqISR0,          &HighFreqISR1   };
        motor[motor_ins].sm_ptr->states[Speed_OL_To_CL]   = (STATE_t){ &SpeedOLToCLEntry,   &SpeedOLToCLExit,	   &SpeedOLToCLISR0,       &SpeedOLToCLISR1};
        motor[motor_ins].sm_ptr->states[Dyno_Lock]        = (STATE_t){ &DynoLockEntry,      &DynoLockExit,         &DynoLockISR0,          &DynoLockISR1   };
        motor[motor_ins].sm_ptr->states[Prof_Rot_Lock]    = (STATE_t){ &MotorProfEntry,     &MotorProfExit,        &MotorProfISR0,         &EmptyFcn       };
        motor[motor_ins].sm_ptr->states[Prof_R]           = (STATE_t){ &MotorProfEntry,     &MotorProfExit,        &MotorProfISR0,         &EmptyFcn       };
        motor[motor_ins].sm_ptr->states[Prof_Ld]          = (STATE_t){ &MotorProfEntry,     &MotorProfExit,        &MotorProfISR0,         &EmptyFcn       };
        motor[motor_ins].sm_ptr->states[Prof_Lq]          = (STATE_t){ &MotorProfEntry,     &MotorProfExit,        &MotorProfISR0,         &EmptyFcn       };
        motor[motor_ins].sm_ptr->states[Prof_Finished]    = (STATE_t){ &MotorProfEntry,     &MotorProfExit,        &MotorProfISR0,         &EmptyFcn       };
#endif
#if defined(CTRL_METHOD_SFO)
        motor[motor_ins].sm_ptr->states[Torque_CL]        = (STATE_t){ &TorqueCLEntry,		&EmptyFcn,			    &TorqueCLISR0,		    &TorqueCLISR1       };
#elif defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
        motor[motor_ins].sm_ptr->states[Current_CL]       = (STATE_t){ &CurrentCLEntry,		&EmptyFcn,			    &CurrentCLISR0,		    &CurrentCLISR1       };
#endif
#if defined(CTRL_METHOD_RFO)
        motor[motor_ins].sm_ptr->states[Current_OL]       = (STATE_t){ &CurrentOLEntry,		&CurrentOLExit,		    &CurrentOLISR0,		    &CurrentOLISR1       };
#endif

        motor[motor_ins].sm_ptr->current = Init;
        motor[motor_ins].sm_ptr->vars.init.param_init_done = false;
        motor[motor_ins].sm_ptr->states[motor[motor_ins].sm_ptr->current].Entry(&motor[motor_ins]);

        motor[motor_ins].sm_ptr->vars.fault.clr_try_cnt = 0U;

		// Must not be STATE_MACHINE_ResetAllModules because that is one of the functions requested by gui:
        FCN_EXE_HANDLER_Init();
        FCN_EXE_HANDLER_Reset();

	#if defined(PC_TEST)
		for (uint32_t index = 0; index < sizeof(motor[motor_ins].sm_ptr->vars.capture_vals) / sizeof(float); ++index)
		{
			motor[motor_ins].sm_ptr->vars.capture_vals[index] = 0.0f;
			motor[motor_ins].sm_ptr->vars.capture_channels[index] = &motor[motor_ins].sm_ptr->vars.capture_vals[index];
		}
	#endif
  }
  for(uint32_t motor_ins =0; motor_ins<MOTOR_CTRL_NO_OF_MOTOR; motor_ins++)
  {
      hw_fcn.StartPeripherals(motor_ins);		// enable all ISRs, PWMs, ADCs, etc.
  }
}

RAMFUNC_BEGIN
void STATE_MACHINE_RunISR0(MOTOR_t *motor_ptr)
{
#if defined(PC_TEST)
    motor_ptr = &motor[0];
#endif
    STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
    CommonISR0Wrap(motor_ptr);
    sm_ptr->states[sm_ptr->current].RunISR0(motor_ptr);

    sm_ptr->add_callback.RunISR0(motor_ptr);
}
RAMFUNC_END

void STATE_MACHINE_RunISR1(MOTOR_t *motor_ptr)
{
#if defined(PC_TEST)
   motor_ptr = &motor[0];
#endif
    STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;    
    CommonISR1Wrap(motor_ptr);
    sm_ptr->states[sm_ptr->current].RunISR1(motor_ptr);

    sm_ptr->add_callback.RunISR1(motor_ptr);

    ConditionCheck(motor_ptr);

    if (sm_ptr->next != sm_ptr->current)
    {
#if defined(PC_TEST)
        for (uint32_t index = 0; index < sizeof(sm_ptr->vars.capture_vals) / sizeof(float); ++index)
        {
            sm_ptr->vars.capture_vals[index] = *(sm_ptr->vars.capture_channels[index]);
        }
#endif
        sm_ptr->states[sm_ptr->current].Exit(motor_ptr);
        sm_ptr->states[sm_ptr->next].Entry(motor_ptr);
        sm_ptr->current = sm_ptr->next;
        // This instruction (motor_ptr->sm_ptr->current update) should be the last one ue to data integrity reasons (between ISR0/ISR1)
    }

}
