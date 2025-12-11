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


SENSOR_IFACE_t sensor_iface[MOTOR_CTRL_NO_OF_MOTOR] = { 0 };

#if !defined(PC_TEST)  // TBD: REGRESSION_TEST & SIL_TEST for single shunt
#define SHUNT_TYPE  (params_ptr->sys.analog.shunt.type)
#else
#define SHUNT_TYPE  (Three_Shunt)
#endif

#if !defined(MOTOR_CTRL_DISABLE_ADDON_FEATURES)
static inline void ApplyCalibration(CALIB_PARAMS_t* calib, ANALOG_SENSOR_t* analog_sens)
{
    analog_sens->calibrated = analog_sens->raw * calib->gain + calib->offset;
}
static inline void ApplyFilterISR0(ANALOG_SENSOR_t* analog_sens)
{
    analog_sens->filt += (analog_sens->calibrated - analog_sens->filt) * analog_sens->filt_coeff;
}
#else
static inline void ApplyCalibration(CALIB_PARAMS_t* calib, ANALOG_SENSOR_t* analog_sens)
{
    analog_sens->calibrated = analog_sens->raw;
}
static inline void ApplyFilterISR0(ANALOG_SENSOR_t* analog_sens)
{
    analog_sens->filt = analog_sens->calibrated;
}
#endif

static inline void ApplyFilterISR1(ANALOG_SENSOR_t* analog_sens)
{
    analog_sens->filt += (analog_sens->calibrated - analog_sens->filt) * analog_sens->filt_coeff;
}

static inline void ApplyOffset(ANALOG_SENSOR_t* analog_sens, float offset)
{
    analog_sens->calibrated -= offset;
}

static inline void SetUVW(UVW_t* uvw, const float* u, const float* v, const float* w)
{
    uvw->u = *u;
    uvw->v = *v;
    uvw->w = *w;
}

void SENSOR_IFACE_Init(MOTOR_t *motor_ptr)
{
    SENSOR_IFACE_t*  sensor_iface_ptr = motor_ptr->sensor_iface_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
#if defined(PC_TEST)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
#endif

    // Analog sensors' initializations
    float i_ph_filt_coeff = params_ptr->sys.analog.filt.w0_i_ph * params_ptr->sys.samp.ts0;
    sensor_iface_ptr->i_u.filt_coeff = i_ph_filt_coeff;
    sensor_iface_ptr->i_v.filt_coeff = i_ph_filt_coeff;
    sensor_iface_ptr->i_w.filt_coeff = i_ph_filt_coeff;
    float v_ph_filt_coeff = params_ptr->sys.analog.filt.w0_v_ph * params_ptr->sys.samp.ts0;
    sensor_iface_ptr->v_uz.filt_coeff = v_ph_filt_coeff;
    sensor_iface_ptr->v_vz.filt_coeff = v_ph_filt_coeff;
    sensor_iface_ptr->v_wz.filt_coeff = v_ph_filt_coeff;
    sensor_iface_ptr->v_dc.filt_coeff = params_ptr->sys.analog.filt.w0_v_dc * params_ptr->sys.samp.ts1;
    sensor_iface_ptr->temp_ps.filt_coeff = params_ptr->sys.analog.filt.w0_temp_ps * params_ptr->sys.samp.ts1;
    sensor_iface_ptr->pot.filt_coeff = params_ptr->sys.analog.filt.w0_pot * params_ptr->sys.samp.ts1;
#if defined(PC_TEST)
    sensor_iface_ptr->uvw_idx = &ctrl_ptr->volt_mod.uvw_idx; // must be done by HwInterface based on HW
#endif
    // Auto offset nulling
    static const float Offset_Null_Tau_Ratio = 5.0f;	// tau = (null_time/Offset_Null_Tau_Ratio), 1-exp(-5)=0.9933
    sensor_iface_ptr->offset_null_loop_gain = Offset_Null_Tau_Ratio / (params_ptr->sys.samp.fs0 * params_ptr->sys.analog.offset_null_time);
    //sensor_iface_ptr->i_uvw_offset_null = UVW_Zero;
    vars_ptr->dir = +1.0f;
}

void SENSOR_IFACE_Reset(MOTOR_t *motor_ptr)
{
	SENSOR_IFACE_t*  sensor_iface_ptr = motor_ptr->sensor_iface_ptr;

    // Analog sensors' filter reset
    sensor_iface_ptr->i_u.filt = 0.0f;
    sensor_iface_ptr->i_v.filt = 0.0f;
    sensor_iface_ptr->i_w.filt = 0.0f;
    sensor_iface_ptr->v_uz.filt = 0.0f;
    sensor_iface_ptr->v_vz.filt = 0.0f;
    sensor_iface_ptr->v_wz.filt = 0.0f;
    sensor_iface_ptr->v_dc.filt = 0.0f;
    sensor_iface_ptr->temp_ps.filt = 0.0f;
    sensor_iface_ptr->pot.filt = 0.0f;
}

RAMFUNC_BEGIN

void SENSOR_IFACE_RunISR0(MOTOR_t *motor_ptr)
{
    // Three shunt:  i_samp -> i_uvw -> calibration -> offset nulling -> filters
    // Single shunt: i_samp -> calibration -> offset nulling -> i_xyz -> i_uvw -> filters

    SENSOR_IFACE_t*  sensor_iface_ptr = motor_ptr->sensor_iface_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    PROFILER_t* profiler_ptr = motor_ptr->profiler_ptr;
#endif
    #if defined(CTRL_METHOD_TBC)
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
    #endif
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
	STATE_MACHINE_t* sm_ptr = motor_ptr->sm_ptr;
#endif

    switch (SHUNT_TYPE)
    {
    default:
    case Three_Shunt:
        // Current reconstruction
        sensor_iface_ptr->i_u.raw = sensor_iface_ptr->i_samp_0.raw;
        sensor_iface_ptr->i_v.raw = sensor_iface_ptr->i_samp_1.raw;
        sensor_iface_ptr->i_w.raw = sensor_iface_ptr->i_samp_2.raw;

        // Apply calibration
        ApplyCalibration(&params_ptr->sys.analog.calib.i_u, &sensor_iface_ptr->i_u);
        ApplyCalibration(&params_ptr->sys.analog.calib.i_v, &sensor_iface_ptr->i_v);
        ApplyCalibration(&params_ptr->sys.analog.calib.i_w, &sensor_iface_ptr->i_w);

        // Apply auto-generated offsets (not to be confused with calibration offsets)
        ApplyOffset(&sensor_iface_ptr->i_u, sensor_iface_ptr->i_uvw_offset_null.u);
        ApplyOffset(&sensor_iface_ptr->i_v, sensor_iface_ptr->i_uvw_offset_null.v);
        ApplyOffset(&sensor_iface_ptr->i_w, sensor_iface_ptr->i_uvw_offset_null.w);
        break;

    case Single_Shunt:
        // Apply calibration (calibration parameters: calib.i_v, only one shunt)
        ApplyCalibration(&params_ptr->sys.analog.calib.i_v, &sensor_iface_ptr->i_samp_0);
        ApplyCalibration(&params_ptr->sys.analog.calib.i_v, &sensor_iface_ptr->i_samp_1);

        // Apply auto-generated offsets (not to be confused with calibration offsets)
        ApplyOffset(&sensor_iface_ptr->i_samp_0, sensor_iface_ptr->i_uvw_offset_null.v);
        ApplyOffset(&sensor_iface_ptr->i_samp_1, sensor_iface_ptr->i_uvw_offset_null.v);

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
        // Current reconstruction
        sensor_iface_ptr->i_xyz[0] = -sensor_iface_ptr->i_samp_1.calibrated;
        sensor_iface_ptr->i_xyz[1] = -sensor_iface_ptr->i_samp_0.calibrated + sensor_iface_ptr->i_samp_1.calibrated;
        sensor_iface_ptr->i_xyz[2] = +sensor_iface_ptr->i_samp_0.calibrated;
        sensor_iface_ptr->i_u.calibrated = sensor_iface_ptr->i_xyz[WORD_TO_BYTE(*sensor_iface_ptr->uvw_idx, 0U)];
        sensor_iface_ptr->i_v.calibrated = sensor_iface_ptr->i_xyz[WORD_TO_BYTE(*sensor_iface_ptr->uvw_idx, 1U)];
        sensor_iface_ptr->i_w.calibrated = sensor_iface_ptr->i_xyz[WORD_TO_BYTE(*sensor_iface_ptr->uvw_idx, 2U)];
        break;

#elif defined(CTRL_METHOD_TBC)
        // Current reconstruction
        if (params_ptr->ctrl.mode == Volt_Mode_Open_Loop)
        {
            sensor_iface_ptr->i_xyz[0] = -sensor_iface_ptr->i_samp_1.calibrated;
            sensor_iface_ptr->i_xyz[1] = -sensor_iface_ptr->i_samp_0.calibrated + sensor_iface_ptr->i_samp_1.calibrated;
            sensor_iface_ptr->i_xyz[2] = +sensor_iface_ptr->i_samp_0.calibrated;
            sensor_iface_ptr->i_u.calibrated = sensor_iface_ptr->i_xyz[WORD_TO_BYTE(*sensor_iface_ptr->uvw_idx, 0U)];
            sensor_iface_ptr->i_v.calibrated = sensor_iface_ptr->i_xyz[WORD_TO_BYTE(*sensor_iface_ptr->uvw_idx, 1U)];
            sensor_iface_ptr->i_w.calibrated = sensor_iface_ptr->i_xyz[WORD_TO_BYTE(*sensor_iface_ptr->uvw_idx, 2U)];
        }
        else
        {
            float i_samp_sum = (ctrl_ptr->block_comm.v_s_sign ? -1.0f : +1.0f) * (sensor_iface_ptr->i_samp_0.calibrated + sensor_iface_ptr->i_samp_1.calibrated);
            sensor_iface_ptr->i_u.calibrated = ((ctrl_ptr->block_comm.high_z_state.u) ? 0.0f : (ctrl_ptr->block_comm.i_uvw_coeff.u - 0.5f) * i_samp_sum);
            sensor_iface_ptr->i_v.calibrated = ((ctrl_ptr->block_comm.high_z_state.v) ? 0.0f : (ctrl_ptr->block_comm.i_uvw_coeff.v - 0.5f) * i_samp_sum);
            sensor_iface_ptr->i_w.calibrated = ((ctrl_ptr->block_comm.high_z_state.w) ? 0.0f : (ctrl_ptr->block_comm.i_uvw_coeff.w - 0.5f) * i_samp_sum);
        }
        break;
#endif
    }

    // Apply filters
    ApplyFilterISR0(&sensor_iface_ptr->i_u);
    ApplyFilterISR0(&sensor_iface_ptr->i_v);
    ApplyFilterISR0(&sensor_iface_ptr->i_w);

#if defined(PC_TEST)
    SetUVW(&vars_ptr->i_uvw_fb, &sensor_iface_ptr->i_u.calibrated, &sensor_iface_ptr->i_v.calibrated, &sensor_iface_ptr->i_w.calibrated);
#else
    SetUVW(&vars_ptr->i_uvw_fb, &sensor_iface_ptr->i_u.filt, &sensor_iface_ptr->i_v.filt, &sensor_iface_ptr->i_w.filt);
#endif

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    ClarkeTransform(&vars_ptr->i_uvw_fb, &vars_ptr->i_ab_fb_tot);
    if (!sm_ptr->vars.high_freq.used && !profiler_ptr->freq_sep_active)
    {
        vars_ptr->i_ab_fb = vars_ptr->i_ab_fb_tot;
    }
#elif defined(CTRL_METHOD_TBC)
    if (params_ptr->ctrl.mode == Volt_Mode_Open_Loop)
    {
        ClarkeTransform(&vars_ptr->i_uvw_fb, &vars_ptr->i_ab_fb_tot);
        vars_ptr->i_ab_fb = vars_ptr->i_ab_fb_tot;
    }
    else if (params_ptr->ctrl.tbc.mode == Block_Commutation)
    {
        BLOCK_COMM_RunCurrSampISR0(motor_ptr);
    }
#endif

#if !defined(MOTOR_CTRL_DISABLE_ADDON_FEATURES)
    static bool phase_voltages_measured_prev = true;
    bool phase_voltages_measured = hw_fcn.ArePhaseVoltagesMeasured(motor_ptr->motor_instance);
    if (phase_voltages_measured)
    {
        // Apply calibration on raw values
        ApplyCalibration(&params_ptr->sys.analog.calib.v_uz, &sensor_iface_ptr->v_uz);
        ApplyCalibration(&params_ptr->sys.analog.calib.v_vz, &sensor_iface_ptr->v_vz);
        ApplyCalibration(&params_ptr->sys.analog.calib.v_wz, &sensor_iface_ptr->v_wz);

        // Apply filters
        ApplyFilterISR0(&sensor_iface_ptr->v_uz);
        ApplyFilterISR0(&sensor_iface_ptr->v_vz);
        ApplyFilterISR0(&sensor_iface_ptr->v_wz);

#if defined(PC_TEST)
        SetUVW(&vars_ptr->v_uvw_z_fb, &sensor_iface_ptr->v_uz.calibrated, &sensor_iface_ptr->v_vz.calibrated, &sensor_iface_ptr->v_wz.calibrated);
#else
        SetUVW(&vars_ptr->v_uvw_z_fb, &sensor_iface_ptr->v_uz.filt, &sensor_iface_ptr->v_vz.filt, &sensor_iface_ptr->v_wz.filt);
#endif

        vars_ptr->v_nz_fb = (1.0f / 3.0f) * (vars_ptr->v_uvw_z_fb.u + vars_ptr->v_uvw_z_fb.v + vars_ptr->v_uvw_z_fb.w);
        vars_ptr->v_uvw_n_fb.u = vars_ptr->v_uvw_z_fb.u - vars_ptr->v_nz_fb;
        vars_ptr->v_uvw_n_fb.v = vars_ptr->v_uvw_z_fb.v - vars_ptr->v_nz_fb;
        vars_ptr->v_uvw_n_fb.w = vars_ptr->v_uvw_z_fb.w - vars_ptr->v_nz_fb;
        ClarkeTransform(&vars_ptr->v_uvw_n_fb, &vars_ptr->v_ab_fb);
    }
    else if (FALL_EDGE(phase_voltages_measured_prev, phase_voltages_measured))
    {
        vars_ptr->v_uvw_z_fb = UVW_Zero;
        vars_ptr->v_nz_fb = 0.0f;
        vars_ptr->v_uvw_n_fb  = UVW_Zero;
        vars_ptr->v_ab_fb = AB_Zero;
    }
    phase_voltages_measured_prev = phase_voltages_measured;
#endif

#if defined(PC_TEST)
    vars_ptr->test[12] = sensor_iface_ptr->i_u.filt;
    vars_ptr->test[13] = sensor_iface_ptr->i_v.filt;
    vars_ptr->test[14] = sensor_iface_ptr->i_w.filt;
    vars_ptr->test[15] = sensor_iface_ptr->v_uz.filt;
    vars_ptr->test[16] = sensor_iface_ptr->v_vz.filt;
    vars_ptr->test[17] = sensor_iface_ptr->v_wz.filt;
#endif
}
RAMFUNC_END

void SENSOR_IFACE_RunISR1(MOTOR_t *motor_ptr)
{

    SENSOR_IFACE_t*  sensor_iface_ptr = motor_ptr->sensor_iface_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
	PROFILER_t* profiler_ptr = motor_ptr->profiler_ptr;
#endif

    // Apply calibration on raw values
    ApplyCalibration(&params_ptr->sys.analog.calib.v_dc, &sensor_iface_ptr->v_dc);
    ApplyCalibration(&params_ptr->sys.analog.calib.temp_ps, &sensor_iface_ptr->temp_ps);
    ApplyCalibration(&params_ptr->sys.analog.calib.pot, &sensor_iface_ptr->pot);

    // Apply filters
    ApplyFilterISR1(&sensor_iface_ptr->v_dc);
    ApplyFilterISR1(&sensor_iface_ptr->temp_ps);
    ApplyFilterISR1(&sensor_iface_ptr->pot);

#if defined(PC_TEST)
    vars_ptr->v_dc = sensor_iface_ptr->v_dc.calibrated;
    vars_ptr->temp_ps = sensor_iface_ptr->temp_ps.calibrated;
    vars_ptr->cmd_int = sensor_iface_ptr->pot.calibrated;
    vars_ptr->test[18] = sensor_iface_ptr->v_dc.filt;
    vars_ptr->test[19] = sensor_iface_ptr->temp_ps.filt;
    vars_ptr->test[20] = sensor_iface_ptr->pot.filt;
#else
    vars_ptr->v_dc = sensor_iface_ptr->v_dc.filt;
    vars_ptr->temp_ps = sensor_iface_ptr->temp_ps.filt;
    vars_ptr->cmd_int = sensor_iface_ptr->pot.filt;
#endif

    switch (params_ptr->sys.cmd.source)
    {
    case Internal:
    default:
    	vars_ptr->cmd_final = vars_ptr->cmd_int;
    	vars_ptr->brk = (sensor_iface_ptr->digital.brk == 1U);
    	vars_ptr->clr_faults = (vars_ptr->cmd_final < params_ptr->sys.faults.cmd_clr_thresh);
    	vars_ptr->dir = (sensor_iface_ptr->digital.dir == 1U) ? (+1.0f) : (-1.0f);
        break;
    case External:
    	vars_ptr->cmd_final = vars_ptr->cmd_ext;
        break;
    }

    // Other sensor related tasks
    switch (params_ptr->ctrl.mode)
    {
    default:
    case Volt_Mode_Open_Loop:
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    case Speed_Mode_FOC_Sensorless_Align_Startup:
    case Speed_Mode_FOC_Sensorless_SixPulse_Startup:
    case Speed_Mode_FOC_Sensorless_HighFreq_Startup:
    case Speed_Mode_FOC_Sensorless_Volt_Startup:
#endif
#if defined(CTRL_METHOD_TBC)
    case Speed_Mode_Block_Comm_Hall:
#elif defined(CTRL_METHOD_RFO)
    case Speed_Mode_FOC_Encoder_Align_Startup:
    case Speed_Mode_FOC_Hall:
    case Curr_Mode_Open_Loop:
#endif
        vars_ptr->w_cmd_ext.mech = vars_ptr->cmd_final * vars_ptr->dir * params_ptr->sys.cmd.w_max.mech;
        vars_ptr->w_cmd_ext.elec = MECH_TO_ELEC(vars_ptr->w_cmd_ext.mech, params_ptr->motor.P);
        break;
#if defined(CTRL_METHOD_RFO)
    case Curr_Mode_FOC_Sensorless_Align_Startup:
    case Curr_Mode_FOC_Sensorless_SixPulse_Startup:
    case Curr_Mode_FOC_Sensorless_HighFreq_Startup:
    case Curr_Mode_FOC_Sensorless_Dyno:
    case Curr_Mode_FOC_Encoder_Align_Startup:
    case Curr_Mode_FOC_Hall:
#elif defined(CTRL_METHOD_TBC)
    case Curr_Mode_Block_Comm_Hall:
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
        vars_ptr->i_cmd_ext = vars_ptr->cmd_final * vars_ptr->dir * params_ptr->sys.cmd.i_max;
        break;
#endif
#if defined(CTRL_METHOD_SFO)
    case Trq_Mode_FOC_Sensorless_Align_Startup:
    case Trq_Mode_FOC_Sensorless_SixPulse_Startup:
    case Trq_Mode_FOC_Sensorless_HighFreq_Startup:
    case Trq_Mode_FOC_Sensorless_Dyno:
        vars_ptr->T_cmd_ext = vars_ptr->cmd_final * vars_ptr->dir * params_ptr->sys.cmd.T_max;
        break;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    case Profiler_Mode:
        vars_ptr->w_cmd_ext.mech = profiler_ptr->w_cmd_final.mech * vars_ptr->dir;
        vars_ptr->w_cmd_ext.elec = profiler_ptr->w_cmd_final.elec * vars_ptr->dir;
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Position_Mode_FOC_Encoder_Align_Startup:
        vars_ptr->p_cmd_ext =vars_ptr->cmd_final*vars_ptr->dir*params_ptr->sys.cmd.p_max;
         break;
#endif
    }
}

RAMFUNC_BEGIN
void SENSOR_IFACE_OffsetNullISR0(MOTOR_t *motor_ptr)
{
    SENSOR_IFACE_t*  sensor_iface_ptr = motor_ptr->sensor_iface_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    float i_samp_ave;

    // IIR bandwidth [Hz] = (loop_gain*fs)/(2*pi)
    switch (SHUNT_TYPE)
    {
    default:
    case Three_Shunt:
        sensor_iface_ptr->i_uvw_offset_null.u += sensor_iface_ptr->offset_null_loop_gain * sensor_iface_ptr->i_u.calibrated;
        sensor_iface_ptr->i_uvw_offset_null.v += sensor_iface_ptr->offset_null_loop_gain * sensor_iface_ptr->i_v.calibrated;
        sensor_iface_ptr->i_uvw_offset_null.w += sensor_iface_ptr->offset_null_loop_gain * sensor_iface_ptr->i_w.calibrated;
        break;

    case Single_Shunt:
        i_samp_ave = 0.5f * (sensor_iface_ptr->i_samp_0.calibrated + sensor_iface_ptr->i_samp_1.calibrated);
        sensor_iface_ptr->i_uvw_offset_null.v += sensor_iface_ptr->offset_null_loop_gain * i_samp_ave;
        break;
    }
}
RAMFUNC_END
