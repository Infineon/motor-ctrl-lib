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

SENSOR_IFACE_t sensor_iface = { 0 };

#if !defined(PC_TEST)  // TBD: REGRESSION_TEST & SIL_TEST for single shunt
#define SHUNT_TYPE  (params.sys.analog.shunt.type)
#else
#define SHUNT_TYPE  (Three_Shunt)
#endif

static inline void ApplyCalibration(CALIB_PARAMS_t* calib, ANALOG_SENSOR_t* analog_sens)
{
    analog_sens->calibrated = analog_sens->raw * calib->gain + calib->offset;
}

static inline void ApplyFilter(ANALOG_SENSOR_t* analog_sens)
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

void SENSOR_IFACE_Init()
{
    // Analog sensors' initializations
    float i_ph_filt_coeff = params.sys.analog.filt.w0_i_ph * params.sys.samp.ts0;
    sensor_iface.i_u.filt_coeff = i_ph_filt_coeff;
    sensor_iface.i_v.filt_coeff = i_ph_filt_coeff;
    sensor_iface.i_w.filt_coeff = i_ph_filt_coeff;
    float v_ph_filt_coeff = params.sys.analog.filt.w0_v_ph * params.sys.samp.ts0;
    sensor_iface.v_uz.filt_coeff = v_ph_filt_coeff;
    sensor_iface.v_vz.filt_coeff = v_ph_filt_coeff;
    sensor_iface.v_wz.filt_coeff = v_ph_filt_coeff;
    sensor_iface.v_dc.filt_coeff = params.sys.analog.filt.w0_v_dc * params.sys.samp.ts1;
    sensor_iface.temp_ps.filt_coeff = params.sys.analog.filt.w0_temp_ps * params.sys.samp.ts1;
    sensor_iface.pot.filt_coeff = params.sys.analog.filt.w0_pot * params.sys.samp.ts1;
#if defined(PC_TEST)
    sensor_iface.uvw_idx = &ctrl.volt_mod.uvw_idx; // must be done by HwInterface based on HW
#endif
    // Auto offset nulling
    static const float Offset_Null_Tau_Ratio = 5.0f;	// tau = (null_time/Offset_Null_Tau_Ratio), 1-exp(-5)=0.9933
    sensor_iface.offset_null_loop_gain = Offset_Null_Tau_Ratio / (params.sys.samp.fs0 * params.sys.analog.offset_null_time);
    //sensor_iface.i_uvw_offset_null = UVW_Zero;
    
    vars.dir = +1.0f;
}

void SENSOR_IFACE_Reset()
{
    // Analog sensors' filter reset
    sensor_iface.i_u.filt = 0.0f;
    sensor_iface.i_v.filt = 0.0f;
    sensor_iface.i_w.filt = 0.0f;
    sensor_iface.v_uz.filt = 0.0f;
    sensor_iface.v_vz.filt = 0.0f;
    sensor_iface.v_wz.filt = 0.0f;
    sensor_iface.v_dc.filt = 0.0f;
    sensor_iface.temp_ps.filt = 0.0f;
    sensor_iface.pot.filt = 0.0f;
}

RAMFUNC_BEGIN
void SENSOR_IFACE_RunISR0()
{
    // Three shunt:  i_samp -> i_uvw -> calibration -> offset nulling -> filters
    // Single shunt: i_samp -> calibration -> offset nulling -> i_xyz -> i_uvw -> filters
    switch (SHUNT_TYPE)
    {
    default:
    case Three_Shunt:
        // Current reconstruction
        sensor_iface.i_u.raw = sensor_iface.i_samp_0.raw;
        sensor_iface.i_v.raw = sensor_iface.i_samp_1.raw;
        sensor_iface.i_w.raw = sensor_iface.i_samp_2.raw;

        // Apply calibration
        ApplyCalibration(&params.sys.analog.calib.i_u, &sensor_iface.i_u);
        ApplyCalibration(&params.sys.analog.calib.i_v, &sensor_iface.i_v);
        ApplyCalibration(&params.sys.analog.calib.i_w, &sensor_iface.i_w);

        // Apply auto-generated offsets (not to be confused with calibration offsets)
        ApplyOffset(&sensor_iface.i_u, sensor_iface.i_uvw_offset_null.u);
        ApplyOffset(&sensor_iface.i_v, sensor_iface.i_uvw_offset_null.v);
        ApplyOffset(&sensor_iface.i_w, sensor_iface.i_uvw_offset_null.w);
        break;

    case Single_Shunt:
        // Apply calibration (calibration parameters: calib.i_v, only one shunt)
        ApplyCalibration(&params.sys.analog.calib.i_v, &sensor_iface.i_samp_0);
        ApplyCalibration(&params.sys.analog.calib.i_v, &sensor_iface.i_samp_1);

        // Apply auto-generated offsets (not to be confused with calibration offsets)
        ApplyOffset(&sensor_iface.i_samp_0, sensor_iface.i_uvw_offset_null.v);
        ApplyOffset(&sensor_iface.i_samp_1, sensor_iface.i_uvw_offset_null.v);

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
        // Current reconstruction
        sensor_iface.i_xyz[0] = -sensor_iface.i_samp_1.calibrated;
        sensor_iface.i_xyz[1] = -sensor_iface.i_samp_0.calibrated + sensor_iface.i_samp_1.calibrated;
        sensor_iface.i_xyz[2] = +sensor_iface.i_samp_0.calibrated;
        sensor_iface.i_u.calibrated = sensor_iface.i_xyz[WORD_TO_BYTE(*sensor_iface.uvw_idx, 0U)];
        sensor_iface.i_v.calibrated = sensor_iface.i_xyz[WORD_TO_BYTE(*sensor_iface.uvw_idx, 1U)];
        sensor_iface.i_w.calibrated = sensor_iface.i_xyz[WORD_TO_BYTE(*sensor_iface.uvw_idx, 2U)];
        break;

#elif defined(CTRL_METHOD_TBC)
        // Current reconstruction
        if (params.ctrl.mode == Volt_Mode_Open_Loop)
        {
            sensor_iface.i_xyz[0] = -sensor_iface.i_samp_1.calibrated;
            sensor_iface.i_xyz[1] = -sensor_iface.i_samp_0.calibrated + sensor_iface.i_samp_1.calibrated;
            sensor_iface.i_xyz[2] = +sensor_iface.i_samp_0.calibrated;
            sensor_iface.i_u.calibrated = sensor_iface.i_xyz[WORD_TO_BYTE(*sensor_iface.uvw_idx, 0U)];
            sensor_iface.i_v.calibrated = sensor_iface.i_xyz[WORD_TO_BYTE(*sensor_iface.uvw_idx, 1U)];
            sensor_iface.i_w.calibrated = sensor_iface.i_xyz[WORD_TO_BYTE(*sensor_iface.uvw_idx, 2U)];
        }
        else
        {
            float i_samp_sum = (ctrl.block_comm.v_s_sign ? -1.0f : +1.0f) * (sensor_iface.i_samp_0.calibrated + sensor_iface.i_samp_1.calibrated);
            sensor_iface.i_u.calibrated = ((ctrl.block_comm.high_z_state.u) ? 0.0f : (ctrl.block_comm.i_uvw_coeff.u - 0.5f) * i_samp_sum);
            sensor_iface.i_v.calibrated = ((ctrl.block_comm.high_z_state.v) ? 0.0f : (ctrl.block_comm.i_uvw_coeff.v - 0.5f) * i_samp_sum);
            sensor_iface.i_w.calibrated = ((ctrl.block_comm.high_z_state.w) ? 0.0f : (ctrl.block_comm.i_uvw_coeff.w - 0.5f) * i_samp_sum);
        }
        break;
#endif
    }

    // Apply filters
    ApplyFilter(&sensor_iface.i_u);
    ApplyFilter(&sensor_iface.i_v);
    ApplyFilter(&sensor_iface.i_w);

#if defined(PC_TEST)
    SetUVW(&vars.i_uvw_fb, &sensor_iface.i_u.calibrated, &sensor_iface.i_v.calibrated, &sensor_iface.i_w.calibrated);
#else
    SetUVW(&vars.i_uvw_fb, &sensor_iface.i_u.filt, &sensor_iface.i_v.filt, &sensor_iface.i_w.filt);
#endif

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    ClarkeTransform(&vars.i_uvw_fb, &vars.i_ab_fb_tot);
    if (!sm.vars.high_freq.used && !profiler.freq_sep_active)
    {
        vars.i_ab_fb = vars.i_ab_fb_tot;
    }
#elif defined(CTRL_METHOD_TBC)
    if (params.ctrl.mode == Volt_Mode_Open_Loop)
    {
        ClarkeTransform(&vars.i_uvw_fb, &vars.i_ab_fb_tot);
        vars.i_ab_fb = vars.i_ab_fb_tot;
    }
    else if (params.ctrl.tbc.mode == Block_Commutation)
    {
        BLOCK_COMM_RunCurrSampISR0();
    }
#endif

    static bool phase_voltages_measured_prev = true;
    bool phase_voltages_measured = hw_fcn.ArePhaseVoltagesMeasured();
    if (phase_voltages_measured)
    {
        // Apply calibration on raw values
        ApplyCalibration(&params.sys.analog.calib.v_uz, &sensor_iface.v_uz);
        ApplyCalibration(&params.sys.analog.calib.v_vz, &sensor_iface.v_vz);
        ApplyCalibration(&params.sys.analog.calib.v_wz, &sensor_iface.v_wz);

        // Apply filters
        ApplyFilter(&sensor_iface.v_uz);
        ApplyFilter(&sensor_iface.v_vz);
        ApplyFilter(&sensor_iface.v_wz);

#if defined(PC_TEST)
        SetUVW(&vars.v_uvw_z_fb, &sensor_iface.v_uz.calibrated, &sensor_iface.v_vz.calibrated, &sensor_iface.v_wz.calibrated);
#else
        SetUVW(&vars.v_uvw_z_fb, &sensor_iface.v_uz.filt, &sensor_iface.v_vz.filt, &sensor_iface.v_wz.filt);
#endif

        vars.v_nz_fb = (1.0f / 3.0f) * (vars.v_uvw_z_fb.u + vars.v_uvw_z_fb.v + vars.v_uvw_z_fb.w);
        vars.v_uvw_n_fb.u = vars.v_uvw_z_fb.u - vars.v_nz_fb;
        vars.v_uvw_n_fb.v = vars.v_uvw_z_fb.v - vars.v_nz_fb;
        vars.v_uvw_n_fb.w = vars.v_uvw_z_fb.w - vars.v_nz_fb;
        ClarkeTransform(&vars.v_uvw_n_fb, &vars.v_ab_fb);
    }
    else if (FALL_EDGE(phase_voltages_measured_prev, phase_voltages_measured))
    {
        vars.v_uvw_z_fb = UVW_Zero;
        vars.v_nz_fb = 0.0f;
        vars.v_uvw_n_fb  = UVW_Zero;
        vars.v_ab_fb = AB_Zero;
    }
    phase_voltages_measured_prev = phase_voltages_measured;

#if defined(PC_TEST)
    vars.test[12] = sensor_iface.i_u.filt;
    vars.test[13] = sensor_iface.i_v.filt;
    vars.test[14] = sensor_iface.i_w.filt;
    vars.test[15] = sensor_iface.v_uz.filt;
    vars.test[16] = sensor_iface.v_vz.filt;
    vars.test[17] = sensor_iface.v_wz.filt;
#endif
}
RAMFUNC_END

void SENSOR_IFACE_RunISR1()
{
    // Apply calibration on raw values
    ApplyCalibration(&params.sys.analog.calib.v_dc, &sensor_iface.v_dc);
    ApplyCalibration(&params.sys.analog.calib.temp_ps, &sensor_iface.temp_ps);
    ApplyCalibration(&params.sys.analog.calib.pot, &sensor_iface.pot);

    // Apply filters
    ApplyFilter(&sensor_iface.v_dc);
    ApplyFilter(&sensor_iface.temp_ps);
    ApplyFilter(&sensor_iface.pot);

#if defined(PC_TEST)
    vars.v_dc = sensor_iface.v_dc.calibrated;
    vars.temp_ps = sensor_iface.temp_ps.calibrated;
    vars.cmd_int = sensor_iface.pot.calibrated;
    vars.test[18] = sensor_iface.v_dc.filt;
    vars.test[19] = sensor_iface.temp_ps.filt;
    vars.test[20] = sensor_iface.pot.filt;
#else
    vars.v_dc = sensor_iface.v_dc.filt;
    vars.temp_ps = sensor_iface.temp_ps.filt;
    vars.cmd_int = sensor_iface.pot.filt;
#endif

    switch (params.sys.cmd.source)
    {
    case Internal:
    default:
        vars.cmd_final = vars.cmd_int;
        vars.brk = (sensor_iface.digital.brk == 1U);
        vars.clr_faults = (vars.cmd_final < params.sys.faults.cmd_clr_thresh);
        vars.dir = (sensor_iface.digital.dir == 1U) ? (+1.0f) : (-1.0f);
        break;
    case External:
        vars.cmd_final = vars.cmd_ext;
        break;
    }

    // Other sensor related tasks
    switch (params.ctrl.mode)
    {
    default:
    case Volt_Mode_Open_Loop:
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    case Speed_Mode_FOC_Sensorless_Align_Startup:
    case Speed_Mode_FOC_Sensorless_SixPulse_Startup:
    case Speed_Mode_FOC_Sensorless_HighFreq_Startup:
    case Speed_Mode_FOC_Sensorless_Volt_Startup:
#endif
#if defined(CTRL_METHOD_RFO)
    case Speed_Mode_FOC_Encoder_Align_Startup:
    case Speed_Mode_FOC_Hall:
#elif defined(CTRL_METHOD_TBC)
    case Speed_Mode_Block_Comm_Hall:
#endif
        vars.w_cmd_ext.mech = vars.cmd_final * vars.dir * params.sys.cmd.w_max.mech;
        vars.w_cmd_ext.elec = MECH_TO_ELEC(vars.w_cmd_ext.mech, params.motor.P);
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
        vars.i_cmd_ext = vars.cmd_final * vars.dir * params.sys.cmd.i_max;
        break;
#endif
#if defined(CTRL_METHOD_SFO)
    case Trq_Mode_FOC_Sensorless_Align_Startup:
    case Trq_Mode_FOC_Sensorless_SixPulse_Startup:
    case Trq_Mode_FOC_Sensorless_HighFreq_Startup:
    case Trq_Mode_FOC_Sensorless_Dyno:
        vars.T_cmd_ext = vars.cmd_final * vars.dir * params.sys.cmd.T_max;
        break;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    case Profiler_Mode:
        vars.w_cmd_ext.mech = profiler.w_cmd_final.mech * vars.dir;
        vars.w_cmd_ext.elec = profiler.w_cmd_final.elec * vars.dir;
        break;
#endif
    }
}

RAMFUNC_BEGIN
void SENSOR_IFACE_OffsetNullISR0()
{
    float i_samp_ave;

    // IIR bandwidth [Hz] = (loop_gain*fs)/(2*pi)
    switch (SHUNT_TYPE)
    {
    default:
    case Three_Shunt:
        sensor_iface.i_uvw_offset_null.u += sensor_iface.offset_null_loop_gain * sensor_iface.i_u.calibrated;
        sensor_iface.i_uvw_offset_null.v += sensor_iface.offset_null_loop_gain * sensor_iface.i_v.calibrated;
        sensor_iface.i_uvw_offset_null.w += sensor_iface.offset_null_loop_gain * sensor_iface.i_w.calibrated;
        break;

    case Single_Shunt:
        i_samp_ave = 0.5f * (sensor_iface.i_samp_0.calibrated + sensor_iface.i_samp_1.calibrated);
        sensor_iface.i_uvw_offset_null.v += sensor_iface.offset_null_loop_gain * i_samp_ave;
        break;
    }
}
RAMFUNC_END
