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

#if defined(PC_TEST)
static bool DummyFlashWrite(PARAMS_t* ram_data) { return true; }
static bool DummyFlashRead(PARAMS_ID_t id, PARAMS_t* ram_data) { return false; };
#define     ADC_CS_SHUNT_RES    (10.0E-3f)
#define     KIT_ID				(0x0006UL)		// Reserved for PSOC6 HW.
#define     ADC_CS_OPAMP_GAIN   (12.0f)
#else
#include "MotorCtrlHWConfig.h"
#endif

#if !defined(PARAMS_DEF)
#define PARAMS_DEF
PARAMS_t params;
MC_INFO_t mc_info;
HW_FCN_t hw_fcn;
#endif

void PARAMS_Init()
{
#if defined(PC_TEST)
    hw_fcn.FlashRead = DummyFlashRead;
    hw_fcn.FlashWrite = DummyFlashWrite;
#endif

    // Parameterss are initialized and saved in flash if
    // - Flash read is not successful
    // - First time running the FW
    // - FW update causes parameter incompatibility
    bool params_id_check = hw_fcn.FlashRead((PARAMS_ID_t) { PARAMS_CODE, BUILD_CONFIG_ID, PARAMS_VER }, & params);
    if (!params_id_check || PARAMS_ALWAYS_OVERWRITE)
    {
        PARAMS_InitManual();
        PARAMS_InitAutoCalc();
        hw_fcn.FlashWrite(&params);
    }

    // mc_info.chip_id is read by hardware interface from the chip
    mc_info.parameter_version = params.id.ver;
    mc_info.firmware_version = FIRMWARE_VER;
    mc_info.kit_id = KIT_ID;
    mc_info.build_config_id = BUILD_CONFIG_ID;
}

void PARAMS_InitManual()
{
    params.id.code = PARAMS_CODE;
    params.id.build_config = BUILD_CONFIG_ID;
    params.id.ver = PARAMS_VER;
    // Motor Parameters (42BL61 Motor):
    params.motor.P = 8.0f; // [#]
    params.motor.lq = 670.0E-6f; // [H]
    params.motor.ld = 670.0E-6f; // [H]
    params.motor.lam = 6.0E-3f; // [Wb]
    params.motor.r = 450.0E-3f; // [Ohm]
    params.motor.T_max = 0.390f; // [Nm]
    params.motor.i_peak = 10.80f; // [A]
    params.motor.i_cont = 3.50f; // [A]
    params.motor.id_max = 1.75f; // [A]
    params.motor.v_nom = LINE_TO_PHASE(24.0f); // [Vpk-ln]
    params.motor.w_nom.elec = MECH_TO_ELEC(HZ_TO_RADSEC(RPM_TO_HZ(4000.0f)), params.motor.P); // [Ra/sec-elec]
    params.motor.w_max.elec = MECH_TO_ELEC(HZ_TO_RADSEC(RPM_TO_HZ(6000.0f)), params.motor.P); // [Ra/sec-elec]
#if defined(CTRL_METHOD_SFO)
    params.motor.mtpv_margin = 0.90f; // [%]
#endif
    params.motor.i2t.therm_tau = 2.5f; // [sec]
    params.motor.i2t.on_level = 1.00f; // [%]
    params.motor.i2t.off_level = 0.95f; // [%]

    // System Parameters:
    // Sampling Parameters:
    params.sys.samp.fpwm_fs0_ratio = 1U; // []
    params.sys.samp.fs0 = 15.0E3; // [Hz], at least 1.2 decade above maximum electrical frequency, 1.5kHz~>25kHz
    params.sys.samp.fs0_fs1_ratio = 15U; // []
#if defined(PC_TEST)
    params.sys.samp.fsim_fs0_ratio = 4U; // []
#endif
    // Calibration Parameters:
    params.sys.analog.calib.i_u = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, A}
    params.sys.analog.calib.i_v = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, A}
    params.sys.analog.calib.i_w = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, A}
    params.sys.analog.calib.v_uz = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, V}
    params.sys.analog.calib.v_vz = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, V}
    params.sys.analog.calib.v_wz = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, V}
    params.sys.analog.calib.v_dc = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, V}
    params.sys.analog.calib.temp_ps = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, C}
    params.sys.analog.calib.pot = (CALIB_PARAMS_t){ 1.0f, 0.0f }; // {%, %}
    // Analog Sensor Filter Parameters:
    params.sys.analog.filt.w0_i_ph = HZ_TO_RADSEC(params.sys.samp.fs0 * ONE_OVER_TWO_PI); // [Ra/sec]
    params.sys.analog.filt.w0_v_ph = HZ_TO_RADSEC(params.sys.samp.fs0 * ONE_OVER_TWO_PI); // [Ra/sec]
    params.sys.analog.filt.w0_v_dc = TAU_TO_RADSEC(0.100f); // [Ra/sec]
    params.sys.analog.filt.w0_temp_ps = TAU_TO_RADSEC(1.000f); // [Ra/sec]
    params.sys.analog.filt.w0_pot = TAU_TO_RADSEC(0.200f); // [Ra/sec]
    // Shunt Parameters:
    params.sys.analog.shunt.type = Three_Shunt;
#if defined(USING_SGD)
    params.sys.analog.shunt.opamp_gain = Gain_12;
#else
    params.sys.analog.shunt.opamp_gain = ADC_CS_OPAMP_GAIN;
#endif
    params.sys.analog.shunt.hyb_mod.adc_t_min = 3.0E-6f; // [sec]
    params.sys.analog.shunt.hyb_mod.ki = 0.5f; // [#]
    params.sys.analog.shunt.res = ADC_CS_SHUNT_RES; // [Ohm]

    params.sys.analog.offset_null_time = 0.250f; // [sec], this will determine the loop-gain of auto-offset-nulling loop
    // Rate Limiter Parameters:
    params.sys.rate_lim.w_cmd.elec = MECH_TO_ELEC(HZ_TO_RADSEC(RPM_TO_HZ(4000.0f / 4.0f)), params.motor.P); // [(Ra/sec-elec)/sec]
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    params.sys.rate_lim.i_cmd = 10.80f * 10.0f; // [A/sec]
#elif defined(CTRL_METHOD_SFO)
    params.sys.rate_lim.T_cmd = 0.390f * 10.0f; // [Nm/sec]
#endif
    // Fault Parameters:
    params.sys.faults.oc_thresh = 1.20f; // [%]
    params.sys.faults.vdc_time = 0.300f; // [sec]
    params.sys.faults.temp_ps_thresh = 90.0f; // [Celsius]
    params.sys.faults.short_method = Alternate_Short; // []
    params.sys.faults.max_clr_tries = 10U; // []
    params.sys.faults.watchdog_time = 1000; // [ms]
    // Command Parameters:
    params.sys.cmd.source = Internal;
    params.sys.cmd.w_max.mech = HZ_TO_RADSEC(RPM_TO_HZ(4000.0f)); //[Ra/sec-elec]
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    params.sys.cmd.i_max = 3.50f * 0.5f; // [A]
#elif defined(CTRL_METHOD_SFO)
    params.sys.cmd.T_max = 0.390f * 0.5f; // [Nm]
#endif
    // Rest of System Parameters:
    params.sys.fb.hall.track_loop.w0_w = HZ_TO_RADSEC(75.0f); // [Ra/sec]
    params.sys.fb.hall.track_loop.w0_th.min = HZ_TO_RADSEC(1.5f); // [Ra/sec]
    params.sys.fb.hall.track_loop.w0_th.max = HZ_TO_RADSEC(75.0f); // [Ra/sec]
    params.sys.fb.hall.track_loop.tau_ratio = 1.5f; // [sec/sec]=[]
    params.sys.fb.hall.w_zsd_thresh.elec = params.motor.w_nom.elec * 0.008f; // [Ra/sec-elec]
    params.sys.fb.hall.th_r_offset.elec = DEG_TO_RAD(0.0f);
    params.sys.fb.hall.block_comm_offset_comp = Dis; // []

    params.sys.fb.encoder.track_loop.cpr = 4096U * 4U; // [#], Nanotech NME2-UVW-W14-05-C
    params.sys.fb.encoder.track_loop.w0_w = HZ_TO_RADSEC(75.0f);   // [Ra/sec]
    params.sys.fb.encoder.track_loop.w0_th.min = HZ_TO_RADSEC(1.5f); // [Ra/sec]
    params.sys.fb.encoder.track_loop.w0_th.max = HZ_TO_RADSEC(75.0f); // [Ra/sec]
    params.sys.fb.encoder.track_loop.tau_ratio = 1.5f;  // [sec/sec]=[]
    params.sys.fb.encoder.w_zsd_thresh.elec = params.motor.w_nom.elec * 0.008f; // [Ra/sec-elec]

    params.sys.boot_time = 0.250f; // [sec]
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    params.sys.dyno_lock_time = 0.500f; // [sec]
#endif
    params.sys.vdc_nom = 24.0f; // [V]

    // Observer Parameters:
    params.obs.flux_filt.k1 = 0.2f; // [#]
    params.obs.flux_filt.k2 = 0.02f; // [#]
    params.obs.flux_filt.k3 = 0.002f; // [#]
    params.obs.biquad_a[0] = 1.0f; // [#]
    params.obs.biquad_a[1] = 0.0f; // [1/(Ra/sec)]
    params.obs.biquad_a[2] = 0.0f; // [1/(Ra/sec)^2]
    params.obs.biquad_b[0] = 1.0f; // [#]
    params.obs.biquad_b[1] = 1.0f / HZ_TO_RADSEC(25.0f) + 1.0f / HZ_TO_RADSEC(75.0f); // [1/(Ra/sec)]
    params.obs.biquad_b[2] = 1.0f / HZ_TO_RADSEC(25.0f) / HZ_TO_RADSEC(75.0f); // [1/(Ra/sec)^2]
    params.obs.pll.w0 = HZ_TO_RADSEC(75.0f); // [Ra/sec]
    params.obs.pll.w_max.elec = params.motor.w_max.elec * 1.5f; // [Ra/sec-elec]
    params.obs.w_thresh.elec = params.motor.w_nom.elec * 0.20f; // [Ra/sec-elec]
    params.obs.w_hyst.elec = params.obs.w_thresh.elec * 0.25f; // [Ra/sec-elec]
    params.obs.lock_time = 0.500f; // [sec]

    // Mechanical System Parameters:
    params.mech.inertia = 1.1E-5f; // [kg.m^2]=[(N.m)/(Ra/sec-mech).sec], =1/2*m*r^2
    params.mech.viscous = 1.2E-5f; // [kg.m^2/sec]=[(N.m)/(Ra/sec-mech)]
    params.mech.friction = 6.0E-3f; // [kg.m^2/sec^2]=[N.m]

    // Filter Parameters:
    params.filt.spd_ar_en = En; // [#]
    params.filt.spd_ar_wp[0] = HZ_TO_RADSEC(75.0f); // [Ra/sec]
    params.filt.spd_ar_wp[1] = HZ_TO_RADSEC(75.0f); // [Ra/sec]
    params.filt.spd_ar_wz[0] = HZ_TO_RADSEC(1500.0f); // [Ra/sec]
    params.filt.spd_ar_wz[1] = HZ_TO_RADSEC(1500.0f); // [Ra/sec]
    params.filt.acc_w0 = HZ_TO_RADSEC(1.0f); // [Ra/sec]
    params.filt.trq_w0 = HZ_TO_RADSEC(1.0f); // [Ra/sec]

    // Control Mode:
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    params.ctrl.mode = Speed_Mode_FOC_Sensorless_Volt_Startup; // [#]
#elif defined(CTRL_METHOD_TBC)
    params.ctrl.mode = Speed_Mode_Block_Comm_Hall; // [#]
#endif

    // Speed Control Parameters:
    params.ctrl.speed.bw = HZ_TO_RADSEC(15.0f); // [Ra/sec], at least 0.5 dec below field weakening loop's bandwidth
    params.ctrl.speed.ki_multiple = 10.0f; // [#]
    params.ctrl.speed.ol_cl_tr_coeff = 1.0f; // [%]

    // Current Control Parameters:
#if defined(CTRL_METHOD_TBC)    
    params.ctrl.curr.bypass = false;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    params.ctrl.curr.bw = HZ_TO_RADSEC(750.0f); // [Ra/sec], at least one dec below switching frequency
    params.ctrl.curr.ff_coef = 1.0f; // [#]
    params.ctrl.curr.i_cmd_thresh = 0.15f * params.motor.i_cont; // [A]
    params.ctrl.curr.i_cmd_hyst = params.ctrl.curr.i_cmd_thresh * 0.80f; // [A]

#elif defined(CTRL_METHOD_SFO)
    // Torque Controller Parameters:
    params.ctrl.trq.w_p = HZ_TO_RADSEC(150.0f); // [Ra/sec]
    params.ctrl.trq.w_ratio = 0.25f; // [%], = w_p/w_z, w_p<w_z
    params.ctrl.trq.delta_max.elec = DEG_TO_RAD(120.0f); // [Ra-elec]
    params.ctrl.trq.T_cmd_thresh = 0.20f * params.motor.T_max; // [Nm]
    params.ctrl.trq.T_cmd_hyst = 0.9f * params.ctrl.trq.T_cmd_thresh; // [Nm]
    params.ctrl.trq.curr_lmt_t_reach = 0.040f; // [sec]

    // Flux Controller Parameters:
    params.ctrl.flux.bw = HZ_TO_RADSEC(300.0f); // [Ra/sec]
    params.ctrl.flux.pole_sep = 1.1f; // [#]

    // Load Angle Controller Parameters:
    params.ctrl.delta.bw = HZ_TO_RADSEC(750.0f); // [Ra/sec]
    params.ctrl.delta.bw_mult = 1.5f; // [#]
    params.ctrl.delta.bw_mult_wl.elec = HZ_TO_RADSEC(150.0f); // [Ra/sec-elec]
    params.ctrl.delta.bw_mult_wh.elec = HZ_TO_RADSEC(300.0f); // [Ra/sec-elec]
    params.ctrl.delta.pole_sep = 1.01f; // [#]
#endif

    // Voltage Control Parameters:
    params.ctrl.volt.w_thresh.elec = params.motor.w_nom.elec * 0.05f; // [Ra/sec-elec], min speed for voltage control
    params.ctrl.volt.w_hyst.elec = params.ctrl.volt.w_thresh.elec * 0.5f; // [Ra/sec-elec]
    params.ctrl.volt.v_min = 0.15f; // [Vpk]
    params.ctrl.volt.v_to_f_ratio = 7.5E-3f; // [Vpk/(Ra/sec-elec)]
    params.ctrl.volt.mod_method = Neutral_Point_Modulation;
    params.ctrl.volt.five_seg.en = Dis; //5-segment enable
    params.ctrl.volt.five_seg.active_mi = 0.1f; //mi = Vpeak/(2*vdc/3)
    params.ctrl.volt.five_seg.inactive_mi = 0.05f;
    params.ctrl.volt.five_seg.w0_filt = HZ_TO_RADSEC(1.0f);

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

    // Flux Weakenning Parameters:
    params.ctrl.flux_weaken.en = En; // [#]
    params.ctrl.flux_weaken.vdc_coeff = LINE_TO_PHASE(0.80f); // [#]
#if defined(CTRL_METHOD_RFO)
    params.ctrl.flux_weaken.bw = HZ_TO_RADSEC(3.0f); // [Ra/sec], at least 0.5 dec below current loop's bandwidth
#endif

    // Align (Pre-Positioning) Parameters:
    params.ctrl.align.time = 1.0f; // [sec]
    params.ctrl.align.voltage = 0.5f * (params.motor.r * params.motor.i_cont); // [Vpk]

    // Six Pulse Injection Parameters:
    params.ctrl.six_pulse_inj.i_peak = 3.50f; // [A], this is the target value, not the actual value

    // High Frequency Injection Parameters:
    params.ctrl.high_freq_inj.w_h = HZ_TO_RADSEC(1500.0f); // [Ra/sec], at least 1 decade below switching frequency
    params.ctrl.high_freq_inj.w_sep = HZ_TO_RADSEC(150.0f); // [Ra/sec], at least 1 decade below switching frequency
    params.ctrl.high_freq_inj.pll.w0 = HZ_TO_RADSEC(60.0f); // [Ra/sec]
    params.ctrl.high_freq_inj.pll.w_max.elec = 0.5f * (params.ctrl.high_freq_inj.w_h - params.ctrl.high_freq_inj.w_sep); // [Ra/sec-elec]
    params.ctrl.high_freq_inj.i_qd_r_peak.d = 1.75f; // [A], excitation current peak (d-axis), limiting factor (larger than q-axis)

    // Profiler Parameters (needs high fs0 & fpwm e.g. fs0=30kHz, fpwm=60kHz):
    params.profiler.overwrite = false; // []
    params.profiler.cmd_thresh = 0.050f; // [%]
    params.profiler.cmd_hyst = 0.025f; // [%]
    params.profiler.i_cmd_dc = params.motor.i_cont * 0.50f; // [A]
    params.profiler.i_cmd_ac = params.motor.i_cont * 0.25f; // [A]
    params.profiler.w_cmd_elec.min = 2.0f * params.obs.w_thresh.elec; // [Ra/sec-elec], must be well above params.obs.w_thresh.elec
    params.profiler.w_cmd_elec.max = 0.75f * params.motor.w_nom.elec; // [Ra/sec-elec]
    params.profiler.time_rot_lock = 1.0f; // [sec]
    params.profiler.time_spd = 1.5f; // [sec]

#elif defined(CTRL_METHOD_TBC)
    params.ctrl.tbc.mode = Block_Commutation;
    params.ctrl.tbc.trap.ramp_cnt = 5U; // [#]
    params.ctrl.tbc.trap.ramp_main_bw_ratio = 0.8f; // [#]
    params.ctrl.tbc.trap.ramp_ff_coef = 4.0f; // [V/A]
    params.ctrl.tbc.trap.main_ff_coef = 1.0f; // [#]

#endif

}

void PARAMS_InitAutoCalc()
{
    // Motor Parameters:
    params.motor.zeta = MAX(params.motor.lq / params.motor.ld, 1.01f); // [#] limited to avoid division by zero when calculating 1/(zeta-1)
#if defined(CTRL_METHOD_SFO)
    params.motor.mtpa_lut.x_min = 0.0f;	// [Nm]
    params.motor.mtpa_lut.x_max = params.motor.T_max; // [Nm]
    params.motor.mtpa_lut.x_step = (params.motor.mtpa_lut.x_max - params.motor.mtpa_lut.x_min) / (LUT_1D_WIDTH - 1); // [Nm]
    params.motor.mtpa_lut.x_step_inv = 1.0f / params.motor.mtpa_lut.x_step; // [1/Nm]
    float c1, c2, c3, c4, lad, laq_sq;
    c1 = 0.5f * POW_TWO(params.motor.ld / ((params.motor.zeta - 1.0f) * 0.75f * params.motor.P) * params.motor.mtpa_lut.x_step);
    c2 = params.motor.zeta / (params.motor.zeta - 1.0f) * params.motor.lam;
    c3 = (params.motor.zeta - 0.75f) / (params.motor.zeta - 1.0f) * params.motor.lam;
    c4 = POW_TWO(params.motor.zeta);
    lad = params.motor.lam; // [Wb]	
    laq_sq = 0.0f; // [Wb^2]
    params.motor.mtpa_lut.y[0] = params.motor.lam; // [Wb]
    for (uint32_t index = 1U; index < LUT_1D_WIDTH; ++index)
    {
        lad += c1 / POW_TWO(lad - c2) / (lad - c3) * (index - 0.5f); // [Wb]
        laq_sq = c4 * (lad - params.motor.lam) * (lad - c2); // [Wb^2]
        params.motor.mtpa_lut.y[index] = sqrtf(POW_TWO(lad) + laq_sq); // [Wb]
    }
    params.motor.mtpv_lut.x_min = 0.0f; // [Wb]
    params.motor.mtpv_lut.x_max = params.motor.mtpa_lut.y[LUT_1D_WIDTH - 1]; // [Wb], =la_max
    params.motor.mtpv_lut.x_step = (params.motor.mtpv_lut.x_max - params.motor.mtpv_lut.x_min) / (LUT_1D_WIDTH - 1); // [Wb]
    params.motor.mtpv_lut.x_step_inv = 1.0f / params.motor.mtpv_lut.x_step; // [1/Wb]
    float cos, sin;
    c1 = params.motor.mtpv_margin * 0.75f * params.motor.P * POW_TWO(params.motor.mtpv_lut.x_step) * (params.motor.zeta - 1.0f) / (params.motor.ld * params.motor.zeta);
    c2 = 0.25f * params.motor.lam * params.motor.zeta / (params.motor.mtpv_lut.x_step * (params.motor.zeta - 1.0f));
    params.motor.mtpv_lut.y[0] = 0.0f; // [Nm]
    for (uint32_t index = 1U; index < LUT_1D_WIDTH; ++index)
    {
        c3 = c2 / index;
        cos = c3 - sqrtf(POW_TWO(c3) + 0.5f);
        sin = sqrtf(1.0f - POW_TWO(cos));
        params.motor.mtpv_lut.y[index] = c1 * POW_TWO(index) * sin * (4.0f * c3 - cos);
    }

#endif

    // System Parameters:
    // Sampling Parameters:
    params.sys.samp.ts0 = 1.0f / params.sys.samp.fs0; // [sec]
    params.sys.samp.fpwm = params.sys.samp.fs0 * params.sys.samp.fpwm_fs0_ratio; // [Hz]
    params.sys.samp.tpwm = 1.0f / params.sys.samp.fpwm; // [Hz]
    params.sys.samp.fs1 = params.sys.samp.fs0 / params.sys.samp.fs0_fs1_ratio; // [Hz]
    params.sys.samp.ts1 = 1.0f / params.sys.samp.fs1; // [sec]
#if defined(PC_TEST)
    params.sys.samp.fsim = params.sys.samp.fs0 * params.sys.samp.fsim_fs0_ratio; // [Hz]
    params.sys.samp.tsim = 1.0f / params.sys.samp.fsim; // [sec]
#endif
    params.sys.analog.shunt.hyb_mod.adc_d_min = 2.0f * params.sys.analog.shunt.hyb_mod.adc_t_min * params.sys.samp.fpwm; // [%]

    // LUT Parameters:
    params.sys.lut.sin.th_step = PI_OVER_TWO / (TRIG_LUT_WIDTH - 1); // [Ra]
    params.sys.lut.sin.th_step_inv = 1.0f / params.sys.lut.sin.th_step; // [1/Ra]
    for (uint32_t index = 0; index < TRIG_LUT_WIDTH; ++index)
    {
        params.sys.lut.sin.val[index] = sinf(index * params.sys.lut.sin.th_step); // [#]
    }
    params.sys.lut.atan.step = 1.0f / (INV_TRIG_LUT_WIDTH - 1); // [#]
    params.sys.lut.atan.step_inv = 1.0f / params.sys.lut.atan.step; // [#]
    params.sys.lut.asin.step = params.sys.lut.atan.step; // [#]
    params.sys.lut.asin.step_inv = params.sys.lut.atan.step_inv; // [#]
    for (uint32_t index = 0; index < INV_TRIG_LUT_WIDTH; ++index)
    {
        params.sys.lut.atan.val[index] = atanf(index * params.sys.lut.atan.step); // [Ra]
        params.sys.lut.asin.val[index] = asinf(index * params.sys.lut.asin.step); // [Ra]
    }

    // Fault Parameters:
    if(!params.autocal_disable.vdc_fault_threshold) /*Skip the calculation if this bit is set*/
    {
      params.sys.faults.vdc_thresh.min = params.sys.vdc_nom * 0.7f; // [V]
      params.sys.faults.vdc_thresh.max = params.sys.vdc_nom * 1.3f; // [V]
    }
    if(!params.autocal_disable.speed_fault_threshold)/*Skip the calculation if this bit is set*/
    {
      params.sys.faults.w_thresh.elec = params.motor.w_max.elec * 2.0f; // [Ra/sec-elec]
    }
    switch (params.ctrl.mode)
    {
    case Volt_Mode_Open_Loop:
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    case Speed_Mode_FOC_Sensorless_Align_Startup:
    case Speed_Mode_FOC_Sensorless_SixPulse_Startup:
    case Speed_Mode_FOC_Sensorless_HighFreq_Startup:
    case Speed_Mode_FOC_Sensorless_Volt_Startup:
#endif
#if defined(CTRL_METHOD_RFO)
    case Speed_Mode_FOC_Hall:
#elif defined(CTRL_METHOD_TBC)
    case Speed_Mode_Block_Comm_Hall:
#endif
        params.sys.faults.cmd_clr_thresh = (params.ctrl.volt.w_thresh.elec - params.ctrl.volt.w_hyst.elec) / MECH_TO_ELEC(params.sys.cmd.w_max.mech, params.motor.P);
        break;
#if defined(CTRL_METHOD_RFO)
    case Curr_Mode_FOC_Sensorless_Align_Startup:
    case Curr_Mode_FOC_Sensorless_SixPulse_Startup:
    case Curr_Mode_FOC_Sensorless_HighFreq_Startup:
    case Curr_Mode_FOC_Sensorless_Dyno:
    case Curr_Mode_FOC_Hall:
#elif defined(CTRL_METHOD_TBC)
    case Curr_Mode_Block_Comm_Hall:
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
        params.sys.faults.cmd_clr_thresh = (params.ctrl.curr.i_cmd_thresh - params.ctrl.curr.i_cmd_hyst) / params.sys.cmd.i_max;
        break;
#endif
#if defined(CTRL_METHOD_SFO)
    case Trq_Mode_FOC_Sensorless_Align_Startup:
    case Trq_Mode_FOC_Sensorless_SixPulse_Startup:
    case Trq_Mode_FOC_Sensorless_HighFreq_Startup:
    case Trq_Mode_FOC_Sensorless_Dyno:
        params.sys.faults.cmd_clr_thresh = (params.ctrl.trq.T_cmd_thresh - params.ctrl.trq.T_cmd_hyst) / params.sys.cmd.T_max;
        break;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    case Profiler_Mode:
        params.sys.faults.cmd_clr_thresh = params.profiler.cmd_thresh - params.profiler.cmd_hyst;
        break;
#endif
    default:
        params.sys.faults.cmd_clr_thresh = 0.0f;
        break;
    }

    params.sys.fb.mode = Sensorless;
#if defined(CTRL_METHOD_RFO)
    if ((params.ctrl.mode == Speed_Mode_FOC_Hall) || (params.ctrl.mode == Curr_Mode_FOC_Hall))
    {
        params.sys.fb.mode = Hall;
    }
    else if ((params.ctrl.mode == Speed_Mode_FOC_Encoder_Align_Startup) || (params.ctrl.mode == Curr_Mode_FOC_Encoder_Align_Startup))
    {
        params.sys.fb.mode = AqB_Enc;
    }
#elif defined(CTRL_METHOD_TBC)
    if ((params.ctrl.mode == Speed_Mode_Block_Comm_Hall) || (params.ctrl.mode == Curr_Mode_Block_Comm_Hall))
    {
        params.sys.fb.mode = Hall;
    }
#endif
    params.sys.fb.hall.track_loop.cpr = (uint16_t)(params.motor.P) * 3U; // [#], always 6*(P/2) for hall sensor

    // Observer Parameters:
    params.obs.flux_filt.c1_coeff = params.obs.flux_filt.k1 + params.obs.flux_filt.k2 + params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.c2_coeff = params.obs.flux_filt.k1 * params.obs.flux_filt.k2 + \
        params.obs.flux_filt.k2 * params.obs.flux_filt.k3 + \
        params.obs.flux_filt.k1 * params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.c3_coeff = params.obs.flux_filt.k1 * params.obs.flux_filt.k2 * params.obs.flux_filt.k3; // [#]
    params.obs.flux_filt.gain = sqrtf(POW_TWO(1.0f - params.obs.flux_filt.c2_coeff) + POW_TWO(params.obs.flux_filt.c3_coeff - params.obs.flux_filt.c1_coeff)); // [#]
    params.obs.flux_filt.th_p.elec = ATan2(params.obs.flux_filt.c1_coeff - params.obs.flux_filt.c3_coeff, 1.0f - params.obs.flux_filt.c2_coeff); // [Ra-elec]
    ParkInit(-params.obs.flux_filt.th_p.elec, &params.obs.flux_filt.phase_lead);

    if(!params.autocal_disable.observer_pll) /*Skip the calculation if this bit is set*/
    {
      params.obs.pll.kp = params.obs.pll.w0 / params.motor.lam; // [(Ra/sec-elec)/Wb]
      params.obs.pll.ki = POW_TWO(params.obs.pll.w0) / params.motor.lam * 0.5f; // [(Ra/sec).(Ra/sec-elec)/Wb]
    }

    params.obs.pll.th_offset.elec = params.obs.flux_filt.th_p.elec; // [Ra-elec]

    // Mechanical System Parameters:

    // Filter Parameters:

    // Speed Control Parameters:
#if defined(CTRL_METHOD_RFO)
    if(!params.autocal_disable.speed_control) /*Skip the calculation if this bit is set*/
    {
      params.ctrl.speed.kp = ((8.0f / 3.0f) / (POW_TWO(params.motor.P) * params.motor.lam)) * params.mech.inertia * params.ctrl.speed.bw; // [A/(Ra/sec-elec)]
      params.ctrl.speed.ki = ((8.0f / 3.0f) / (POW_TWO(params.motor.P) * params.motor.lam)) * params.mech.viscous * params.ctrl.speed.bw * params.ctrl.speed.ki_multiple; // [A/(Ra/sec-elec).(Ra/sec)]
    }
    params.ctrl.speed.ff_k_inertia = ((8.0f / 3.0f) / (POW_TWO(params.motor.P) * params.motor.lam)) * params.mech.inertia; // [A/(Ra/sec-elec).sec]
    params.ctrl.speed.ff_k_viscous = ((8.0f / 3.0f) / (POW_TWO(params.motor.P) * params.motor.lam)) * params.mech.viscous; // [A/(Ra/sec-elec)]
    params.ctrl.speed.ff_k_friction = ((4.0f / 3.0f) / (params.motor.P * params.motor.lam)) * params.mech.friction; // [A]

#elif defined(CTRL_METHOD_SFO)
    if(!params.autocal_disable.speed_control) /*Skip the calculation if this bit is set*/
    {
      params.ctrl.speed.kp = 2.0f / params.motor.P * params.mech.inertia * params.ctrl.speed.bw; // [Nm/(Ra/sec-elec)]
      params.ctrl.speed.ki = 2.0f / params.motor.P * params.mech.viscous * params.ctrl.speed.bw * params.ctrl.speed.ki_multiple; // [Nm/(Ra/sec-elec).(Ra/sec)]
    }
    params.ctrl.speed.ff_k_inertia = 2.0f / params.motor.P * params.mech.inertia; // [Nm/(Ra/sec-elec).sec]
    params.ctrl.speed.ff_k_viscous = 2.0f / params.motor.P * params.mech.viscous; // [Nm/(Ra/sec-elec)]
    params.ctrl.speed.ff_k_friction = params.mech.friction; // [Nm]

#elif defined(CTRL_METHOD_TBC)
    if(!params.autocal_disable.speed_control) /*Skip the calculation if this bit is set*/
    {
      params.ctrl.speed.kp = ((2.0f * SQRT_TWO) / (POW_TWO(params.motor.P) * params.motor.lam)) * params.mech.inertia * params.ctrl.speed.bw; // [A/(Ra/sec-elec)]
      params.ctrl.speed.ki = ((2.0f * SQRT_TWO) / (POW_TWO(params.motor.P) * params.motor.lam)) * params.mech.viscous * params.ctrl.speed.bw * params.ctrl.speed.ki_multiple; // [A/(Ra/sec-elec).(Ra/sec)]
    }
    params.ctrl.speed.ff_k_inertia = ((2.0f * SQRT_TWO) / (POW_TWO(params.motor.P) * params.motor.lam)) * params.mech.inertia; // [A/(Ra/sec-elec).sec]
    params.ctrl.speed.ff_k_viscous = ((2.0f * SQRT_TWO) / (POW_TWO(params.motor.P) * params.motor.lam)) * params.mech.viscous; // [A/(Ra/sec-elec)]
    params.ctrl.speed.ff_k_friction = (SQRT_TWO / (params.motor.P * params.motor.lam)) * params.mech.friction; // [A]

#endif

#if defined(CTRL_METHOD_RFO)
    // Current Control Parameters:
    if(!params.autocal_disable.current_control) /*Execute calculation if this bit is not set*/
    {
      params.ctrl.curr.kp.q = params.ctrl.curr.bw * params.motor.lq; // [V/A]
      params.ctrl.curr.kp.d = params.ctrl.curr.bw * params.motor.ld; // [V/A]
      params.ctrl.curr.ki.q = params.ctrl.curr.bw * params.motor.r; // [V/A.(Ra/sec)]
      params.ctrl.curr.ki.d = params.ctrl.curr.bw * params.motor.r; // [V/A.(Ra/sec)]
    }
    params.ctrl.curr.v_max.q = LINE_TO_PHASE(params.sys.vdc_nom); // [V]
    params.ctrl.curr.v_max.d = LINE_TO_PHASE(params.sys.vdc_nom); // [V]

#elif defined(CTRL_METHOD_TBC)
    params.ctrl.curr.k_bypass = params.motor.r * ONE_OVER_SQRT_TWO; // [V/A]
    if(!params.autocal_disable.current_control) /*Execute calculation if this bit is not set*/
    {
      params.ctrl.curr.kp = params.ctrl.curr.bw * (0.5f * ONE_OVER_SQRT_TWO) * (params.motor.lq + params.motor.ld); // [V/A]
      params.ctrl.curr.ki = params.ctrl.curr.bw * ONE_OVER_SQRT_TWO * params.motor.r; // [V/A.(Ra/sec)]
    }
    params.ctrl.curr.v_max = 0.5f * params.sys.vdc_nom; // [V]

#elif defined(CTRL_METHOD_SFO)
    // Torque Controller Parameters:
    float k = 0.75f * params.motor.P * params.motor.lam * (params.motor.lam / params.motor.ld + (1.0f / params.motor.lq - 1.0f / params.motor.ld) * params.motor.lam);
    if(!params.autocal_disable.torque_control) /*Execute calculation if this bit is not set*/
    {
      params.ctrl.trq.kp = params.ctrl.trq.w_ratio / ((1.0f - params.ctrl.trq.w_ratio) * k); // [(Ra-elec)/(Nm)]
      params.ctrl.trq.ki = params.ctrl.trq.w_p / ((1.0f - params.ctrl.trq.w_ratio) * k); // [(Ra/sec).(Ra-elec)/(Nm)]
    }
    float dl = MIN(ABS(params.motor.lq - params.motor.ld), 1.0E-9f);
    c1 = 1.0f / (0.75f * params.motor.P * params.motor.lam);
    c2 = 1.0f / sqrtf(0.375f * params.motor.P * dl * params.motor.T_max);
    float c_min = MIN(c1, c2);
    params.ctrl.trq.curr_lmt_ki = params.sys.samp.ts0 * (params.motor.i_peak - params.motor.i_cont) / (c_min * params.ctrl.trq.curr_lmt_t_reach);

    // Flux Controller Parameters:
    if(!params.autocal_disable.flux_control) /*Execute calculation if this bit is not set*/
    {
      params.ctrl.flux.kp = params.ctrl.flux.bw * params.ctrl.flux.pole_sep; // [V/Wb]
      params.ctrl.flux.ki = POW_TWO(params.ctrl.flux.bw) * (params.ctrl.flux.pole_sep - 1.0f); // [(Ra/sec).V/Wb]
    }
    params.ctrl.flux.vd_max = LINE_TO_PHASE(params.sys.vdc_nom); // [V]

    // Load Angle Controller Parameters:
    params.ctrl.delta.bw_mult_slope = 1.0f / (params.ctrl.delta.bw_mult_wh.elec - params.ctrl.delta.bw_mult_wl.elec); // [1/(Ra/sec-elec)]
    params.ctrl.delta.bw_mult_inter = -params.ctrl.delta.bw_mult_wl.elec / (params.ctrl.delta.bw_mult_wh.elec - params.ctrl.delta.bw_mult_wl.elec); // [#]
    params.ctrl.delta.vq_max = LINE_TO_PHASE(params.sys.vdc_nom); // [V]

#endif

    // Voltage Control Parameters:

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

    // Flux Weakenning Parameters:
#if defined(CTRL_METHOD_RFO)
    if(!params.autocal_disable.flux_weakening) /*Execute calculation if this bit is not set*/
    {
      params.ctrl.flux_weaken.ki = params.motor.lam / (params.motor.ld * LINE_TO_PHASE(params.sys.vdc_nom)) * params.ctrl.flux_weaken.bw * params.sys.samp.ts0; // % [Ra/sec.A/V]
    }
#elif defined(CTRL_METHOD_SFO)
    params.ctrl.flux_weaken.w_min.elec = 0.5f * params.sys.vdc_nom * params.ctrl.flux_weaken.vdc_coeff / params.motor.mtpa_lut.y[LUT_1D_WIDTH - 1]; // [Ra/sec-elec]
#endif

    // Six Pulse Injection Parameters:
    float motor_l_ave = 0.5f * (params.motor.lq + params.motor.ld); // [H]
    float motor_tau_ave = motor_l_ave / params.motor.r; // [sec]
    params.ctrl.six_pulse_inj.t_on = QUANTIZE_FLOAT(motor_tau_ave, params.sys.samp.ts0); // [sec], to maximize the difference between currents
    params.ctrl.six_pulse_inj.t_off = QUANTIZE_FLOAT(10.0f * motor_tau_ave, params.sys.samp.ts0); // [sec], enough time for current to go to zero
    params.ctrl.six_pulse_inj.v_pulse = (1.5f / (1.0f - EXP_MINUS_ONE)) * params.motor.r * params.ctrl.six_pulse_inj.i_peak; // [V], dc+ to dc-

    // High Frequency Injection Parameters:
    params.ctrl.high_freq_inj.v_qd_r_coeff.q = params.ctrl.high_freq_inj.i_qd_r_peak.d * params.motor.ld; // [V/(Ra/sec)]
    params.ctrl.high_freq_inj.v_qd_r_coeff.d = params.ctrl.high_freq_inj.i_qd_r_peak.d * params.motor.ld * params.ctrl.high_freq_inj.w_h; // [V]

    params.ctrl.high_freq_inj.i_qd_r_peak.q = params.ctrl.high_freq_inj.i_qd_r_peak.d *
        0.5f * (params.motor.zeta - 1.0f) / params.motor.zeta; //[A], excitation current peak (q-axis), smaller than (d-axis)

    params.ctrl.high_freq_inj.pll.kp = params.ctrl.high_freq_inj.pll.w0 / params.ctrl.high_freq_inj.i_qd_r_peak.q; // [(Ra/sec-elec)/A]
    params.ctrl.high_freq_inj.pll.ki = 0.25f * POW_TWO(params.ctrl.high_freq_inj.pll.w0) / params.ctrl.high_freq_inj.i_qd_r_peak.q; // [(Ra/sec).(Ra/sec-elec)/A]
    params.ctrl.high_freq_inj.pll.th_offset.elec = 0.0f;

    params.ctrl.high_freq_inj.lpf_biquad_a[0] = 1.0f; // [#], dc gain
    params.ctrl.high_freq_inj.lpf_biquad_a[1] = 0.0f; // [1/(Ra/sec)]
    params.ctrl.high_freq_inj.lpf_biquad_a[2] = 0.0f; // [1/(Ra/sec)^2]
    params.ctrl.high_freq_inj.lpf_biquad_b[0] = 1.0f; // [#]
    params.ctrl.high_freq_inj.lpf_biquad_b[1] = 2.0f / params.ctrl.high_freq_inj.w_sep; // [1/(Ra/sec)]
    params.ctrl.high_freq_inj.lpf_biquad_b[2] = 1.0f / POW_TWO(params.ctrl.high_freq_inj.w_sep); // [1/(Ra/sec)^2]
#if defined(CTRL_METHOD_RFO)
    params.ctrl.high_freq_inj.bw_red_coeff = params.ctrl.high_freq_inj.w_sep / params.ctrl.curr.bw;
#elif defined(CTRL_METHOD_SFO)
    params.ctrl.high_freq_inj.bw_red_coeff = params.ctrl.high_freq_inj.w_sep / MAX(params.ctrl.flux.bw, params.ctrl.delta.bw);
#endif
    params.ctrl.high_freq_inj.lock_time = RADSEC_TO_TAU(params.ctrl.high_freq_inj.pll.w0) * 40.0f; // [sec]

    // Profiler Parameters:
    // Ranges:
    // r:        10m -> 700m   ,[Ohm]
    // l:        10u -> 700u   ,[H]
    // f (=r/l): 500 -> 1000   ,[Hz]
    const MINMAX_t Range_R = { 10.0E-3f, 700.0E-3f }; // [Ohm]
    const MINMAX_t Range_F = { 900.0f, 1100.0f }; // [Hz], excitation

    const float Slope_F = (Range_F.max - Range_F.min) / (PROF_FREQ_POINTS - 1);
    for (uint8_t index = 0U; index < PROF_FREQ_POINTS; ++index)
    {
        params.profiler.w_h[index] = HZ_TO_RADSEC(Range_F.min + Slope_F * index); // [Ra/sec]
    }
    params.profiler.w_sep = HZ_TO_RADSEC(Range_F.min * 0.1f); // [Ra/sec]
    params.profiler.w0_idc = params.profiler.w_sep * 0.5f; // [Ra/sec]
    params.profiler.kp_idc = params.profiler.w0_idc / params.profiler.w_sep * AVE(Range_R.min, Range_R.max); // [V/A]
    params.profiler.ki_idc = params.profiler.w0_idc * AVE(Range_R.min, Range_R.max); // [(V/A).(Ra/sec)]
    params.profiler.time_res = 80.0f * RADSEC_TO_TAU(params.profiler.w_sep); // [sec]
    params.profiler.time_ind = 80.0f * RADSEC_TO_TAU(params.profiler.w_sep); // [sec]
    params.profiler.w0_flux = TAU_TO_RADSEC(params.profiler.time_spd * 0.1f);

#elif defined(CTRL_METHOD_TBC)
    params.ctrl.tbc.trap.ramp_kp = params.ctrl.curr.kp * params.ctrl.tbc.trap.ramp_main_bw_ratio; // [V/A]
    params.ctrl.tbc.trap.ramp_ki = params.ctrl.curr.ki * params.ctrl.tbc.trap.ramp_main_bw_ratio; // [(V/A).(Ra/sec)]
    params.ctrl.tbc.trap.ramp_cnt_inv = (params.ctrl.tbc.trap.ramp_cnt == 0U) ? 1.0f : 1.0f / (float)(params.ctrl.tbc.trap.ramp_cnt); // [#]

#endif

}
