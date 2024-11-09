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

#pragma once

// Latest firmware version, 0xAABC==vAA.B.C, Example: 0x0150==v1.5.0.
#define FIRMWARE_VER			(0x0180UL)
#define PARAMS_CODE				(~0xBADC0DEUL)	// Do not change this code
#define PARAMS_VER				(0x0003UL)		// Parameters version. Change when params struct changes.
#define PARAMS_ALWAYS_OVERWRITE     (false)     // For testing only. Using this will ensure that parameters are always overwritten.
#if defined(CTRL_METHOD_RFO)
#define BUILD_CONFIG_ID			(0x0000UL)		// Indicating RFO build config
#elif defined(CTRL_METHOD_SFO)
#define BUILD_CONFIG_ID			(0x0001UL)		// Indicating SFO build config
#elif defined(CTRL_METHOD_TBC)
#define BUILD_CONFIG_ID			(0x0002UL)		// Indicating TBC build config
#endif

#include "General.h"

typedef struct
{
    float therm_tau;			// [sec], thermal time constant
    float on_level;				// [%], percentage of motor's thermal capacity
    float off_level;			// [%], percentage of motor's thermal capacity
} I2T_PARAMS_t;

typedef struct
{
    float P;				// [#], pole pairs
    float lq;				// [H], q-axis inductance
    float ld;				// [H], d-axis inductance
    float lam;				// [Wb], permanent magnet's flux linkage
    float r;				// [ohm], stator resistance
    float T_max;			// [Nm], maximum torque
    float i_peak;			// [A], peak current rating
    float i_cont;			// [A], continuous current rating
    float id_max;			// [A], maximum demagnetization current
    float v_nom;			// [Vpk-ln], nominal voltage peak, line to neutral
    ELEC_t w_nom;			// [Ra/sec-elec], nominal speed
    ELEC_t w_max;			// [Ra/sec-elec], maximum speed
    float zeta;				// [#], saliency ratio = lq/ld
#if defined(CTRL_METHOD_SFO)
    float mtpv_margin;		// [%], mtpv margin (applied on maximum torque)
    LUT_1D_t mtpa_lut;		// [Nm, Wb], MTPA look up table for flux command based on torque command
    LUT_1D_t mtpv_lut;		// [Wb, Nm], MTPV look up table for max torque based on flux command
#endif
    I2T_PARAMS_t i2t;
} MOTOR_PARAMS_t;

typedef struct
{
    float fpwm;						// [Hz], pwm frequency
    float tpwm;						// [sec], pwm period
    uint32_t fpwm_fs0_ratio;		// [], ratio of pwm to isr0 freqeuncy
    float fs0;						// [Hz], sampling frequency, isr0
    float ts0;						// [sec], sampling time, isr0
    uint32_t fs0_fs1_ratio;			// [], ratio of isr0 to isr1 frequency
    float fs1;						// [Hz], sampling frequency, isr1
    float ts1;						// [sec], sampling time, isr1
#if defined(PC_TEST)
    uint32_t fsim_fs0_ratio;		// [], ratio of sim to isr0 frequency
    float fsim;						// [Hz], sampling frequency, simulations
    float tsim;						// [sec], sampling time, simulations
#endif
} SAMPLE_PARAMS_t;

typedef struct
{
    TRIG_LUT_t sin;                 // [#], sin lut for park transforms
    INV_TRIG_LUT_t atan;            // [#], atan lut for control
    INV_TRIG_LUT_t asin;            // [#], asin lut for control
} LUT_PARAMS_t;

typedef struct
{
    // output = input*gain+offset
    // input and output must be already scaled by HAL to be in SI units
    float gain;			// [%]
    float offset;		// [SI unit]
} CALIB_PARAMS_t;

typedef struct
{
    CALIB_PARAMS_t i_u;
    CALIB_PARAMS_t i_v;
    CALIB_PARAMS_t i_w;
    CALIB_PARAMS_t v_uz;
    CALIB_PARAMS_t v_vz;
    CALIB_PARAMS_t v_wz;
    CALIB_PARAMS_t v_dc;
    CALIB_PARAMS_t temp_ps;	// power stage temperature
    CALIB_PARAMS_t pot;		// potentiometer
} ANALOG_CALIB_PARAMS_t;

typedef struct
{
    float w0_i_ph;		// [Ra/sec]
    float w0_v_ph;		// [Ra/sec]
    float w0_v_dc;		// [Ra/sec]
    float w0_temp_ps;	// [Ra/sec]
    float w0_pot;		// [Ra/sec]
} ANALOG_FILT_PARAMS_t;

typedef enum
{
    Three_Shunt = 0U,
    Single_Shunt
} SHUNT_TYPE_t;
#if defined(USING_SGD)
typedef enum
{
    Gain_4 = 0U,
    Gain_8 = 1U,
    Gain_12 = 2U,
    Gain_16 = 3U,
    Gain_20 = 4U,
    Gain_24 = 5U,
    Gain_32 = 6U,
    Gain_64 = 7U
} SHUNT_OPAMP_GAIN_t;
#endif
typedef struct
{
    float adc_t_min;    // [sec], minimum time for ADC sampling
    float adc_d_min;    // [%], minimum duty cycle for ADC sampling
    float ki;           // [#], error[n]/error[n-1]=1-ki
} SHUNT_HYB_MOD_t;

typedef struct
{
    SHUNT_TYPE_t type;
#if defined(USING_SGD)
    SHUNT_OPAMP_GAIN_t opamp_gain;
#else
    float opamp_gain;
#endif
    SHUNT_HYB_MOD_t hyb_mod;    // hybrid modulation, for single shunt
    float res;                  // [Ohm]
} ANALOG_SHUNT_PARAMS_t;

typedef struct
{
    ANALOG_CALIB_PARAMS_t calib;
    ANALOG_FILT_PARAMS_t filt;
    ANALOG_SHUNT_PARAMS_t shunt;
    float offset_null_time; // [sec], offset nulling time in init state
} ANALOG_SENS_PARAMS_t;

typedef struct
{
    ELEC_t w_cmd;				// [(Ra/sec-elec)/sec], limiting speed command slope
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    float i_cmd;				// [A/sec], limiting current command slope
#elif defined(CTRL_METHOD_SFO)
    float T_cmd;				// [Nm/sec], limiting torque command slope
#endif
} RATE_LIM_PARAMS_t;

typedef enum
{
    Low_Side_Short = 0U,
    High_Side_Short = 1U,
    Alternate_Short = 2U
} SHORT_METHOD_t;

typedef struct
{
    float oc_thresh;				// [%], oc threshold, percentage of motor's thermal capacity, works with motor i2t
    MINMAX_t vdc_thresh;			// [V], vdc fault thresholds
    float vdc_time;					// [sec], vdc fault detection time
    float temp_ps_thresh;			// [Celsius], power-stage over-temperature fault threshold
    ELEC_t w_thresh;				// [Ra/sec-elec], over-speed fault threshold
    SHORT_METHOD_t short_method;	// [], output short type
    float cmd_clr_thresh;			// [%], command threshold below which faults are cleared and the system restarts
    uint32_t max_clr_tries;			// [], maximum tries to clear faults and restart, before system permanently goes to the faulted state
    uint32_t watchdog_time;			// [ms], maximum time before watchdog resets the processor
} FAULT_PARAMS_t;

typedef enum
{
    Internal = 0,				    // From potentiometer
    External,						// From GUI, UART, etc.
} CMD_SOURCE_t;

typedef struct
{
    CMD_SOURCE_t source;
    // Full scale values corresponding to potentiometer reading of 100%
    MECH_t w_max;			// [Ra/sec-mech]
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    float i_max;			// [A]
#elif defined(CTRL_METHOD_SFO)
    float T_max;			// [Nm]
#endif
} CMD_PARAMS_t;

typedef struct
{
    uint16_t cpr;           // [#], counts per mech-revs, cpr=6*(P/2) for hall sensor, cpr=4*lpr for incremental encoder
    float w0_w;				// [Ra/sec], bandwidth of speed-feed-forward low pass filter
    MINMAX_t w0_th;			// [Ra/sec], limits of angle estimation loop's bandwidth
    float tau_ratio;		// [sec/sec], ratio of angle estimation loop's time constant to captured angle's period
} ADAP_TRACK_LOOP_PARAMS_t;

typedef struct
{
    ADAP_TRACK_LOOP_PARAMS_t track_loop;    // [], adaptive tracking loop parameters 
    ELEC_t w_zsd_thresh;    // [Ra/sec-elec], zero speed detection threshold

    EN_DIS_t block_comm_offset_comp;	    // [], enable/disable offset compensation for block commutation
    ELEC_t th_r_offset;     // [Ra-elec], based on construction of motor and location of hall sensors, normally zero
} HALL_SENS_PARAMS_t;


typedef struct
{
    ADAP_TRACK_LOOP_PARAMS_t track_loop;    // [], adaptive tracking loop parameters
    ELEC_t w_zsd_thresh;    // [Ra/sec-elec], zero speed detection threshold
} INC_ENCODER_PARAMS_t;

typedef struct
{
    FB_MODE_t mode;
    HALL_SENS_PARAMS_t hall;
    INC_ENCODER_PARAMS_t encoder;
} FB_PARAMS_t;

typedef struct
{
    SAMPLE_PARAMS_t samp;			// [], sampling parameters
    LUT_PARAMS_t lut;				// [#], luts
    ANALOG_SENS_PARAMS_t analog;	// [], analog sensor parameters
    RATE_LIM_PARAMS_t rate_lim;		// [], rate limiter parameters
    FAULT_PARAMS_t faults;			// [], fault parameters
    CMD_PARAMS_t cmd;				// [], command parameters
    FB_PARAMS_t fb;					// [#], feedback parameters
    float boot_time;				// [sec], time for charging bootstrap capactiors
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    float dyno_lock_time;			// [sec], minimum lock time in dyno mode
#endif
    float vdc_nom;					// [], dc bus voltage parameters
} SYS_PARAMS_t;

typedef struct
{
    // H(s)=s^2/(s^3+c1*s^2+c2*s+c3)*gain
    // c1=(k1+k2+k3)*|w0|
    // c2=(k1*k2+k1*k3+k2*k3)*|w0|^2
    // c3=k1*k2*k3*|w0|^3
    // th_p=atan(((k1+k2+k3)-(k1*k2*k3))/(1-(k1*k2+k1*k3+k2*k3)))
    // th_p'=th_p*sign(w0)
    float gain;				// [#], see definitions above
    float k1;				// [#], see definitions above
    float k2;				// [#], see definitions above
    float k3;				// [#], see definitions above
    float c1_coeff;			// =c1/|w0|=(k1+k2+k3)
    float c2_coeff;			// =c2/|w0|^2=(k1*k2+k1*k3+k2*k3)
    float c3_coeff;			// =c3/|w0|^3=(k1*k2*k3)
    ELEC_t th_p;			// [Ra-elec], phase shift of the filter
    PARK_t phase_lead;		// Park transform for phase shift compensation
} FLUX_FILT_PARAMS_t;

typedef struct
{
    float w0;			// [Ra/sec]
    float kp;			// [(Ra/sec-elec)/(Input Unit)]
    float ki;			// [(Ra/sec).(Ra/sec-elec)/(Input Unit)]
    ELEC_t w_max;		// [Ra/sec-elec]
    ELEC_t th_offset;	// [Ra-elec]
} PLL_PARAMS_t;

typedef struct
{
    FLUX_FILT_PARAMS_t flux_filt;
    float biquad_a[3];		// Angular frequency biquad filter coefficients
    float biquad_b[3];		// H(s) = (a0+a1*s+a2*s^2)/(b0+b1*s+b2*s^2)
    PLL_PARAMS_t pll;
    ELEC_t w_thresh;		// [Ra/sec-elec], min speed for observer
    ELEC_t w_hyst;			// [Ra/sec-elec], hysteresis for min speed
    float lock_time;		// [sec], waiting time to ensure pll locking above w_thresh
} OBS_PARAMS_t;

typedef struct
{
    float inertia;	// [kg.m^2]=[(N.m)/(Ra/sec-mech).sec], =1/2*m*r^2
    float viscous;	// [kg.m^2/sec]=[(N.m)/(Ra/sec-mech)]
    float friction; // [kg.m^2/sec^2]=[N.m]
} MECH_PARAMS_t;

typedef struct
{
    EN_DIS_t spd_ar_en;		// [#], enable or disable speed anti-resonant filter
    float spd_ar_wz[2];		// [Ra/sec], speed anti-resonant filter's zero locations
    float spd_ar_wp[2];		// [Ra/sec], speed anti-resonant filter's pole locations
    float acc_w0;			// [Ra/sec], cut off frequency for acceleration estimation filter
    float trq_w0;			// [Ra/sec], cut off frequency for torque estimation filter
} FILTER_PARAMS_t;

typedef enum
{
    Volt_Mode_Open_Loop = 0,					// Open-loop V/Hz control
#if defined(CTRL_METHOD_RFO)
    Curr_Mode_FOC_Sensorless_Align_Startup,		// Closed-loop sensorless-foc current control with pre-alignment at startup
    Curr_Mode_FOC_Sensorless_SixPulse_Startup,	// Closed-loop sensorless-foc current control with six pulse injection at startup
    Curr_Mode_FOC_Sensorless_HighFreq_Startup,	// Closed-loop sensorless-foc current control with high frequency injection at startup
    Curr_Mode_FOC_Sensorless_Dyno,				// Closed-loop sensorless-foc current control in dyno mode (waiting for observer lock to start up)
    Curr_Mode_FOC_Encoder_Align_Startup,        // Closed-loop sensored-foc current control with encoder feedback and pre-alignment at startup
    Curr_Mode_FOC_Hall,							// Closed-loop sensored-foc current control with hall sensor feedback
#elif defined(CTRL_METHOD_TBC)
    Curr_Mode_Block_Comm_Hall,					// Closed-loop block-commutation current control with hall sensor feedback
#elif defined(CTRL_METHOD_SFO)
    Trq_Mode_FOC_Sensorless_Align_Startup,		// Closed-loop sensorless-foc torque control with pre-alignment at startup
    Trq_Mode_FOC_Sensorless_SixPulse_Startup,	// Closed-loop sensorless-foc torque control with six pulse injection at startup
    Trq_Mode_FOC_Sensorless_HighFreq_Startup,	// Closed-loop sensorless-foc torque control with high frequency injection at startup
    Trq_Mode_FOC_Sensorless_Dyno,				// Closed-loop sensorless-foc torque control in dyno mode (waiting for observer lock to start up)
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    Speed_Mode_FOC_Sensorless_Align_Startup,	// Closed-loop sensorless-foc speed control with pre-alignment at startup
    Speed_Mode_FOC_Sensorless_SixPulse_Startup,	// Closed-loop sensorless-foc speed control with six pulse injection at startup
    Speed_Mode_FOC_Sensorless_HighFreq_Startup,	// Closed-loop sensorless-foc speed control high frequency injection at startup
    Speed_Mode_FOC_Sensorless_Volt_Startup,		// Closed-loop sensorless-foc speed control with open-loop V/Hz at startup
#endif
#if defined(CTRL_METHOD_RFO)
    Speed_Mode_FOC_Encoder_Align_Startup,       // Closed-loop sensored-foc speed control with encoder feedback and pre-alignment at startup
    Speed_Mode_FOC_Hall,						// Closed-loop sensored-foc speed control with hall sensor feedback
#elif defined(CTRL_METHOD_TBC)
    Speed_Mode_Block_Comm_Hall,					// Closed-loop block-commutation speed control with hall sensor feedback
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    Profiler_Mode,                              // Profiler mode
#endif
} CTRL_MODE_t;

typedef struct
{
    float bw;				// [Ra/sec], bandwidth
    float kp;				// [A/(Ra/sec-elec)] in RFO, [Nm/(Ra/sec-elec)] in SFO
    float ki;				// [A/(Ra/sec-elec).(Ra/sec)] in RFO, [Nm/(Ra/sec-elec).(Ra/sec)] in SFO
    float ki_multiple;      // [#], usually in [1-100] range, higher values increase the responsiveness of the speed loop but reduce the phase margin
    float ff_k_inertia;		// [A/(Ra/sec-elec).sec] in RFO, [Nm/(Ra/sec-elec).sec] in SFO, inertia feed forward coefficient
    float ff_k_viscous;		// [A/(Ra/sec-elec)] in RFO, [Nm/(Ra/sec-elec)] in SFO, viscous damping feed forward coefficient
    float ff_k_friction;	// [A] in RFO, [Nm] in SFO, friction feed forward coefficient
    float ol_cl_tr_coeff;	// [%] torque precalculation coefficient when transitioning from ol to cl
} SPEED_CTRL_PARAMS_t;


#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
typedef struct
{
    float bw;			// [Ra/sec], bandwidth
#if defined(CTRL_METHOD_RFO)
    QD_t kp;			// [V/A]
    QD_t ki;			// [(V/A).(Ra/sec)]
    QD_t v_max;			// [V]
#elif defined(CTRL_METHOD_TBC)
    float kp;           // [V/A]
    float ki;           // [(V/A).(Ra/sec)]
    float v_max;        // [V]
    bool bypass;        // []
    float k_bypass;     // [V/A]
#endif
    float ff_coef;		// [#], feed forward coefficient
    float i_cmd_thresh;	// [A], threshold for transition to current control
    float i_cmd_hyst;	// [A], hysteresis for i_cmd_thresh
} CURRENT_CTRL_PARAMS_t;

#elif defined(CTRL_METHOD_SFO)
typedef struct
{
    float w_p;				// [Ra/sec]
    float w_ratio;			// [%], = w_p/w_z, w_p<w_z
    float kp;				// [(Ra-elec)/(Nm)]
    float ki;				// [(Ra/sec).(Ra-elec)/(Nm)]
    ELEC_t delta_max;		// [Ra-elec], maximum load angle command
    float T_cmd_thresh;		// [Nm], threshold for transition from prepositioning to torque control
    float T_cmd_hyst;		// [Nm], hysteresis for T_cmd_thresh
    float curr_lmt_t_reach; // [sec], sliding-mode current limiter's worst-case reaching time
    float curr_lmt_ki;		// [Nm/A.Ra/sec], sliding-mode current limiter's ki
} TRQ_CTRL_PARAMS_t;

typedef struct
{
    float bw;				// [Ra/sec]
    float pole_sep;			// [#]
    float kp;				// [V/Wb]
    float ki;				// [(Ra/sec).(V/Wb)]
    float vd_max;			// [V], maximum d-axis voltage command
} FLUX_CTRL_PARAMS_t;

typedef struct
{
    float bw;				// [Ra/sec], bandwidth of load angle controller
    float bw_mult;			// [#], bandwidth multiplication factor
    ELEC_t bw_mult_wl;		// [Ra/sec-elec], low speed threshold for bandwidth multiplcation factor
    ELEC_t bw_mult_wh;		// [Ra/sec-elec], high speed threshold for bandwidth multiplcation factor
    float bw_mult_slope;	// [1/(Ra/sec-elec)], =+1/(bw_mult_wh-bw_mult_wl)
    float bw_mult_inter;	// [#], = -bw_mult_wl/(bw_mult_wh-bw_mult_wl)
    float pole_sep;			// [#], pole separation factor for controller
    float vq_max;			// [V], maximum q-axis voltage
} DELTA_CTRL_PARAMS_t;
#endif

typedef enum
{
    Neutral_Point_Modulation = 0,
    Space_Vector_Modulation = 1,
} MOD_t;

typedef struct
{
    EN_DIS_t en;
    float active_mi;			// [0-1] 7-segment to 5-segment switchover modulation index
    float inactive_mi;			// [0-1] 5-segment to 7-segment switchover modulation index
    float w0_filt;              // [Ra/sec], modulation index low pass filter bandwidth
} FIVE_SEG_PARAMS_t;

typedef struct
{
    ELEC_t w_thresh;			// [Ra/sec-elec], min speed for voltage control
    ELEC_t w_hyst;				// [Ra/sec-elec], hysteresis for min speed
    float v_min;				// [Vpk], min applied voltage at zero speed
    float v_to_f_ratio;			// [Vpk/(Ra/sec-elec)], voltage to frequency ratio
    MOD_t mod_method;			// [], modulation method
    FIVE_SEG_PARAMS_t five_seg; // [], 5-segment SVM parameters
} VOLT_CTRL_PARAMS_t;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

typedef struct
{
    EN_DIS_t en;			// [#], enable or disable switch
    float vdc_coeff;		// [#], vdc coefficient = (vdc margin) / sqrt(3) 
#if defined(CTRL_METHOD_RFO)
    float bw;				// [Ra/sec], bandwidth
    float ki;				// [Ra/sec.A/V]
#elif defined(CTRL_METHOD_SFO)
    ELEC_t w_min;			// [Ra/sec-elec], minimum speed for flux weakening
#endif
} FLUX_WEAKEN_PARAMS_t;

typedef struct
{
    float time;			// [sec]
    float voltage;		// [sec]
} ALIGN_PARAMS_t;

typedef struct
{
    float i_peak;		// [A]
    float t_on;			// [sec]
    float t_off;		// [sec]
    float v_pulse;		// [V], dc+ to dc-
} SIX_PULSE_INJ_PARAMS_t;

typedef struct
{
    float w_h;				// [Ra/sec], excitation freqeuncy, at least 1 decade below switching frequency
    float w_sep;			// [Ra/sec], separation freqeuncy, at least 1 decade below excitation frequency
    PLL_PARAMS_t pll;		// []
    QD_t i_qd_r_peak;		// [A], excitation current peaks
    QD_t v_qd_r_coeff;		// {[V/(Ra/sec],[V]}, excitation voltage coefficients, line-to-neutral
    float lpf_biquad_a[3];	// Low-pass biquad filter coefficients:
    float lpf_biquad_b[3];	// H(s) = (a0+a1*s+a2*s^2)/(b0+b1*s+b2*s^2)
    float bw_red_coeff;		// [%], bandwidth reduction coefficient applied to current loop (RFO) or flux/delta loops (SFO) when running high frequency injection
    float lock_time;		// [sec], lock time in sec
} HIGH_FREQ_INJ_PARAMS_t;

typedef struct
{
    bool overwrite;                 // [], whether to overwrite params.motor.x with the results after being done
    float cmd_thresh;               // [%], command threshold for starting
    float cmd_hyst;                 // [%], command hysteresis for stopping
    float i_cmd_dc;                 // [A], dc current command
    float i_cmd_ac;                 // [A], desired ac current command magnitude
    MINMAX_t w_cmd_elec;            // [Ra/sec-elec], speed command range for lam estimation
    float time_rot_lock;            // [sec], time spent for initial rotor locking
    float time_res;                 // [sec], time spent for resistance estimation
    float time_ind;                 // [sec], time spent for inductance estimation
    float time_spd;                 // [sec], time spent at each speed point for estimating flux, inertia, friction, and voltage parameters
    float w_h[PROF_FREQ_POINTS];    // [Ra/sec], excitation frequency list
    float w_sep;                    // [Ra/sec], seperation frequency, for separating dc and ac components
    float w0_idc;                   // [Ra/sec], dc current controller's bandwidth
    float kp_idc;                   // [V/A], dc current controller's kp
    float ki_idc;                   // [(V/A).(Ra/sec)], dc current controller's ki
    float w0_flux;                  // [Ra/sec], flux-estimation low-pass-filter's bandwidth

} PROFILER_PARAMS_t;

#endif

#if defined(CTRL_METHOD_TBC)
typedef enum
{
    Block_Commutation = 0U,
    Trapezoidal_Commutation = 1U
} TRAP_BLOCK_COMM_MODE_t;

typedef struct
{
    uint16_t ramp_cnt;          // [#]
    float ramp_cnt_inv;         // [#]
    float ramp_main_bw_ratio;   // [#], ramp to main PI controller bandwith ratio
    float ramp_kp;			    // [V/A]
    float ramp_ki;			    // [(V/A).(Ra/sec)]
    float ramp_ff_coef;		    // [V/A], ramp PI feedforward coefficient
    float main_ff_coef;         // [#], main PI feedforward coefficient
} TRAP_COMM_PARAMS_t;
typedef struct
{
    TRAP_BLOCK_COMM_MODE_t mode;
    TRAP_COMM_PARAMS_t trap;
} TRAP_BLOCK_COMM_PARAMS_t;
#endif

typedef struct
{
    CTRL_MODE_t mode;
    SPEED_CTRL_PARAMS_t speed;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    CURRENT_CTRL_PARAMS_t curr;
#if defined(CTRL_METHOD_TBC)
    TRAP_BLOCK_COMM_PARAMS_t tbc;
#endif
#elif defined(CTRL_METHOD_SFO)
    TRQ_CTRL_PARAMS_t trq;
    FLUX_CTRL_PARAMS_t flux;
    DELTA_CTRL_PARAMS_t delta;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    FLUX_WEAKEN_PARAMS_t flux_weaken;
    ALIGN_PARAMS_t align;
    SIX_PULSE_INJ_PARAMS_t six_pulse_inj;
    HIGH_FREQ_INJ_PARAMS_t high_freq_inj;
#endif
    VOLT_CTRL_PARAMS_t volt;
} CTRL_PARAMS_t;


typedef struct
{
    uint32_t code;
    uint16_t build_config;	// changing build config should overwrite params
    uint16_t ver;			// 16 bits only for legacy reasons (compatibility with gui)
} PARAMS_ID_t;

typedef struct /* Enable/disable autocal for selected functions*/
{
  union
  {
    struct
    {
      uint32_t vdc_fault_threshold    : 1; /* if Bit is set, auto calculation in PARAMS_InitAutoCalc() for current (d & q) loop related parameter are skipped*/
      uint32_t speed_fault_threshold  : 1;
      uint32_t speed_control          : 1;
#if defined(CTRL_METHOD_SFO)
      uint32_t torque_control         : 1;
#else
      uint32_t current_control        : 1;
#endif
      uint32_t observer_pll		      : 1;
#if defined(CTRL_METHOD_RFO)
      uint32_t flux_weakening         : 1;
#endif
#if defined(CTRL_METHOD_SFO)
      uint32_t flux_control           :1;
#endif
     };
     uint32_t word_access;
  };

}PARAM_AUTOCAL_DISABLE_t;

typedef struct
{
    PARAMS_ID_t id;
    MOTOR_PARAMS_t motor;
    SYS_PARAMS_t sys;
    OBS_PARAMS_t obs;
    MECH_PARAMS_t mech;
    FILTER_PARAMS_t filt;
    CTRL_PARAMS_t ctrl;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    PROFILER_PARAMS_t profiler;
#endif

    PARAM_AUTOCAL_DISABLE_t autocal_disable;
} PARAMS_t;

#pragma pack(push,1)
typedef struct	// For external identification (e.g. GUI)
{
    uint32_t  chip_id;				    // Chip ID
    uint16_t  parameter_version;		// Parameter version
    uint16_t  firmware_version;			// Firmware version
    uint8_t   kit_id;					// Kit ID
    uint8_t   build_config_id;			// Build configuration ID
} MC_INFO_t;
#pragma pack(pop)




extern PARAMS_t params;

extern MC_INFO_t mc_info;

void PARAMS_Init();
void PARAMS_InitManual();
void PARAMS_InitAutoCalc();

void PARAMS_DEFAULT_Init();
void PARAMS_DEFAULT_InitManual();
void PARAMS_DEFAULT_InitAutoCalc();


