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

/**
 * @file Params.h
 * @brief Motor control parameter definitions and structures
 *
 * Defines all parameter structures for motor control including motor parameters,
 * system parameters, control parameters, and calibration parameters. Provides
 * version management and parameter storage functionality.
 */

#pragma once
#include "General.h"

#ifndef MOTOR_CTRL_NO_OF_MOTOR
#define MOTOR_CTRL_NO_OF_MOTOR (1)
#endif

// Latest firmware version, 0xAABC==vAA.B.C, Example: 0x0150==v1.5.0.
#define FIRMWARE_VER (0x0320UL)    /**< Firmware version encoded as 0xAABC = vAA.B.C */
#define PARAMS_CODE (~0xBADC0DEUL) /**< Magic code that identifies a valid parameter set (inverted key) */
#define PARAMS_VER (0x0006UL)      /**< Parameter structure version — increment when PARAMS_t layout changes */
#if defined(CTRL_METHOD_RFO)
#define BUILD_CONFIG_ID (0x0000UL) /**< Build configuration ID: 0 = RFO */
#elif defined(CTRL_METHOD_SFO)
#define BUILD_CONFIG_ID (0x0001UL) /**< Build configuration ID: 1 = SFO */
#elif defined(CTRL_METHOD_TBC)
#define BUILD_CONFIG_ID (0x0002UL) /**< Build configuration ID: 2 = TBC */
#else
#define BUILD_CONFIG_ID (0x0000UL) /**< Build configuration ID: default = RFO */
#endif

/**
 * @brief Motor thermal (I²T) protection parameters
 */
typedef struct
{
    float therm_tau; /**< Thermal time constant [sec] */
    float on_level;  /**< I²T protection enable level [% of motor thermal capacity] */
    float off_level; /**< I²T protection disable level [% of motor thermal capacity] */
} I2T_PARAMS_t;

/**
 * @brief Motor electrical and mechanical parameters
 */
typedef struct
{
    float P;      /**< Number of pole pairs */
    float lq;     /**< Q-axis inductance [H] */
    float ld;     /**< D-axis inductance [H] */
    float lam;    /**< PM flux linkage [Wb] */
    float r;      /**< Stator resistance [ohm] */
    float T_max;  /**< Maximum torque rating [Nm] */
    float i_peak; /**< Peak current rating [A] */
    float i_cont; /**< Continuous current rating [A] */
    float id_max; /**< Maximum demagnetization d-axis current [A] */
    float v_nom;  /**< Nominal phase voltage peak, line-to-neutral [Vpk] */
    ELEC_t w_nom; /**< Nominal electrical speed [rad/sec-elec] */
    ELEC_t w_max; /**< Maximum electrical speed [rad/sec-elec] */
    float zeta;   /**< Saliency ratio: Lq / Ld */
#if defined(CTRL_METHOD_SFO)
    float mtpv_margin; /**< MTPV torque margin applied to maximum torque [%] */
    LUT_1D_t mtpa_lut; /**< MTPA lookup table: torque command [Nm] → flux command [Wb] */
    LUT_1D_t mtpv_lut; /**< MTPV lookup table: flux command [Wb] → maximum torque [Nm] */
#endif
    I2T_PARAMS_t i2t; /**< Motor thermal (I²T) protection parameters */
} MOTOR_PARAMS_t;

/**
 * @brief Sampling frequency and timing parameters
 */
typedef struct
{
    float fpwm;              /**< PWM switching frequency [Hz] */
    float tpwm;              /**< PWM period [sec] */
    uint32_t fpwm_fs0_ratio; /**< Ratio of PWM frequency to ISR0 frequency */
    float fs0;               /**< ISR0 sampling frequency [Hz] */
    float ts0;               /**< ISR0 sampling period [sec] */
    uint32_t fs0_fs1_ratio;  /**< Ratio of ISR0 to ISR1 frequency */
    float fs1;               /**< ISR1 sampling frequency [Hz] */
    float ts1;               /**< ISR1 sampling period [sec] */
    float deadtime;          /**< PWM deadtime [sec] */
#if defined(PC_TEST)
    uint32_t fsim_fs0_ratio; /**< Ratio of simulation frequency to ISR0 frequency */
    float fsim;              /**< Simulation sampling frequency [Hz] */
    float tsim;              /**< Simulation sampling period [sec] */
#endif
} SAMPLE_PARAMS_t;

/**
 * @brief Pre-computed trigonometric lookup tables
 */
typedef struct
{
    TRIG_LUT_t sin;      /**< Sin/cos LUT for Park transforms */
    INV_TRIG_LUT_t atan; /**< Arctangent LUT for control calculations */
    INV_TRIG_LUT_t asin; /**< Arcsine LUT for control calculations */
} LUT_PARAMS_t;

/**
 * @brief Gain and offset calibration for a single analog channel
 *
 * Applied as: output = input * gain + offset.
 * Input and output are in SI units as scaled by the HAL.
 */
typedef struct
{
    float gain;   /**< Gain correction factor [%] */
    float offset; /**< Offset correction in SI units */
} CALIB_PARAMS_t;

/**
 * @brief Calibration parameters for all analog sensor channels
 */
typedef struct
{
    CALIB_PARAMS_t i_u;     /**< Phase U current calibration */
    CALIB_PARAMS_t i_v;     /**< Phase V current calibration */
    CALIB_PARAMS_t i_w;     /**< Phase W current calibration */
    CALIB_PARAMS_t v_uz;    /**< Phase U-to-neutral voltage calibration */
    CALIB_PARAMS_t v_vz;    /**< Phase V-to-neutral voltage calibration */
    CALIB_PARAMS_t v_wz;    /**< Phase W-to-neutral voltage calibration */
    CALIB_PARAMS_t v_dc;    /**< DC bus voltage calibration */
    CALIB_PARAMS_t temp_ps; /**< Power-stage temperature sensor calibration */
    CALIB_PARAMS_t pot;     /**< Potentiometer calibration */
} ANALOG_CALIB_PARAMS_t;

/**
 * @brief Low-pass filter cut-off frequencies for analog sensor channels
 */
typedef struct
{
    float w0_i_ph;    /**< Phase current filter bandwidth [rad/sec] */
    float w0_v_ph;    /**< Phase voltage filter bandwidth [rad/sec] */
    float w0_v_dc;    /**< DC bus voltage filter bandwidth [rad/sec] */
    float w0_temp_ps; /**< Power-stage temperature filter bandwidth [rad/sec] */
    float w0_pot;     /**< Potentiometer filter bandwidth [rad/sec] */
} ANALOG_FILT_PARAMS_t;

/**
 * @brief Phase current measurement shunt topology
 */
typedef enum
{
    Three_Shunt = 0U,  /**< Direct measurement using three phase shunts */
    Single_Shunt = 1U, /**< Reconstruction via DC-link single shunt */
    Two_Shunt =2U         /**< Direct measurement using two phase shunts (third reconstructed) */
} SHUNT_TYPE_t;

/**
 * @brief Current sensing element type
 */
typedef enum
{
    Shunt_Res = 0U, /**< Shunt resistor current sensing */
    Active_Sensor   /**< Active current sensor (e.g. Hall-effect or isolated amplifier) */
} CS_MEAS_TYPE_t;

/**
 * @brief Current sense polarity (placement of sense element)
 */
typedef enum
{
    LS_Current_Sense = 1, /**< Shunt/sensor between motor phase and ground (low side) */
    HS_Current_Sense = -1 /**< Shunt/sensor between supply and motor phase (high side) */
} CS_MEAS_POLARITY_t;
/**
 * @brief Single-shunt current reconstruction method
 */
typedef enum
{
    Hyb_Mod = 0U,    /**< Hybrid modulation: shifts PWM vectors to create ADC sampling windows */
    Phase_Shift = 1U /**< Phase-shifted PWM: creates fixed ADC sampling window per phase */
} SINGLE_SHUNT_MEAS_TYPE_t;

/**
 * @brief Single-shunt hybrid modulation ADC timing parameters
 */
typedef struct
{
    float adc_t_min; /**< Minimum ADC sampling window time [sec] */
    float adc_d_min; /**< Minimum duty cycle required for ADC sampling [%] */
    float ki;        /**< Duty cycle correction gain (error[n]/error[n-1] = 1 - ki) */
} SHUNT_HYB_MOD_t;
/**
 * @brief Single-shunt current sensing parameters (hybrid or phase-shift modulation)
 */
typedef struct
{
    float adc_t_min;               /**< Minimum ADC sampling window time [sec] */
    float adc_d_min;               /**< Minimum duty cycle for ADC sampling [%] */
    float ki;                      /**< Duty cycle correction gain for hybrid modulation */
    float adc_t_sh_delay;          /**< Phase-shift PWM ADC switch delay time [sec] */
    float adc_d_sh_delay;          /**< Phase-shift PWM ADC switch delay duty [%] */
    SINGLE_SHUNT_MEAS_TYPE_t type; /**< Current reconstruction method (hybrid or phase-shift) */
} SINGLE_SHUNT_CS_t;

/**
 * @brief Current sensor (shunt / active) configuration parameters
 */
typedef struct
{
    SHUNT_TYPE_t type; /**< Shunt topology (three-shunt, single-shunt, two-shunt) */
    float opamp_gain;  /**< Op-amp gain applied to the shunt signal */
    union
    {
        SHUNT_HYB_MOD_t hyb_mod;        /**< Hybrid modulation parameters (single-shunt only) */
        SINGLE_SHUNT_CS_t single_shunt; /**< Phase-shift modulation parameters (single-shunt only) */
    };
    float res;                     /**< Shunt resistance [ohm] (used when cs_meas_type = Shunt_Res) */
    float current_sensitivity;     /**< Sensor sensitivity [V/A] (used when cs_meas_type = Active_Sensor) */
    int8_t current_sense_polarity; /**< Measurement polarity: +1 = low-side, -1 = high-side */
    float cs_settle_raio;          /**< Current sense settle ratio for timing margin */
    int8_t internal_gain;          /**< Internal amplifier gain applied to current inputs */
} ANALOG_SHUNT_PARAMS_t;

/**
 * @brief Voltage measurement resistor divider scale factors
 */
typedef struct
{
    float vdc_adc_scale;  /**< DC bus voltage divider scale: LowSideRes/(LowSideRes+HighSideRes) [V/V] */
    float vuvw_adc_scale; /**< Phase voltage divider scale for phase-to-neutral measurement [V/V] */
} VOLTAGE_MEASUREMENT_PARAMS_t;

/**
 * @brief Complete analog sensor configuration parameters
 */
typedef struct
{
    ANALOG_CALIB_PARAMS_t calib;       /**< Gain/offset calibration for all channels */
    ANALOG_FILT_PARAMS_t filt;         /**< IIR filter cut-off frequencies */
    CS_MEAS_TYPE_t cs_meas_type;       /**< Current measurement element type (shunt or active) */
    ANALOG_SHUNT_PARAMS_t shunt;       /**< Current shunt/sensor configuration */
    float offset_null_time;            /**< Duration of offset-nulling in Init state [sec] */
    VOLTAGE_MEASUREMENT_PARAMS_t volt; /**< Voltage measurement scale factors */
    float vref_gain;                   /**< ADC reference voltage gain correction */
} ANALOG_SENS_PARAMS_t;

/**
 * @brief Command rate limiter slopes
 */
typedef struct
{
    ELEC_t w_cmd;    /**< Speed command ramp rate [rad/sec-elec / sec] */
    ELEC_t w_ol_cmd; /**< Open-loop speed command ramp rate [rad/sec-elec / sec] */
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    float i_cmd; /**< Current command ramp rate [A/sec] */
#elif defined(CTRL_METHOD_SFO)
    float T_cmd; /**< Torque command ramp rate [Nm/sec] */
#endif
#if defined(CTRL_METHOD_RFO)
    float p_cmd; /**< Position command ramp rate [rad/sec] */
#endif
} RATE_LIM_PARAMS_t;

/**
 * @brief Fault reaction short-circuit method
 */
typedef enum
{
    Low_Side_Short = 0U,  /**< Enable all low-side switches (motor short via low side) */
    High_Side_Short = 1U, /**< Enable all high-side switches (motor short via high side) */
    Alternate_Short = 2U  /**< Alternate between low and high side shorting */
} SHORT_METHOD_t;

/**
 * @brief Phase loss detection parameters
 */
typedef struct
{
    float tau;                /**< Phase loss detection filter time constant [sec] */
    float zero_current_thres; /**< Current threshold below which a phase is considered open [A] */
} PHASE_LOSS_PARAMS_t;

/**
 * @brief Fault detection thresholds and reaction configuration
 */
typedef struct
{
    float oc_thresh;                /**< Over-current threshold [% of motor I²T capacity] */
    MINMAX_t vdc_thresh;            /**< DC bus over/under-voltage fault thresholds [V] */
    float vdc_time;                 /**< DC bus voltage fault debounce time [sec] */
    float temp_ps_thresh;           /**< Power-stage over-temperature fault threshold [°C] */
    ELEC_t w_thresh;                /**< Over-speed fault threshold [rad/sec-elec] */
    SHORT_METHOD_t short_method;    /**< Motor short method used for fault reaction */
    float cmd_clr_thresh;           /**< Command threshold below which faults can be cleared [%] */
    uint32_t max_clr_tries;         /**< Maximum fault-clear retries before permanent fault state */
    float clr_try_period;           /**< Fault-clear retry period [sec] */
    uint32_t watchdog_time;         /**< Watchdog reset timeout [ms] */
    PHASE_LOSS_PARAMS_t phase_loss; /**< Phase loss detection parameters */
} FAULT_PARAMS_t;

/**
 * @brief Command input source selection
 */
typedef enum
{
    Internal = 0, /**< Command from potentiometer (on-board analog input) */
    External,     /**< Command from external interface (GUI, UART, etc.) */
} CMD_SOURCE_t;

/**
 * @brief Command source and full-scale limit configuration
 */
typedef struct
{
    CMD_SOURCE_t source; /**< Command input source (potentiometer or external) */
    // Full scale values corresponding to potentiometer reading of 100%
    MECH_t w_max; /**< Full-scale speed command [rad/sec-mech] */
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    float i_max; /**< Full-scale current command [A] */
#elif defined(CTRL_METHOD_SFO)
    float T_max; /**< Full-scale torque command [Nm] */
#endif
#if defined(CTRL_METHOD_RFO)
    float p_max; /**< Full-scale position command [rad] */
#endif
} CMD_PARAMS_t;

/**
 * @brief Adaptive tracking loop parameters (used by Hall and incremental encoder)
 */
typedef struct
{
    uint16_t cpr;    /**< Counts per mechanical revolution (6*(P/2) for Hall, 4*lpr for encoder) */
    float w0_w;      /**< Speed feed-forward low-pass filter bandwidth [rad/sec] */
    MINMAX_t w0_th;  /**< Angle estimation loop bandwidth limits [rad/sec] */
    float tau_ratio; /**< Ratio of angle estimation loop time constant to captured angle period */
} ADAP_TRACK_LOOP_PARAMS_t;

/**
 * @brief Hall sensor feedback parameters
 */
typedef struct
{
    ADAP_TRACK_LOOP_PARAMS_t track_loop; /**< Adaptive tracking loop parameters */
    ELEC_t w_zsd_thresh;                 /**< Zero-speed detection threshold [rad/sec-elec] */
    EN_DIS_t block_comm_offset_comp;     /**< Enable/disable offset compensation for block commutation */
    ELEC_t th_r_offset;                  /**< Rotor angle offset based on Hall sensor placement [rad-elec] */
} HALL_SENS_PARAMS_t;

/**
 * @brief Incremental encoder feedback parameters
 */
typedef struct
{
    ADAP_TRACK_LOOP_PARAMS_t track_loop; /**< Adaptive tracking loop parameters */
    ELEC_t w_zsd_thresh;                 /**< Zero-speed detection threshold [rad/sec-elec] */
} INC_ENCODER_PARAMS_t;

/**
 * @brief Feedback sensor mode and sensor-specific configuration
 */
typedef struct
{
    FB_MODE_t mode;               /**< Active feedback sensor mode */
    HALL_SENS_PARAMS_t hall;      /**< Hall sensor configuration */
    INC_ENCODER_PARAMS_t encoder; /**< Incremental encoder configuration */
} FB_PARAMS_t;

#if defined(CTRL_METHOD_RFO)
typedef enum
{
    Zero_Current_Control = 0, // Control zero current control
    Direct_Bemf_Measure = 1,  // Direct Phase bemf measurement
} CATCH_SPIN_MODE_t;

typedef struct
{
    CATCH_SPIN_MODE_t mode; // Zero current control or direct bemf measurement
    float time;             // [sec], minimum lock time to capture the speed of the free running motor
    ELEC_t w_thresh;        // [Ra/sec-elec], threshold speed
} CATCH_SPIN_PARAMS_t;
#endif

typedef enum
{
    Boot_and_Brake = 0, // Brake and boot by clamp all the low side switches
    Boot_Only = 1,      // Enable each phase low side switches separately, every PWM period
} BOOT_STRAP_MODE_t;

typedef struct
{
    SAMPLE_PARAMS_t samp;        // [], sampling parameters
    ANALOG_SENS_PARAMS_t analog; // [], analog sensor parameters
    RATE_LIM_PARAMS_t rate_lim;  // [], rate limiter parameters
    FAULT_PARAMS_t faults;       // [], fault parameters
    CMD_PARAMS_t cmd;            // [], command parameters
    FB_PARAMS_t fb;              // [#], feedback parameters
    float boot_time;             // [sec], time for charging bootstrap capacitors
    BOOT_STRAP_MODE_t boot_mode; // 0 - Boot and brake, 1 : Boot only
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    float dyno_lock_time; // [sec], minimum lock time in dyno mode
#endif
#if defined(CTRL_METHOD_RFO)
    CATCH_SPIN_PARAMS_t catch_spin;
#endif
    float vdc_nom; // [], dc bus voltage parameters
} SYS_PARAMS_t;

typedef struct
{
    // H(s)=s^2/(s^3+c1*s^2+c2*s+c3)*gain
    // c1=(k1+k2+k3)*|w0|
    // c2=(k1*k2+k1*k3+k2*k3)*|w0|^2
    // c3=k1*k2*k3*|w0|^3
    // th_p=atan(((k1+k2+k3)-(k1*k2*k3))/(1-(k1*k2+k1*k3+k2*k3)))
    // th_p'=th_p*sign(w0)
    float gain;        // [#], see definitions above
    float k1;          // [#], see definitions above
    float k2;          // [#], see definitions above
    float k3;          // [#], see definitions above
    float c1_coeff;    // =c1/|w0|=(k1+k2+k3)
    float c2_coeff;    // =c2/|w0|^2=(k1*k2+k1*k3+k2*k3)
    float c3_coeff;    // =c3/|w0|^3=(k1*k2*k3)
    ELEC_t th_p;       // [Ra-elec], phase shift of the filter
    PARK_t phase_lead; // Park transform for phase shift compensation
} FLUX_FILT_PARAMS_t;

typedef struct
{
    float w0;         // [Ra/sec]
    float kp;         // [(Ra/sec-elec)/(Input Unit)]
    float ki;         // [(Ra/sec).(Ra/sec-elec)/(Input Unit)]
    ELEC_t w_max;     // [Ra/sec-elec]
    ELEC_t th_offset; // [Ra-elec]
} PLL_PARAMS_t;

typedef struct
{
    FLUX_FILT_PARAMS_t flux_filt;
    float biquad_a[3]; // Angular frequency biquad filter coefficients
    float biquad_b[3]; // H(s) = (a0+a1*s+a2*s^2)/(b0+b1*s+b2*s^2)
    PLL_PARAMS_t pll;
    ELEC_t w_thresh; // [Ra/sec-elec], min speed for observer
    ELEC_t w_hyst;   // [Ra/sec-elec], hysteresis for min speed
    float lock_time; // [sec], waiting time to ensure pll locking above w_thresh
} OBS_PARAMS_t;

typedef struct
{
    float inertia;  // [kg.m^2]=[(N.m)/(Ra/sec-mech).sec], =1/2*m*r^2
    float viscous;  // [kg.m^2/sec]=[(N.m)/(Ra/sec-mech)]
    float friction; // [kg.m^2/sec^2]=[N.m]
} MECH_PARAMS_t;

typedef struct
{
    EN_DIS_t spd_ar_en; // [#], enable or disable speed anti-resonant filter
    float spd_ar_wz[2]; // [Ra/sec], speed anti-resonant filter's zero locations
    float spd_ar_wp[2]; // [Ra/sec], speed anti-resonant filter's pole locations
    float acc_w0;       // [Ra/sec], cut off frequency for acceleration estimation filter
    float trq_w0;       // [Ra/sec], cut off frequency for torque estimation filter
} FILTER_PARAMS_t;

typedef enum
{
    Volt_Mode_Open_Loop = 0, // Open-loop V/Hz control
#if defined(CTRL_METHOD_RFO)
    Curr_Mode_Open_Loop,                       // Open-loop current control
    Curr_Mode_FOC_Sensorless_Align_Startup,    // Closed-loop sensorless-foc current control with pre-alignment at startup
    Curr_Mode_FOC_Sensorless_SixPulse_Startup, // Closed-loop sensorless-foc current control with six pulse injection at
                                               // startup
    Curr_Mode_FOC_Sensorless_HighFreq_Startup, // Closed-loop sensorless-foc current control with high frequency
                                               // injection at startup
    Curr_Mode_FOC_Sensorless_Dyno,             // Closed-loop sensorless-foc current control in dyno mode (waiting for observer lock
                                               // to start up)
    Curr_Mode_FOC_Encoder_Align_Startup,       // Closed-loop sensored-foc current control with encoder feedback and
                                               // pre-alignment at startup
    Curr_Mode_FOC_Hall,                        // Closed-loop sensored-foc current control with hall sensor feedback
#elif defined(CTRL_METHOD_TBC)
    Curr_Mode_Block_Comm_Hall, // Closed-loop block-commutation current control with hall sensor feedback
#elif defined(CTRL_METHOD_SFO)
    Trq_Mode_FOC_Sensorless_Align_Startup,    // Closed-loop sensorless-foc torque control with pre-alignment at startup
    Trq_Mode_FOC_Sensorless_SixPulse_Startup, // Closed-loop sensorless-foc torque control with six pulse injection at
                                              // startup
    Trq_Mode_FOC_Sensorless_HighFreq_Startup, // Closed-loop sensorless-foc torque control with high frequency injection
                                              // at startup
    Trq_Mode_FOC_Sensorless_Dyno,             // Closed-loop sensorless-foc torque control in dyno mode (waiting for observer lock
                                              // to start up)
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    Speed_Mode_FOC_Sensorless_Align_Startup,    // Closed-loop sensorless-foc speed control with pre-alignment at startup
    Speed_Mode_FOC_Sensorless_SixPulse_Startup, // Closed-loop sensorless-foc speed control with six pulse injection at
                                                // startup
    Speed_Mode_FOC_Sensorless_HighFreq_Startup, // Closed-loop sensorless-foc speed control high frequency injection at
                                                // startup
    Speed_Mode_FOC_Sensorless_Volt_Startup,     // Closed-loop sensorless-foc speed control with open-loop voltage control
                                                // at startup
#endif
#if defined(CTRL_METHOD_RFO)
    Speed_Mode_FOC_Sensorless_Curr_Startup, // Closed-loop sensorless-foc speed control with open-loop current control
                                            // at startup
    Speed_Mode_FOC_Encoder_Align_Startup,   // Closed-loop sensored-foc speed control with encoder feedback and
                                            // pre-alignment at startup
    Speed_Mode_FOC_Hall,                    // Closed-loop sensored-foc speed control with hall sensor feedback
#elif defined(CTRL_METHOD_TBC)
    Speed_Mode_Block_Comm_Hall, // Closed-loop block-commutation speed control with hall sensor feedback
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    Profiler_Mode, // Profiler mode
#endif
#if defined(CTRL_METHOD_RFO)
    Position_Mode_FOC_Encoder_Align_Startup // Closed-loop sensored-foc position control with encoder feedback and
                                            // pre-alignment at startup
#endif
} CTRL_MODE_t;

typedef struct
{
    float bw;             // [Ra/sec], bandwidth
    float kp;             // [A/(Ra/sec-elec)] in RFO, [Nm/(Ra/sec-elec)] in SFO
    float ki;             // [A/(Ra/sec-elec).(Ra/sec)] in RFO, [Nm/(Ra/sec-elec).(Ra/sec)] in SFO
    float ki_multiple;    // [#], usually in [1-100] range, higher values increase the responsiveness of the speed loop but
                          // reduce the phase margin
    float ff_k_inertia;   // [A/(Ra/sec-elec).sec] in RFO, [Nm/(Ra/sec-elec).sec] in SFO, inertia feed forward coefficient
    float ff_k_viscous;   // [A/(Ra/sec-elec)] in RFO, [Nm/(Ra/sec-elec)] in SFO, viscous damping feed forward coefficient
    float ff_k_friction;  // [A] in RFO, [Nm] in SFO, friction feed forward coefficient
    float ol_cl_tr_coeff; // [%] torque precalculation coefficient when transitioning from ol to cl
    float ff_coef;        // [#], feed forward coefficient
} SPEED_CTRL_PARAMS_t;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
typedef struct
{
    float bw; // [Ra/sec], bandwidth
#if defined(CTRL_METHOD_RFO)
    QD_t kp;    // [V/A]
    QD_t ki;    // [(V/A).(Ra/sec)]
    QD_t v_max; // [V]
#elif defined(CTRL_METHOD_TBC)
    float kp;       // [V/A]
    float ki;       // [(V/A).(Ra/sec)]
    float v_max;    // [V]
    bool bypass;    // []
    float k_bypass; // [V/A]
#endif
    float ff_coef;      // [#], feed forward coefficient
    float i_cmd_thresh; // [A], threshold for transition to current control
    float i_cmd_hyst;   // [A], hysteresis for i_cmd_thresh
    float i_cmd_ol;     // [A], open-loop current command
} CURRENT_CTRL_PARAMS_t;

#elif defined(CTRL_METHOD_SFO)
typedef struct
{
    float w_p;              // [Ra/sec]
    float w_ratio;          // [%], = w_p/w_z, w_p<w_z
    float kp;               // [(Ra-elec)/(Nm)]
    float ki;               // [(Ra/sec).(Ra-elec)/(Nm)]
    ELEC_t delta_max;       // [Ra-elec], maximum load angle command
    float T_cmd_thresh;     // [Nm], threshold for transition from prepositioning to torque control
    float T_cmd_hyst;       // [Nm], hysteresis for T_cmd_thresh
    float curr_lmt_t_reach; // [sec], sliding-mode current limiter's worst-case reaching time
    float curr_lmt_ki;      // [Nm/A.Ra/sec], sliding-mode current limiter's ki
} TRQ_CTRL_PARAMS_t;

typedef struct
{
    float bw;       // [Ra/sec]
    float pole_sep; // [#]
    float kp;       // [V/Wb]
    float ki;       // [(Ra/sec).(V/Wb)]
    float vd_max;   // [V], maximum d-axis voltage command
} FLUX_CTRL_PARAMS_t;

typedef struct
{
    float bw;            // [Ra/sec], bandwidth of load angle controller
    float bw_mult;       // [#], bandwidth multiplication factor
    ELEC_t bw_mult_wl;   // [Ra/sec-elec], low speed threshold for bandwidth multiplcation factor
    ELEC_t bw_mult_wh;   // [Ra/sec-elec], high speed threshold for bandwidth multiplcation factor
    float bw_mult_slope; // [1/(Ra/sec-elec)], =+1/(bw_mult_wh-bw_mult_wl)
    float bw_mult_inter; // [#], = -bw_mult_wl/(bw_mult_wh-bw_mult_wl)
    float pole_sep;      // [#], pole separation factor for controller
    float vq_max;        // [V], maximum q-axis voltage
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
    float active_mi;   // [0-1] 7-segment to 5-segment switchover modulation index
    float inactive_mi; // [0-1] 5-segment to 7-segment switchover modulation index
    float w0_filt;     // [Ra/sec], modulation index low pass filter bandwidth
} FIVE_SEG_PARAMS_t;

typedef struct
{
    ELEC_t w_thresh;            // [Ra/sec-elec], min speed for voltage control
    ELEC_t w_hyst;              // [Ra/sec-elec], hysteresis for min speed
    float v_min;                // [Vpk], min applied voltage at zero speed
    float v_to_f_ratio;         // [Vpk/(Ra/sec-elec)], voltage to frequency ratio
    MOD_t mod_method;           // [], modulation method
    FIVE_SEG_PARAMS_t five_seg; // [], 5-segment SVM parameters
} VOLT_CTRL_PARAMS_t;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

typedef struct
{
    EN_DIS_t en;     // [#], enable or disable switch
    float vdc_coeff; // [#], vdc coefficient = (vdc margin) / sqrt(3)
#if defined(CTRL_METHOD_RFO)
    float bw; // [Ra/sec], bandwidth
    float ki; // [Ra/sec.A/V]
#elif defined(CTRL_METHOD_SFO)
    ELEC_t w_min; // [Ra/sec-elec], minimum speed for flux weakening
#endif
} FLUX_WEAKEN_PARAMS_t;

typedef struct
{
    float time;    // [sec]
    float voltage; // [sec]
} ALIGN_PARAMS_t;

typedef struct
{
    float i_peak;  // [A]
    float t_on;    // [sec]
    float t_off;   // [sec]
    float v_pulse; // [V], dc+ to dc-
} SIX_PULSE_INJ_PARAMS_t;

typedef enum
{
    Sine_Wave = 0U,
    Square_Wave
} HIGH_FREQ_INJ_TYPE_t;

typedef struct
{
    float w_sep;           // [Ra/sec], separation freqeuncy, at least 1 decade below excitation frequency
    QD_t i_qd_r_peak;      // [A], excitation current peaks
    QD_t v_qd_r_coeff;     // {[V/(Ra/sec],[V]}, excitation voltage coefficients, line-to-neutral
    float lpf_biquad_a[3]; // Low-pass biquad filter coefficients:
    float lpf_biquad_b[3]; // H(s) = (a0+a1*s+a2*s^2)/(b0+b1*s+b2*s^2)
} HIGH_FREQ_INJ_SIN_PARAMS_t;

typedef struct
{
    float t_h; // [sec], excitation time (half period)
    float v_h; // [V], excitation voltage magnitude
} HIGH_FREQ_INJ_SQR_PARAMS_t;

typedef struct
{
    HIGH_FREQ_INJ_TYPE_t type; // [], hfi type
    float i_peak;              // [A], excitation current peak
    float w_h;                 // [Ra/sec], excitation freqeuncy, must be less than half fs0
    union
    { /* Added to maintain backward compatibility  between Motor control lib V3.0.0 */
        struct
        {
            float w_sep;           // [Ra/sec], separation freqeuncy, at least 1 decade below excitation frequency
            QD_t i_qd_r_peak;      // [A], excitation current peaks
            QD_t v_qd_r_coeff;     // {[V/(Ra/sec],[V]}, excitation voltage coefficients, line-to-neutral
            float lpf_biquad_a[3]; // Low-pass biquad filter coefficients:
            float lpf_biquad_b[3]; // H(s) = (a0+a1*s+a2*s^2)/(b0+b1*s+b2*s^2)
        };
        HIGH_FREQ_INJ_SIN_PARAMS_t sine; // [], sine wave type's parameters
    };
    HIGH_FREQ_INJ_SQR_PARAMS_t square; // [], square wave type's parameters
    PLL_PARAMS_t pll;                  // [], phase lock loop
    float bw_red_coeff;                // [%], bandwidth reduction coefficient applied to current loop (RFO) or flux/delta loops (SFO)
                                       // when running high frequency injection
    float lock_time;                   // [sec], lock time in sec
} HIGH_FREQ_INJ_PARAMS_t;

typedef struct
{
    bool overwrite;              // [], whether to overwrite params.motor.x with the results after being done
    float cmd_thresh;            // [%], command threshold for starting
    float cmd_hyst;              // [%], command hysteresis for stopping
    float i_cmd_dc;              // [A], dc current command
    float i_cmd_ac;              // [A], desired ac current command magnitude
    MINMAX_t w_cmd_elec;         // [Ra/sec-elec], speed command range for lam estimation
    float time_rot_lock;         // [sec], time spent for initial rotor locking
    float time_res;              // [sec], time spent for resistance estimation
    float time_ind;              // [sec], time spent for inductance estimation
    float time_spd;              // [sec], time spent at each speed point for estimating flux, inertia, friction, and voltage
                                 // parameters
    float w_h[PROF_FREQ_POINTS]; // [Ra/sec], excitation frequency list
    float w_sep;                 // [Ra/sec], seperation frequency, for separating dc and ac components
    float w0_idc;                // [Ra/sec], dc current controller's bandwidth
    float kp_idc;                // [V/A], dc current controller's kp
    float ki_idc;                // [(V/A).(Ra/sec)], dc current controller's ki
    float w0_flux;               // [Ra/sec], flux-estimation low-pass-filter's bandwidth

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
    uint16_t ramp_cnt;        // [#]
    float ramp_cnt_inv;       // [#]
    float ramp_main_bw_ratio; // [#], ramp to main PI controller bandwith ratio
    float ramp_kp;            // [V/A]
    float ramp_ki;            // [(V/A).(Ra/sec)]
    float ramp_ff_coef;       // [V/A], ramp PI feedforward coefficient
    float main_ff_coef;       // [#], main PI feedforward coefficient
} TRAP_COMM_PARAMS_t;
typedef struct
{
    TRAP_BLOCK_COMM_MODE_t mode;
    TRAP_COMM_PARAMS_t trap;
} TRAP_BLOCK_COMM_PARAMS_t;
#endif
#if defined(CTRL_METHOD_RFO)
typedef struct
{
    float bw;       // [Ra/sec], bandwidth
    float pole_sep; // [#]
    float kp;
    float ki;
    float ff_coef; // [#], feed forward coefficient
    float pi_output_limit;
} POSITION_CTRL_PARAMS_t;
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
#if defined(CTRL_METHOD_RFO)
    POSITION_CTRL_PARAMS_t position;
#endif
} CTRL_PARAMS_t;

typedef struct
{
    uint32_t code;
    uint16_t build_config; // changing build config should overwrite params
    uint16_t ver;          // 16 bits only for legacy reasons (compatibility with gui)
} PARAMS_ID_t;

typedef struct /* Enable/disable autocal for selected functions*/
{
    union
    {
        struct
        {
            uint32_t vdc_fault_threshold : 1; /* if Bit is set, auto calculation in PARAMS_InitAutoCalc() for current (d
                                                 & q) loop related parameter are skipped*/
            uint32_t speed_fault_threshold : 1;
            uint32_t speed_control : 1;
#if defined(CTRL_METHOD_SFO)
            uint32_t torque_control : 1;
#else
            uint32_t current_control : 1;
#endif
            uint32_t observer_pll : 1;
#if defined(CTRL_METHOD_RFO)
            uint32_t flux_weakening : 1;
            uint32_t position_control : 1;
#endif
#if defined(CTRL_METHOD_SFO)
            uint32_t flux_control : 1;
#endif
        };
        uint32_t word_access;
    };

} PARAM_AUTOCAL_DISABLE_t;

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

#pragma pack(push, 1)
typedef struct // For external identification (e.g. GUI)
{
    uint32_t chip_id;           // Chip ID
    uint16_t parameter_version; // Parameter version
    uint16_t firmware_version;  // Firmware version
    uint8_t kit_id;             // Kit ID
    uint8_t build_config_id;    // Build configuration ID
} MC_INFO_t;
#pragma pack(pop)

extern PARAMS_t params[MOTOR_CTRL_NO_OF_MOTOR];

extern LUT_PARAMS_t params_lut; // [#], luts

extern MC_INFO_t mc_info;

#if !defined(PC_TEST)
/**
 * @brief Initialize all motor parameters for an embedded target
 *
 * Calls PARAMS_InitManual() to load user-configured values, then calls
 * PARAMS_InitAutoCalc() to derive dependent parameters (loop gains, filter
 * coefficients, lookup tables, etc.).
 *
 * @param motor_ptr Pointer to motor instance
 */
void PARAMS_Init(MOTOR_t *motor_ptr);
#endif

extern void PARAMS_InitManual(PARAMS_t *params_ptr);
#if (MOTOR_CTRL_NO_OF_MOTOR > 1)
extern void PARAMS_InitManual_M1(PARAMS_t *params_ptr);
#endif
extern void PARAMS_InitAutoCalc(PARAMS_t *params_ptr);

/**
 * @brief Rebuild all runtime lookup tables from current parameters
 *
 * Re-computes interpolation tables (e.g. flux-weakening voltage vs speed,
 * MTPA current angle vs torque) whenever motor parameters change at runtime.
 */
void PARAMS_UpdateLookupTable(void);
