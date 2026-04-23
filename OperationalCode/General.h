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
 * @file General.h
 * @brief General definitions, macros, and utility functions
 *
 * Provides fundamental data types, coordinate transformations, mathematical utilities,
 * PI controllers, timers, filters, and common constants used throughout the motor
 * control library. Includes type definitions for electrical/mechanical quantities,
 * coordinate frame representations (UVW, AB, QD), and utility macros.
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>

#define POW_TWO(x) ((x) * (x))                                             /**< Square of x: x² */
#define POW_THREE(x) ((x) * (x) * (x))                                     /**< Cube of x: x³ */
#define MIN(x, y) (((x) < (y)) ? (x) : (y))                                /**< Minimum of two values */
#define MAX(x, y) (((x) < (y)) ? (y) : (x))                                /**< Maximum of two values */
#define SAT(xl, xh, x) (((x) < (xl)) ? (xl) : (((x) > (xh)) ? (xh) : (x))) /**< Saturate x to the range [xl, xh] */
#define ABS(x) (fabsf(x))                                                  /**< Absolute value (compiles to VABS.F32 on Cortex-M4) */
#define SIGN(x) (((x) >= 0.0f) ? (+1.0f) : (-1.0f))                        /**< Sign of x: +1 if x >= 0, -1 otherwise */
#define IS_POS(x) ((x) > 0.0f)                                             /**< True if x is strictly positive */
#define IS_NEG(x) ((x) < 0.0f)                                             /**< True if x is strictly negative */

#define RISE_EDGE(previous,current)		((!(previous))&&(current))
#define FALL_EDGE(previous,current)		((previous)&&(!(current)))
#define TRANS_EDGE(previous,current)	(RISE_EDGE(previous,current)||FALL_EDGE(previous,current)) 

#define AVE(x, y) (((x) + (y)) * (0.5f))                                             /**< Arithmetic mean of x and y */
#define ABS_BELOW_LIM(x, lim) (ABS(x) < (lim))                                       /**< True if |x| < lim (lim must be positive) */
#define ABS_ABOVE_LIM(x, lim) (ABS(x) > (lim))                                       /**< True if |x| > lim (lim must be positive) */
#define ROUND_FLOAT_TO_INT(x) ((int32_t)((x) + (((x) >= 0.0f) ? (+0.5f) : (-0.5f)))) /**< Round float to nearest int32 */
#define QUANTIZE_FLOAT(x, q) ((float)(ROUND_FLOAT_TO_INT((x) / (q))) * (q))          /**< Quantize x to the nearest multiple of q */

#define SQRT_TWO (1.414213562373095f)            /**< √2 */
#define SQRT_THREE (1.732050807568877f)          /**< √3 */
#define SQRT_THREE_OVER_TWO (0.866025403784439f) /**< √3 / 2 */
#define ONE_OVER_SQRT_TWO (0.707106781186547f)   /**< 1 / √2 */
#define ONE_OVER_SQRT_THREE (0.577350269189626f) /**< 1 / √3 */
#define TWO_OVER_SQRT_THREE (1.154700538379252f) /**< 2 / √3 */
#define EXP_ONE (2.718281828459046f)             /**< e (Euler's number) */
#define EXP_MINUS_ONE (0.367879441171442f)       /**< 1 / e */
#define LOG_TWO_E (1.442695040888963f)           /**< log₂(e) */
#define LOG_TEN_E (0.434294481903252f)           /**< log₁₀(e) */
#define LN_TWO (0.693147180559945f)              /**< ln(2) */
#define LN_TEN (2.302585092994046f)              /**< ln(10) */
#define TWO_PI (6.283185307179586f)              /**< 2π */
#define TWO_PI_OVER_THREE (2.094395102393195f)   /**< 2π / 3 */
#define FIVE_PI_OVER_SIX (2.617993877991494f)    /**< 5π / 6 */
#define PI (3.141592653589793f)                  /**< π */
#define PI_OVER_TWO (1.570796326794897f)         /**< π / 2 */
#define PI_OVER_THREE (1.047197551196598f)       /**< π / 3 */
#define PI_OVER_FOUR (0.785398163397448f)        /**< π / 4 */
#define PI_OVER_SIX (0.523598775598299f)         /**< π / 6 */
#define PI_OVER_TWELVE (0.261799387799149f)      /**< π / 12 */
#define ONE_OVER_PI (0.318309886183791f)         /**< 1 / π */
#define ONE_OVER_TWO_PI (0.159154943091895f)     /**< 1 / (2π) */
#define ONE_OVER_THREE_PI (0.106103295394597f)   /**< 1 / (3π) */
#define TWO_OVER_PI (0.636619772367581f)         /**< 2 / π */
#define THREE_OVER_PI (0.954929658551372f)       /**< 3 / π */
#define SIX_OVER_PI (1.909859317102744f)         /**< 6 / π */
#define EPSILON (1.0E-9f)                        /**< Small positive value for near-zero comparisons */

#define SCALE_PI_TO_INT32 ((float)(INT32_MAX) / PI)      /**< Scale factor: floating-point [0,π] → int32 full-scale */
#define SCALE_INT32_TO_DEG (180.0f / (float)(INT32_MAX)) /**< Scale factor: int32 → degrees */
#define SCALE_INT32_TO_PI (PI / (float)(INT32_MAX))      /**< Scale factor: int32 → radians */

#define LUT_1D_N (7)                 /**< Log2 of the 1D LUT table width */
#define LUT_1D_WIDTH (1 << LUT_1D_N) /**< Number of entries in a 1D LUT (128) */

#define TRIG_LUT_N (6)                   /**< Log2 of the trig LUT table width */
#define TRIG_LUT_WIDTH (1 << TRIG_LUT_N) /**< Number of entries in the trig LUT (64) */

#define INV_TRIG_LUT_N (5)                       /**< Log2 of the inverse-trig LUT table width */
#define INV_TRIG_LUT_WIDTH (1 << INV_TRIG_LUT_N) /**< Number of entries in the inverse-trig LUT (32) */

#define TEMP_SENS_LUT_N (4)                        /**< Log2 of the temperature sensor LUT table width */
#define TEMP_SENS_LUT_WIDTH (1 << TEMP_SENS_LUT_N) /**< Number of entries in the temperature sensor LUT (16) */

#define HALL_SIGNAL_PERMUTATIONS 8U /**< Total Hall signal permutations (6 valid out of 8 possible) */

#if defined(PC_TEST)
#define PROF_FREQ_POINTS 11U  /**< Number of profiler excitation frequency points (PC test build) */
#define PROF_SPEED_POINTS 11U /**< Number of profiler speed command points (PC test build) */
#else
#define PROF_FREQ_POINTS 25U  /**< Number of profiler excitation frequency points */
#define PROF_SPEED_POINTS 25U /**< Number of profiler speed command points */
#endif

#define HZ_TO_RADSEC(x) (TWO_PI * (x))                                                                   /**< Convert frequency [Hz] to angular frequency [rad/sec] */
#define RADSEC_TO_HZ(x) (ONE_OVER_TWO_PI * (x))                                                          /**< Convert angular frequency [rad/sec] to frequency [Hz] */
#define TAU_TO_RADSEC(x) (1.0f / (x))                                                                    /**< Convert time constant τ [sec] to ω0 [rad/sec] */
#define RADSEC_TO_TAU(x) (1.0f / (x))                                                                    /**< Convert ω0 [rad/sec] to time constant τ [sec] */
#define PERIOD_TO_RADSEC(x) (TWO_PI / (x))                                                               /**< Convert period [sec] to angular frequency [rad/sec] */
#define RADSEC_TO_PERIOD(x) (TWO_PI / (x))                                                               /**< Convert angular frequency [rad/sec] to period [sec] */
#define DISABLE_LPF_FS(x) HZ_TO_RADSEC((x) * ONE_OVER_TWO_PI)                                            /**< Set LPF cut-off to sampling frequency (effectively bypass) */
#define MECH_TO_ELEC(x, p) ((x) * (0.5f) * (p))                                                          /**< Convert mechanical angle/speed to electrical (p = pole count) */
#define ELEC_TO_MECH(x, p) ((x) * (2.0f) / (p))                                                          /**< Convert electrical angle/speed to mechanical (p = pole count) */
#define RPM_TO_HZ(x) ((x) * (1.0f / 60.0f))                                                              /**< Convert speed [RPM] to frequency [Hz] */
#define HZ_TO_RPM(x) ((x) * (60.0f))                                                                     /**< Convert frequency [Hz] to speed [RPM] */
#define DEG_TO_RAD(x) ((x) * (PI / 180.0f))                                                              /**< Convert degrees to radians */
#define RAD_TO_DEG(x) ((x) * (180.0f / PI))                                                              /**< Convert radians to degrees */
#define RMS_TO_PK(x) ((x) * SQRT_TWO)                                                                    /**< Convert RMS value to peak value (multiply by √2) */
#define PK_TO_RMS(x) ((x) * ONE_OVER_SQRT_TWO)                                                           /**< Convert peak value to RMS value (divide by √2) */
#define PHASE_TO_LINE(x) ((x) * SQRT_THREE)                                                              /**< Convert phase voltage to line voltage (multiply by √3) */
#define LINE_TO_PHASE(x) ((x) * ONE_OVER_SQRT_THREE)                                                     /**< Convert line voltage to phase voltage (divide by √3) */
#define BIT_TO_FLOAT(word, bit_mask, one_val, zero_val) (((word) & (bit_mask)) ? (one_val) : (zero_val)) /**< Return one_val if bit_mask is set in word, else zero_val */
#define PERC_TO_NORM(x) ((x) * 0.01f)                                                                    /**< Convert percentage [%] to normalized [0–1] */
#define NORM_TO_PERC(x) ((x) * 100.0f)                                                                   /**< Convert normalized [0–1] to percentage [%] */

#define BYTE_TO_WORD(byte,index)    ((byte)<<((index)*8U))
#define WORD_TO_BYTE(word,index)    (((word)>>((index)*8U))&(0xFF))
#define THREE_BYTES_TO_WORD(byte0,byte1,byte2)  (BYTE_TO_WORD(byte0,0U)|BYTE_TO_WORD(byte1,1U)|BYTE_TO_WORD(byte2,2U))

#define STRUCT_TO_ARRAY(instance) ((float *)(&instance)) /**< Cast a struct instance to a float pointer array (use with caution) */

/*Note: RAMFUNC_ENABLE macro definition is done in makefile*/
#if !defined(PC_TEST) && defined(RAMFUNC_ENABLE)
#include "cy_utils.h"
#define RAMFUNC_BEGIN CY_RAMFUNC_BEGIN /**< Begin a RAM-resident function section (Cypress toolchain) */
#define RAMFUNC_END CY_RAMFUNC_END     /**< End a RAM-resident function section (Cypress toolchain) */
#else
#define RAMFUNC_BEGIN /**< No-op when RAM function placement is not enabled */
#define RAMFUNC_END   /**< No-op when RAM function placement is not enabled */
#endif

#ifndef STATIC_ASSERT
#if defined(__cplusplus)
#define STATIC_ASSERT(cond, msg) static_assert(cond, msg)   /**< Compile-time assertion (C++ keyword) */
#else
#define STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)  /**< Compile-time assertion (C11 keyword, no header needed) */
#endif
#endif

/*MOTOR_CTRL_MOTOR0_ENABLED and MOTOR_CTRL_MOTOR1_ENABLED macros maintained for backward compatibility*/
#ifndef MOTOR_CTRL_MOTOR0_ENABLED
#define MOTOR_CTRL_MOTOR0_ENABLED (1U)                         /**< Motor 0 is always present (minimum one motor required) */
#endif
#ifndef MOTOR_CTRL_MOTOR1_ENABLED
#define MOTOR_CTRL_MOTOR1_ENABLED  (MOTOR_CTRL_NO_OF_MOTOR >1) /**< Motor 1 is present when two or more motors are configured */
#endif

#pragma pack(push,4)

/**
 * @brief Three-phase UVW quantity (e.g. phase currents or voltages)
 */
typedef struct
{
    float u; /**< U (phase A) value */
    float v; /**< V (phase B) value */
    float w; /**< W (phase C) value */
} UVW_t;

/**
 * @brief Generic three-element XYZ quantity
 */
typedef struct
{
    float x; /**< X (first) element */
    float y; /**< Y (second) element */
    float z; /**< Z (third) element */
} XYZ_t;

/**
 * @brief Two-phase αβ (Clarke-frame) quantity
 */
typedef struct
{
    float alpha; /**< α (alpha) component */
    float beta;  /**< β (beta) component */
} AB_t;

/**
 * @brief Rotating QD (Park-frame) quantity
 */
typedef struct
{
    float q; /**< Q-axis (torque-producing) component */
    float d; /**< D-axis (flux-producing) component */
} QD_t;

/**
 * @brief Scalar minimum / maximum bounds pair
 */
typedef struct
{
    float min; /**< Minimum bound */
    float max; /**< Maximum bound */
} MINMAX_t;

/**
 * @brief Pre-computed Park-transform sin/cos pair
 *
 * Populated by ParkInit() from an electrical angle; consumed by
 * ParkTransform() and ParkTransformInv().
 */
typedef struct
{
    float sine;   /**< Sine of the electrical angle */
    float cosine; /**< Cosine of the electrical angle */
} PARK_t;

/**
 * @brief Polar coordinate representation (magnitude and angle)
 */
typedef struct
{
    float rad;   /**< Magnitude (radius) */
    float theta; /**< Angle [rad] */
} POLAR_t;

#pragma pack(pop)

/**
 * @brief One-dimensional lookup table with linear interpolation
 */
typedef struct
{
    float x_min;           /**< Minimum input x (table start) */
    float x_max;           /**< Maximum input x (table end) */
    float x_step;          /**< Step size: (x_max - x_min) / (LUT_1D_WIDTH - 1) */
    float x_step_inv;      /**< Reciprocal of x_step */
    float y[LUT_1D_WIDTH]; /**< Output y values at each table entry */
} LUT_1D_t;

/**
 * @brief PI controller with back-calculation anti-windup (H(s) = kp + ki/s)
 */
typedef struct
{
    // PI with back-calculation type anti-windup
    // H(s)=kp+ki/s
    float kp;         /**< Proportional gain */
    float ki;         /**< Integral gain (discrete: ki_continuous * Ts) */
    float output_min; /**< Minimum output clamp limit */
    float output_max; /**< Maximum output clamp limit */

    float integ; /**< Integrator state */

    float ff;     /**< Feed-forward term */
    float error;  /**< Last computed error (cmd - fb) */
    float output; /**< Controller output after clamping */
} PI_t;

/**
 * @brief Bilinear (trapezoidal) integrator state
 */
typedef struct
{
    float prev_input; /**< Input value from the previous step */
    float integ;      /**< Integrator accumulator */
} BILINEAR_INTEG_t;

/**
 * @brief Motor magnet configuration type
 */
typedef enum
{
    SPM = 0, /**< Surface Permanent Magnet: Ld ≈ Lq */
    IPM = 1, /**< Interior Permanent Magnet: Ld < Lq */
} MOTOR_TYPE_t;

/**
 * @brief Enable / disable selector
 */
typedef enum
{
    Dis = 0, /**< Disabled */
    En = 1,  /**< Enabled */
} EN_DIS_t;

/**
 * @brief Asynchronous task execution status
 */
typedef enum
{
    Task_Waiting = 0U, /**< Task queued but not yet started */
    Task_Started,      /**< Task currently executing */
    Task_Finished,     /**< Task completed successfully */
    Task_Error         /**< Task completed with an error */
} TASK_STATUS_t;

/**
 * @brief Speed / control bandwidth response preset
 */
typedef enum
{
    Slow = 0U, /**< Low bandwidth — stable, less responsive */
    Moderate,  /**< Medium bandwidth */
    Fast       /**< High bandwidth — fast response */
} SPEED_ATTRIB_t;

/**
 * @brief Position / speed feedback sensor mode
 */
typedef enum
{
    Sensorless = 0, /**< Sensorless observer or high-frequency injection */
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    Hall = 1,    /**< Hall-effect sensor */
    AqB_Enc = 2, /**< A/B quadrature encoder */
    Direct = 3,  /**< Direct analog or digital feedback */
#endif
} FB_MODE_t;

/**
 * @brief Electrical-domain scalar (e.g. electrical angle [rad-elec] or speed [rad/sec-elec])
 */
typedef struct
{
    float elec; /**< Electrical domain value */
} ELEC_t;

/**
 * @brief Mechanical-domain scalar (e.g. mechanical angle [rad-mech] or speed [rad/sec-mech])
 */
typedef struct
{
    float mech; /**< Mechanical domain value */
} MECH_t;

/**
 * @brief Combined electrical and mechanical scalar pair
 */
typedef struct
{
    float elec; /**< Electrical domain value */
    float mech; /**< Mechanical domain value */
} ELEC_MECH_t;

/**
 * @brief Trigonometric (sin/cos) lookup table over [0, π/2]
 */
typedef struct
{
    // theta is in [0,pi/2]
    float th_step;             /**< Angular step size per entry [rad] */
    float th_step_inv;         /**< Reciprocal of th_step */
    float val[TRIG_LUT_WIDTH]; /**< Tabulated sin values for theta in [0, π/2] */
} TRIG_LUT_t;

/**
 * @brief Inverse trigonometric (atan/asin) lookup table over input [0, 1]
 */
typedef struct
{
    // input is in [0,1]
    // output is in [0, pi/4] for atan and [0, pi/2] for asin
    float step;                    /**< Input step size per entry */
    float step_inv;                /**< Reciprocal of step */
    float val[INV_TRIG_LUT_WIDTH]; /**< Tabulated inverse-trig output values */
} INV_TRIG_LUT_t;

/**
 * @brief Temperature sensor linearization lookup table
 *
 * Input is in [step, 1-step] to avoid asymptotic endpoints.
 */
typedef struct
{
    // input is in [step,1-step], excluding {0,1} asymptotic infinites
    float step;                     /**< Input step size per entry */
    float step_inv;                 /**< Reciprocal of step */
    float val[TEMP_SENS_LUT_WIDTH]; /**< Tabulated temperature conversion output values */
} TEMP_SENS_LUT_t;

/**
 * @brief Stopwatch / debounce counter timer
 */
typedef struct
{
    float time_thresh;          /**< Timeout threshold [sec] */
    float run_period;           /**< Period of one StopWatchRun tick [sec] */
    uint32_t time_thresh_ticks; /**< Threshold expressed as an integer tick count */
    uint32_t time_ticks;        /**< Current elapsed tick count */
} TIMER_t;

/**
 * @brief Incremental least-squares linear regression (y = mx + c)
 */
typedef struct
{
    float cnt;      /**< Number of accumulated data points */
    float sigma_x;  /**< Sum of x samples: Σx */
    float sigma_xx; /**< Sum of squared x samples: Σx² */
    float sigma_xy; /**< Sum of x·y products: Σxy */
    float sigma_y;  /**< Sum of y samples: Σy */
    float sigma_yy; /**< Sum of squared y samples: Σy² */
    float var_xx;   /**< x variance scaled by N² */
    float var_yy;   /**< y variance scaled by N² */
    float cov_xy;   /**< x–y covariance scaled by N² */
    float m;        /**< Fitted slope (y = mx + c) */
    float c;        /**< Fitted y-intercept (y = mx + c) */
    float r;        /**< Pearson correlation coefficient */
} LIN_REG_t;

/**
 * @brief Pseudo-Random Binary Sequence (PRBS) generator state
 *
 * Implements a Galois-form maximal-length LFSR for generating repeatable
 * binary noise used in frequency-response measurement and profiling.
 */
typedef struct
{                      // Pseudo Random Binary Sequence (PRBS)
    uint8_t order;     /**< Shift-register polynomial order (period = 2^order - 1) */
    uint8_t term[4U];  /**< Feedback tap positions (polynomial terms) */
    uint8_t shift[4U]; /**< Bit shifts corresponding to each tap */
    uint32_t mask[4U]; /**< Bit masks corresponding to each tap */
    uint32_t period;   /**< PRBS sequence period: 2^order - 1 */
    uint32_t lfsr;     /**< Current Linear Feedback Shift Register state */
} PRBS_t;
extern UVW_t UVW_Zero;
extern UVW_t UVW_One;
extern UVW_t UVW_Half;
extern AB_t AB_Zero;
extern QD_t QD_Zero;
extern MINMAX_t MinMax_Zero;
extern PARK_t Park_Zero;
extern POLAR_t Polar_Zero;
extern ELEC_t Elec_Zero;
extern ELEC_t Mech_Zero;
extern ELEC_MECH_t ElecMech_Zero;

/** @brief No-op callback for void() function pointers. */
void EmptyFcn();
/** @brief No-op callback for void(uint8_t) function pointers.
 *  @param id Unused motor instance ID */
void EmptyFcn_OneArgument(uint8_t id);
/** @brief No-op callback for void(void*) function pointers.
 *  @param ptr Unused pointer argument */
void EmptyFcn_PtrArgument(void *ptr);

/** @brief Callback stub that unconditionally returns true.
 *         Used as the default for bool-returning function pointers.
 *  @param id Unused motor instance ID
 *  @return Always true */
bool AlwaysTrue(uint8_t id);

/** @brief Reset PI controller state to zero (integrator, error, output).
 *  @param pi Pointer to PI_t structure */
void PI_Reset(PI_t *pi);
/** @brief Update PI controller tuning parameters and output clamp limits.
 *  @param pi         Pointer to PI_t structure
 *  @param kp         Proportional gain
 *  @param ki         Discrete integral gain (= ki_continuous * Ts)
 *  @param output_min Lower output saturation limit
 *  @param output_max Upper output saturation limit */
void PI_UpdateParams(PI_t *pi, const float kp, const float ki, const float output_min, const float output_max);
/** @brief Run one step of the PI controller with feed-forward and clamping anti-windup.
 *         Computes error = cmd - fb, accumulates integrator (with clamp), and
 *         saturates output to [output_min, output_max].
 *  @param pi  Pointer to PI_t structure
 *  @param cmd Reference command
 *  @param fb  Feedback measurement
 *  @param ff  Feed-forward term added directly to the output */
void PI_Run(PI_t *pi, const float cmd, const float fb, const float ff); // with feed forward
/** @brief Pre-load PI internal state for a given output (bumpless transfer).
 *         Back-calculates the integrator value so the controller output matches
 *         the provided output immediately on the next PI_Run call.
 *  @param pi     Pointer to PI_t structure
 *  @param output Desired output value to pre-load
 *  @param error  Error at the pre-load instant
 *  @param ff     Feed-forward term at the pre-load instant */
void PI_IntegBackCalc(PI_t *pi, const float output, const float error, const float ff);

/** @brief Reset bilinear (trapezoidal) integrator to an initial value.
 *  @param bilinear  Pointer to BILINEAR_INTEG_t structure
 *  @param integ_val Initial integrator state */
void BILINEAR_INTEG_Reset(BILINEAR_INTEG_t *bilinear, const float integ_val);
/** @brief Advance bilinear integrator by one step: integ += (prev + input) / 2.
 *  @param bilinear Pointer to BILINEAR_INTEG_t structure
 *  @param input    Current input sample
 *  @return Updated integrator value */
float BILINEAR_INTEG_Run(BILINEAR_INTEG_t *bilinear, const float input);

/** @brief Forward Clarke transform: three-phase UVW → two-phase alpha-beta.
 *         Power-invariant form: alpha = u, beta = (w - v) / sqrt(3).
 *  @param input  Pointer to three-phase UVW quantities
 *  @param output Pointer to two-phase AB result */
void ClarkeTransform(const UVW_t *input, AB_t *output);
/** @brief Inverse Clarke transform: alpha-beta → three-phase UVW.
 *  @param input  Pointer to two-phase AB quantities
 *  @param output Pointer to three-phase UVW result */
void ClarkeTransformInv(const AB_t *input, UVW_t *output);

/** @brief Pre-compute sin/cos for a given electrical angle (LUT-based).
 *         Must be called before ParkTransform or ParkTransformInv.
 *  @param angle Electrical angle [rad-elec]
 *  @param park  Pointer to PARK_t structure to populate */
void ParkInit(const float angle, PARK_t *park);
/** @brief Forward Park transform: stationary AB → rotating QD frame.
 *         q = alpha*cos - beta*sin;  d = alpha*sin + beta*cos.
 *  @param input  Pointer to stationary-frame AB quantities
 *  @param park   Pointer to pre-computed PARK_t (from ParkInit)
 *  @param output Pointer to rotating-frame QD result */
void ParkTransform(const AB_t *input, const PARK_t *park, QD_t *output);
/** @brief Inverse Park transform: rotating QD → stationary AB frame.
 *  @param input  Pointer to rotating-frame QD quantities
 *  @param park   Pointer to pre-computed PARK_t (from ParkInit)
 *  @param output Pointer to stationary-frame AB result */
void ParkTransformInv(const QD_t *input, const PARK_t *park, AB_t *output);

/** @brief Four-quadrant arctangent via LUT (equivalent to atan2).
 *  @param y Sine-proportional component
 *  @param x Cosine-proportional component
 *  @return Angle [rad] in (-pi, pi] */
float ATan2(const float y, const float x);
/** @brief Arcsine via LUT. Output is in [-pi/2, +pi/2].
 *  @param y Input value in [-1, 1]
 *  @return Angle [rad] in [-pi/2, +pi/2] */
float ASin(const float y); // output is in [-pi/2,+pi/2]
/** @brief Arccosine via LUT. Output is in [0, +pi].
 *  @param x Input value in [-1, 1]
 *  @return Angle [rad] in [0, +pi] */
float ACos(const float x); // output is in [0,+pi]

/** @brief Convert Cartesian coordinates to polar form.
 *  @param x     X-component
 *  @param y     Y-component
 *  @param polar Output polar structure (rad = magnitude, theta = angle [rad]) */
void ToPolar(const float x, const float y, POLAR_t *polar);

/** @brief Linear interpolation from a 1D lookup table.
 *         Clamps to endpoint values outside the table range.
 *  @param lut   Pointer to LUT_1D_t structure
 *  @param input Input x-value
 *  @return Interpolated y-value */
float LUT1DInterp(const LUT_1D_t *lut, const float input);

/** @brief Evaluate y = slope * x + intercept.
 *  @param slope     Line slope
 *  @param intercept Y-axis intercept
 *  @param x         Input value
 *  @return Result y */
float SlopeIntercept(const float slope, const float intercept, const float x);

/** @brief Linearly blend two scalars: x = ratio*x1 + (1-ratio)*x2.
 *         ratio must be in [0, 1].
 *  @param ratio Blend ratio (1.0 = pure x1, 0.0 = pure x2)
 *  @param x1   First value
 *  @param x2   Second value
 *  @param x    Output blended value */
void ScalarBlend(const float ratio, const float x1, const float x2, float *x); // ratio must be within [0-1]
/** @brief Blend two angles handling the ±pi wrap boundary.
 *         Angles must be in [-pi, pi).
 *  @param ratio Blend ratio (1.0 = pure th1, 0.0 = pure th2)
 *  @param th1   First angle [rad]
 *  @param th2   Second angle [rad]
 *  @param th    Output blended angle [rad], wrapped to [-pi, pi) */
void AngleBlend(const float ratio, const float th1, const float th2, float *th); // angles must be within [-pi,pi)
/** @brief Blend two polar vectors (radius and angle independently).
 *  @param ratio   Blend ratio
 *  @param polar1  First polar vector
 *  @param polar2  Second polar vector
 *  @param result  Output blended polar vector */
void PolarBlend(const float ratio, const POLAR_t *polar1, const POLAR_t *polar2, POLAR_t *result);
/** @brief Wrap an angle to the half-open range [-pi, pi).
 *  @param th Input angle [rad]
 *  @return Angle [rad] wrapped to [-pi, pi) */
float Wrap2Pi(const float th); // in range [-pi,pi)

/** @brief Apply a symmetric rate limiter: limits change per step to \p rate.
 *         rate must be positive; limits both positive and negative slopes.
 *  @param rate    Maximum allowed change per ISR step
 *  @param target  Desired final value
 *  @param current Current value
 *  @return New value no more than \p rate away from \p current */
float RateLimit(const float rate, const float target, const float current);	// rate must be positive, limits positive and negative slopes

/** @brief Sort three UVW duties into descending XYZ order and return index mappings.
 *         Assuming X>=Y>=Z, builds two packed byte-index words for converting
 *         between UVW and XYZ orderings (used for single-shunt ADC scheduling).
 *  @param uvw     Pointer to UVW duties to sort
 *  @param xyz_idx Packed byte-indices: byte[n] = UVW index of the n-th largest value
 *  @param uvw_idx Packed byte-indices: byte[n] = XYZ position of the n-th UVW element */
void SortUVW(UVW_t *uvw, uint32_t *xyz_idx, uint32_t *uvw_idx);

/** @brief Initialise a stopwatch timer.
 *  @param timer       Pointer to TIMER_t structure
 *  @param time_thresh Threshold duration [sec]
 *  @param run_period  Period of one StopWatchRun increment [sec] */
void StopWatchInit(TIMER_t *timer, const float time_thresh, const float run_period);
/** @brief Reset the stopwatch tick counter to zero (restart from beginning).
 *  @param timer Pointer to TIMER_t structure */
void StopWatchReset(TIMER_t *timer); // resets the timer
/** @brief Increment the stopwatch by one tick; saturates at the threshold.
 *         Should be called once per ISR execution.
 *  @param timer Pointer to TIMER_t structure */
void StopWatchRun(TIMER_t *timer);
/** @brief Check whether the stopwatch has elapsed its threshold.
 *  @param timer Pointer to TIMER_t structure
 *  @return true if time_ticks >= time_thresh_ticks */
bool StopWatchIsDone(TIMER_t *timer); // returns true if it is time
/** @brief Return elapsed time expressed in seconds.
 *  @param timer Pointer to TIMER_t structure
 *  @return Elapsed time [sec] */
float StopWatchGetTime(TIMER_t *timer); // get time in [sec]

extern void (*const DebounceFiltInit)(TIMER_t *timer, const float time_thresh, const float run_period);
extern void (*const DebounceFiltReset)(TIMER_t *timer);
extern void (*const DebounceFiltInc)(TIMER_t *timer);
extern bool (*const DebounceFiltIsSet)(TIMER_t *timer);
extern float (*const DebounceFiltGetTime)(TIMER_t *timer);
/** @brief Decrement debounce filter counter, saturating at zero.
 *  @param timer Pointer to TIMER_t structure */
void DebounceFiltDec(TIMER_t *timer);
/** @brief Check whether the debounce filter counter is at zero (cleared).
 *  @param timer Pointer to TIMER_t structure
 *  @return true if counter == 0 */
bool DebounceFiltIsClear(TIMER_t *timer);

/** @brief Reset linear regression accumulators to zero.
 *  @param lin_reg Pointer to LIN_REG_t structure */
void LinearRegressionReset(LIN_REG_t *lin_reg);
/** @brief Add one (x, y) sample to the regression accumulators.
 *  @param lin_reg Pointer to LIN_REG_t structure
 *  @param x       Independent variable sample
 *  @param y       Dependent variable sample */
void LinearRegressionAddDataPoint(LIN_REG_t *lin_reg, const float x, const float y);
/** @brief Compute slope, intercept and Pearson correlation from accumulated data.
 *         Results stored in lin_reg->m (slope), lin_reg->c (intercept),
 *         lin_reg->r (correlation coefficient).
 *  @param lin_reg Pointer to LIN_REG_t structure */
void LinearRegressionProcessData(LIN_REG_t *lin_reg);

/** @brief Initialise a Galois-form maximal-length PRBS generator.
 *         Configures the LFSR polynomial for the requested order (2–16);
 *         period = 2^order - 1.
 *  @param prbs  Pointer to PRBS_t structure
 *  @param order Shift-register order (2–16) */
void PseudoRandBinaryInit(PRBS_t *prbs, const uint8_t order);
/** @brief Generate the next PRBS output bit.
 *  @param prbs Pointer to PRBS_t structure
 *  @return Next pseudo-random bit (0 or 1) */
bool PseudoRandBinaryGen(PRBS_t *prbs);
