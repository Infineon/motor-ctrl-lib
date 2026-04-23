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
 * @file SixPulseInj.c
 * @brief Six-pulse injection implementation
 *
 * Implements six-pulse voltage injection for initial rotor position detection.
 */

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

#include "Controller.h"

#define LABEL 0
#define WVU 1
#define TABLE_DEPTH 2

int8_t Pulse_Table[TABLE_DEPTH][SIX_PULSE] = \
// This pattern is chosen to minimize rotor movement during detection
// Indices:				Time Sequence:		1    ->2    ->3    ->4    ->5    ->6
// First Row (Labels):	Space Vector:		1    ->4    ->2    ->5    ->3    ->6
// Second Row (Values):	WVU Binary:			1    ->6    ->3    ->4    ->2    ->5
// Resulting Current:						(+U) ->(-U) ->(-W) ->(+W) ->(+V) ->(-V)
{   {   0x1,    0x4,    0x2,    0x5,    0x3,    0x6},
    {   0b001,  0b110,  0b011,  0b100,  0b010,  0b101   }   };

/** @brief Initialise six-pulse injection: configure pulse timers and compute PWM duty cycles.
 *         Computes d_pos/d_neg from v_pulse normalised to vdc_nom, then calls Reset.
 *  @param motor_ptr  Pointer to the motor instance */
void SIX_PULSE_INJ_Init(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    // Configure ON and OFF stopwatch timers from parameter-defined durations
    StopWatchInit(&ctrl_ptr->six_pulse_inj.timer_on, params_ptr->ctrl.six_pulse_inj.t_on, params_ptr->sys.samp.ts0);
    StopWatchInit(&ctrl_ptr->six_pulse_inj.timer_off, params_ptr->ctrl.six_pulse_inj.t_off, params_ptr->sys.samp.ts0);

    // Compute PWM duty cycles from the desired pulse voltage normalised to DC bus:
    //   d_vdc  = v_pulse / vdc_nom  (active duty for the energised phase)
    //   d_0    = 1 - d_vdc          (remaining duty shared equally as zero-vector)
    //   d_pos  = d_vdc + d_0/2      (duty for the +V phase)
    //   d_neg  = d_0/2              (duty for the -V phase)
    float d_vdc = SAT(0.0f, 1.0f, params_ptr->ctrl.six_pulse_inj.v_pulse / params_ptr->sys.vdc_nom);
    float d_0 = 1.0f - d_vdc;
    ctrl_ptr->six_pulse_inj.d_pos = d_vdc + 0.5f * d_0;
    ctrl_ptr->six_pulse_inj.d_neg = 0.5f * d_0;

    SIX_PULSE_INJ_Reset(motor_ptr);
}

/** @brief Reset six-pulse injection state machine to Not_Started.
 *  @param motor_ptr  Pointer to the motor instance */
void SIX_PULSE_INJ_Reset(MOTOR_t *motor_ptr)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    ctrl_ptr->six_pulse_inj.state = Not_Started;
}

static inline void ResetLabels(CTRL_t *ctrl_ptr)
{
    for (int8_t index = 0; index < SIX_PULSE; ++index)
    {
        int8_t wvu = Pulse_Table[LABEL][index];
        ctrl_ptr->six_pulse_inj.i_peak.label[index] = wvu;
        ctrl_ptr->six_pulse_inj.k.label[index] = wvu;
    }
}

/** @brief Six-pulse injection ISR0 state machine: sequence voltage pulses and sample current.
 *         Cycles through six pulses.  For each pulse arms ON/OFF timers, samples peak current
 *         on the first decay cycle, then accumulates a least-squares inverse-decay-rate estimate:
 *         k = Σ(i[n-1]²) / Σ(i[n]·i[n-1]) = 1/K where K is the decay ratio from §6.11.2.2 Eq.266.
 *         Higher k = lower K = more saturated direction (d-axis alignment).
 *         On the final pulse transitions the state to Processing_Results.
 *  @param motor_ptr  Pointer to the motor instance */
RAMFUNC_BEGIN
void SIX_PULSE_INJ_RunISR0(MOTOR_t *motor_ptr)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    int8_t wvu;
    switch (ctrl_ptr->six_pulse_inj.state)
    {
    case Not_Started:
        // --- Arm first pulse ---
        // Reset the ON timer and clear all measurement labels/results
        StopWatchReset(&ctrl_ptr->six_pulse_inj.timer_on);
        ResetLabels(ctrl_ptr);
        ctrl_ptr->six_pulse_inj.th_r_est.elec = 0.0f;
        ctrl_ptr->six_pulse_inj.pulse_num = 0;
        // Apply the duty cycle for pulse 0 to all three phases using the WVU bit pattern
        wvu = Pulse_Table[WVU][ctrl_ptr->six_pulse_inj.pulse_num];
        vars_ptr->d_uvw_cmd.u = BIT_TO_FLOAT(wvu, 0b001, ctrl_ptr->six_pulse_inj.d_pos, ctrl_ptr->six_pulse_inj.d_neg);
        vars_ptr->d_uvw_cmd.v = BIT_TO_FLOAT(wvu, 0b010, ctrl_ptr->six_pulse_inj.d_pos, ctrl_ptr->six_pulse_inj.d_neg);
        vars_ptr->d_uvw_cmd.w = BIT_TO_FLOAT(wvu, 0b100, ctrl_ptr->six_pulse_inj.d_pos, ctrl_ptr->six_pulse_inj.d_neg);
        vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
        ctrl_ptr->six_pulse_inj.state = In_Progress_On_Pulse;
        break;
    case In_Progress_On_Pulse:
        // --- Wait for the voltage pulse to fully energise the phase ---
        StopWatchRun(&ctrl_ptr->six_pulse_inj.timer_on);
        if (StopWatchIsDone(&ctrl_ptr->six_pulse_inj.timer_on))
        {
            // Pulse ON complete: switch off inverter and arm the OFF (decay) timer
            StopWatchReset(&ctrl_ptr->six_pulse_inj.timer_off);
            ctrl_ptr->six_pulse_inj.k_num = 0.0f;
            ctrl_ptr->six_pulse_inj.k_den = 0.0f;
            vars_ptr->d_uvw_cmd = UVW_Zero;
            vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
            ctrl_ptr->six_pulse_inj.state = In_Progress_Off_Pulse;
            // Record the per-phase sign so that all three currents can be
            // projected onto a single scalar axis aligned with this pulse
            wvu = Pulse_Table[WVU][ctrl_ptr->six_pulse_inj.pulse_num];
            ctrl_ptr->six_pulse_inj.i_uvw_sign.u = BIT_TO_FLOAT(wvu, 0b001, +1.0f, -1.0f);
            ctrl_ptr->six_pulse_inj.i_uvw_sign.v = BIT_TO_FLOAT(wvu, 0b010, +1.0f, -1.0f);
            ctrl_ptr->six_pulse_inj.i_uvw_sign.w = BIT_TO_FLOAT(wvu, 0b100, +1.0f, -1.0f);
        }
        break;
    case In_Progress_Off_Pulse:
        // Project the three-phase current feedback onto the pulse axis using
        // the pre-computed sign vector; factor 0.5 accounts for the two active phases
        ctrl_ptr->six_pulse_inj.i_fb = 0.5f * (vars_ptr->i_uvw_fb.u * ctrl_ptr->six_pulse_inj.i_uvw_sign.u + vars_ptr->i_uvw_fb.v * ctrl_ptr->six_pulse_inj.i_uvw_sign.v + vars_ptr->i_uvw_fb.w * ctrl_ptr->six_pulse_inj.i_uvw_sign.w);
        if (ctrl_ptr->six_pulse_inj.timer_off.time_ticks == 0U) // first cycle, sample peak current
        {
            // Record the peak current immediately after the pulse ends (maximum induction response)
            ctrl_ptr->six_pulse_inj.i_peak.value[ctrl_ptr->six_pulse_inj.pulse_num] = ctrl_ptr->six_pulse_inj.i_fb;
        }
        else // next cycles, calculate exponential decay (least squares method)
        {
            // Accumulate least-squares numerator (i[n-1]^2) and denominator (i[n]*i[n-1])
            // Optimal k = sum(i[n-1]^2) / sum(i[n]*i[n-1]) for i[n] = k * i[n-1]
            ctrl_ptr->six_pulse_inj.k_num += POW_TWO(ctrl_ptr->six_pulse_inj.i_fb_prev);
            ctrl_ptr->six_pulse_inj.k_den += ctrl_ptr->six_pulse_inj.i_fb * ctrl_ptr->six_pulse_inj.i_fb_prev;
        }
        ctrl_ptr->six_pulse_inj.i_fb_prev = ctrl_ptr->six_pulse_inj.i_fb;
        StopWatchRun(&ctrl_ptr->six_pulse_inj.timer_off);
        if (StopWatchIsDone(&ctrl_ptr->six_pulse_inj.timer_off))
        {
            // Store the final decay rate estimate for this pulse
            ctrl_ptr->six_pulse_inj.k.value[ctrl_ptr->six_pulse_inj.pulse_num] = ctrl_ptr->six_pulse_inj.k_num / ctrl_ptr->six_pulse_inj.k_den;
            if (ctrl_ptr->six_pulse_inj.pulse_num < (SIX_PULSE - 1))
            {
                // More pulses remaining: arm the next ON pulse
                StopWatchReset(&ctrl_ptr->six_pulse_inj.timer_on);
                ++ctrl_ptr->six_pulse_inj.pulse_num; // S061987
                wvu = Pulse_Table[WVU][ctrl_ptr->six_pulse_inj.pulse_num];
                vars_ptr->d_uvw_cmd.u = BIT_TO_FLOAT(wvu, 0b001, ctrl_ptr->six_pulse_inj.d_pos, ctrl_ptr->six_pulse_inj.d_neg);
                vars_ptr->d_uvw_cmd.v = BIT_TO_FLOAT(wvu, 0b010, ctrl_ptr->six_pulse_inj.d_pos, ctrl_ptr->six_pulse_inj.d_neg);
                vars_ptr->d_uvw_cmd.w = BIT_TO_FLOAT(wvu, 0b100, ctrl_ptr->six_pulse_inj.d_pos, ctrl_ptr->six_pulse_inj.d_neg);
                vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
                ctrl_ptr->six_pulse_inj.state = In_Progress_On_Pulse;
            }
            else
            {
                // All six pulses collected — hand off to ISR1 for result processing
                ctrl_ptr->six_pulse_inj.state = Processing_Results;
            }
        }
        break;
    case Processing_Results:
    case Finished_Success:
    case Finished_Ambiguous:
    default:
        break;
    }
}
RAMFUNC_END

// Only sorting top two pulses
// Finds the label with the maximum value, then selects the better of its
// two immediate neighbours (sector ±1) as the second candidate.
static void ExtractTwoLabels(SIX_PULSE_INJ_DATA_t *data, int8_t results[TWO_PULSE])
{
    int8_t max_index = 0; // Index correspnding to max. value
    for (int8_t index = 1; index < SIX_PULSE; ++index)
    {
        if (data->value[index] > data->value[max_index])
        {
            max_index = index;
        }
    }
    results[0] = data->label[max_index]; // Label corresponding to max. value

    // Find the two labels adjacent (±1 sector) to the winning label
    int8_t adj_labels[2];           // Adjacent labels to max. label (results[0])
    int8_t adj_indices[2] = {0, 0}; // Adjacent indices corresponding to adjacent labels
    adj_labels[0] = (results[0]) % SIX_PULSE + 1;
    adj_labels[1] = (results[0] + (SIX_PULSE - 2)) % SIX_PULSE + 1;
    for (int8_t index = 0; index < SIX_PULSE; ++index)
    {
        if (data->label[index] == adj_labels[0])
        {
            adj_indices[0] = index;
        }
        else if (data->label[index] == adj_labels[1])
        {
            adj_indices[1] = index;
        }
    }

    // Pick whichever adjacent label has the higher value as the second result
    if (data->value[adj_indices[0]] >= data->value[adj_indices[1]])
    {
        results[1] = data->label[adj_indices[0]];
    }
    else
    {
        results[1] = data->label[adj_indices[1]];
    }
}

// Converts 1-based circular sector labels to 0-based unwrapped indices,
// handling the wrap-around at the boundary between sector 6 and sector 1.
static inline void UnWrapLabels(CTRL_t *ctrl_ptr, int8_t labels[TWO_PULSE])
{
    // Case 1: label[0] is just after the wrap point (e.g. 6 -> 1 boundary)
    // Shift label[1] up by one full revolution worth of sectors
    if ((labels[0] - labels[1]) > (SIX_PULSE >> 1))
    {
        ctrl_ptr->six_pulse_inj.labels_unwrapped[0] = labels[0] - 1;
        ctrl_ptr->six_pulse_inj.labels_unwrapped[1] = labels[1] + SIX_PULSE - 1;
    }
    // Case 2: label[1] is just after the wrap point
    else if ((labels[1] - labels[0]) > (SIX_PULSE >> 1))
    {
        ctrl_ptr->six_pulse_inj.labels_unwrapped[0] = labels[0] + SIX_PULSE - 1;
        ctrl_ptr->six_pulse_inj.labels_unwrapped[1] = labels[1] - 1;
    }
    // Case 3: no wrap needed — simple 1-to-0 base conversion
    else
    {
        ctrl_ptr->six_pulse_inj.labels_unwrapped[0] = labels[0] - 1;
        ctrl_ptr->six_pulse_inj.labels_unwrapped[1] = labels[1] - 1;
    }
}

// Cross-verifies the two independent estimates (i_peak and k) against each other.
// Both methods should point to the same two sectors if the detection is reliable.
static inline void VerifyLabels(CTRL_t *ctrl_ptr)
{
    // Success only if both metrics agree on the same two candidate sectors
    if ((ctrl_ptr->six_pulse_inj.labels_i_peak[0] == ctrl_ptr->six_pulse_inj.labels_k[0]) && (ctrl_ptr->six_pulse_inj.labels_i_peak[1] == ctrl_ptr->six_pulse_inj.labels_k[1]))
    {
        ctrl_ptr->six_pulse_inj.state = Finished_Success;
    }
    else
    {
        // Disagreement between peak-current and decay-rate methods — result is ambiguous
        ctrl_ptr->six_pulse_inj.state = Finished_Ambiguous;
    }
}

/** @brief Six-pulse injection ISR1: process collected pulse data and estimate rotor angle.
 *         Extracts the two dominant sector labels from i_peak and k, cross-verifies them,
 *         unwraps the label pair across the sector boundary, then computes th_r_est by
 *         weighted interpolation: th_r_est = PI/2 + PI/4*label[0] + PI/12*label[1].
 *  @param motor_ptr  Pointer to the motor instance */
void SIX_PULSE_INJ_RunISR1(MOTOR_t *motor_ptr)
{
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    switch (ctrl_ptr->six_pulse_inj.state)
    {
    case Processing_Results:
        // Step 1: Extract the two dominant sector labels from each independent metric
        ExtractTwoLabels(&ctrl_ptr->six_pulse_inj.i_peak, ctrl_ptr->six_pulse_inj.labels_i_peak);
        ExtractTwoLabels(&ctrl_ptr->six_pulse_inj.k, ctrl_ptr->six_pulse_inj.labels_k);

        // Step 2: Cross-verify both metrics; set state to Success or Ambiguous
        VerifyLabels(ctrl_ptr);

        // Step 3: Unwrap the decay-rate labels across the circular sector boundary
        UnWrapLabels(ctrl_ptr, ctrl_ptr->six_pulse_inj.labels_k);

        // Step 4: Compute the estimated rotor angle from the two unwrapped labels.
        // The formula performs a weighted interpolation within the sector:
        //   th_r_est = PI/2 + PI/3 * (label[0]*3/4 + label[1]*1/4)
        //            = PI/2 + PI/4*label[0] + PI/12*label[1]
        ctrl_ptr->six_pulse_inj.th_r_est.elec = Wrap2Pi(PI_OVER_TWO + PI_OVER_FOUR * ctrl_ptr->six_pulse_inj.labels_unwrapped[0] + PI_OVER_TWELVE * ctrl_ptr->six_pulse_inj.labels_unwrapped[1]);
        break;
    case Not_Started:
    case In_Progress_On_Pulse:
    case In_Progress_Off_Pulse:
    case Finished_Success:
    case Finished_Ambiguous:
    default:
        break;
    }
}

#endif
