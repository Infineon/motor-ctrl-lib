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


#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

#include "Controller.h"

#define LABEL		0
#define WVU			1
#define TABLE_DEPTH	2

int8_t Pulse_Table[TABLE_DEPTH][SIX_PULSE] = \
// This pattern is chosen to minimize rotor movement during detection
// Indices:				Time Sequence:		1    ->2    ->3    ->4    ->5    ->6
// First Row (Labels):	Space Vector:		1    ->4    ->2    ->5    ->3    ->6
// Second Row (Values):	WVU Binary:			1    ->6    ->3    ->4    ->2    ->5
// Resulting Current:						(+U) ->(-U) ->(-W) ->(+W) ->(+V) ->(-V)
{   {   0x1,    0x4,    0x2,    0x5,    0x3,    0x6},
    {   0b001,  0b110,  0b011,  0b100,  0b010,  0b101   }   };

void SIX_PULSE_INJ_Init()
{
    StopWatchInit(&ctrl.six_pulse_inj.timer_on, params.ctrl.six_pulse_inj.t_on, params.sys.samp.ts0);
    StopWatchInit(&ctrl.six_pulse_inj.timer_off, params.ctrl.six_pulse_inj.t_off, params.sys.samp.ts0);

    float d_vdc = SAT(0.0f, 1.0f, params.ctrl.six_pulse_inj.v_pulse / params.sys.vdc_nom);
    float d_0 = 1.0f - d_vdc;
    ctrl.six_pulse_inj.d_pos = d_vdc + 0.5f * d_0;
    ctrl.six_pulse_inj.d_neg = 0.5f * d_0;

    SIX_PULSE_INJ_Reset();
}

void SIX_PULSE_INJ_Reset()
{
    ctrl.six_pulse_inj.state = Not_Started;
}

static inline void ResetLabels()
{
    for (int8_t index = 0; index < SIX_PULSE; ++index)
    {
        int8_t wvu = Pulse_Table[LABEL][index];
        ctrl.six_pulse_inj.i_peak.label[index] = wvu;
        ctrl.six_pulse_inj.k.label[index] = wvu;
    }
}

RAMFUNC_BEGIN
void SIX_PULSE_INJ_RunISR0()
{
    int8_t wvu;
    switch (ctrl.six_pulse_inj.state)
    {
    case Not_Started:
        StopWatchReset(&ctrl.six_pulse_inj.timer_on);
        ResetLabels();
        ctrl.six_pulse_inj.th_r_est.elec = 0.0f;
        ctrl.six_pulse_inj.pulse_num = 0;
        wvu = Pulse_Table[WVU][ctrl.six_pulse_inj.pulse_num];
        vars.d_uvw_cmd.u = BIT_TO_FLOAT(wvu, 0b001, ctrl.six_pulse_inj.d_pos, ctrl.six_pulse_inj.d_neg);
        vars.d_uvw_cmd.v = BIT_TO_FLOAT(wvu, 0b010, ctrl.six_pulse_inj.d_pos, ctrl.six_pulse_inj.d_neg);
        vars.d_uvw_cmd.w = BIT_TO_FLOAT(wvu, 0b100, ctrl.six_pulse_inj.d_pos, ctrl.six_pulse_inj.d_neg);
        ctrl.six_pulse_inj.state = In_Progress_On_Pulse;
        break;
    case In_Progress_On_Pulse:
        StopWatchRun(&ctrl.six_pulse_inj.timer_on);
        if (StopWatchIsDone(&ctrl.six_pulse_inj.timer_on))
        {
            StopWatchReset(&ctrl.six_pulse_inj.timer_off);
            ctrl.six_pulse_inj.k_num = 0.0f;
            ctrl.six_pulse_inj.k_den = 0.0f;
            vars.d_uvw_cmd = UVW_Zero;
            ctrl.six_pulse_inj.state = In_Progress_Off_Pulse;
            wvu = Pulse_Table[WVU][ctrl.six_pulse_inj.pulse_num];
            ctrl.six_pulse_inj.i_uvw_sign.u = BIT_TO_FLOAT(wvu, 0b001, +1.0f, -1.0f);
            ctrl.six_pulse_inj.i_uvw_sign.v = BIT_TO_FLOAT(wvu, 0b010, +1.0f, -1.0f);
            ctrl.six_pulse_inj.i_uvw_sign.w = BIT_TO_FLOAT(wvu, 0b100, +1.0f, -1.0f);
        }
        break;
    case In_Progress_Off_Pulse:
        ctrl.six_pulse_inj.i_fb = 0.5f * (vars.i_uvw_fb.u * ctrl.six_pulse_inj.i_uvw_sign.u + vars.i_uvw_fb.v * ctrl.six_pulse_inj.i_uvw_sign.v + vars.i_uvw_fb.w * ctrl.six_pulse_inj.i_uvw_sign.w);
        if (ctrl.six_pulse_inj.timer_off.time_ticks == 0U) // first cycle, sample peak current
        {
            ctrl.six_pulse_inj.i_peak.value[ctrl.six_pulse_inj.pulse_num] = ctrl.six_pulse_inj.i_fb;
        }
        else // next cycles, calculate exponential decay (least squares method)
        {
            ctrl.six_pulse_inj.k_num += POW_TWO(ctrl.six_pulse_inj.i_fb_prev);
            ctrl.six_pulse_inj.k_den += ctrl.six_pulse_inj.i_fb * ctrl.six_pulse_inj.i_fb_prev;
        }
        ctrl.six_pulse_inj.i_fb_prev = ctrl.six_pulse_inj.i_fb;
        StopWatchRun(&ctrl.six_pulse_inj.timer_off);
        if (StopWatchIsDone(&ctrl.six_pulse_inj.timer_off))
        {
            ctrl.six_pulse_inj.k.value[ctrl.six_pulse_inj.pulse_num] = ctrl.six_pulse_inj.k_num / ctrl.six_pulse_inj.k_den;
            if (ctrl.six_pulse_inj.pulse_num < (SIX_PULSE - 1))
            {
                StopWatchReset(&ctrl.six_pulse_inj.timer_on);
                ++ctrl.six_pulse_inj.pulse_num; // S061987
                wvu = Pulse_Table[WVU][ctrl.six_pulse_inj.pulse_num];
                vars.d_uvw_cmd.u = BIT_TO_FLOAT(wvu, 0b001, ctrl.six_pulse_inj.d_pos, ctrl.six_pulse_inj.d_neg);
                vars.d_uvw_cmd.v = BIT_TO_FLOAT(wvu, 0b010, ctrl.six_pulse_inj.d_pos, ctrl.six_pulse_inj.d_neg);
                vars.d_uvw_cmd.w = BIT_TO_FLOAT(wvu, 0b100, ctrl.six_pulse_inj.d_pos, ctrl.six_pulse_inj.d_neg);
                ctrl.six_pulse_inj.state = In_Progress_On_Pulse;
            }
            else
            {
                ctrl.six_pulse_inj.state = Processing_Results;
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
static void ExtractTwoLabels(SIX_PULSE_INJ_DATA_t* data, int8_t results[TWO_PULSE])
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

    int8_t adj_labels[2]; // Adjacent labels to max. label (results[0])
    int8_t adj_indices[2] = { 0, 0 }; // Adjacent indices corresponding to adjacent labels
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

    if (data->value[adj_indices[0]] >= data->value[adj_indices[1]])
    {
        results[1] = data->label[adj_indices[0]];
    }
    else
    {
        results[1] = data->label[adj_indices[1]];
    }
}

static inline void UnWrapLabels(int8_t labels[TWO_PULSE])
{
    if ((labels[0] - labels[1]) > (SIX_PULSE >> 1))
    {
        ctrl.six_pulse_inj.labels_unwrapped[0] = labels[0] - 1;
        ctrl.six_pulse_inj.labels_unwrapped[1] = labels[1] + SIX_PULSE - 1;
    }
    else if ((labels[1] - labels[0]) > (SIX_PULSE >> 1))
    {
        ctrl.six_pulse_inj.labels_unwrapped[0] = labels[0] + SIX_PULSE - 1;
        ctrl.six_pulse_inj.labels_unwrapped[1] = labels[1] - 1;
    }
    else
    {
        ctrl.six_pulse_inj.labels_unwrapped[0] = labels[0] - 1;
        ctrl.six_pulse_inj.labels_unwrapped[1] = labels[1] - 1;
    }
}

static inline void VerifyLabels()
{
    if ((ctrl.six_pulse_inj.labels_i_peak[0] == ctrl.six_pulse_inj.labels_k[0]) && (ctrl.six_pulse_inj.labels_i_peak[1] == ctrl.six_pulse_inj.labels_k[1]))
    {
        ctrl.six_pulse_inj.state = Finished_Success;
    }
    else
    {
        ctrl.six_pulse_inj.state = Finished_Ambiguous;
    }
}


void SIX_PULSE_INJ_RunISR1()
{
    switch (ctrl.six_pulse_inj.state)
    {
    case Processing_Results:
        ExtractTwoLabels(&ctrl.six_pulse_inj.i_peak, ctrl.six_pulse_inj.labels_i_peak);
        ExtractTwoLabels(&ctrl.six_pulse_inj.k, ctrl.six_pulse_inj.labels_k);
        VerifyLabels();
        UnWrapLabels(ctrl.six_pulse_inj.labels_k);
        // th_r_est = PI/2 + PI/3 * ( label[0]*3/4 + label[1]*1/4 ):
        ctrl.six_pulse_inj.th_r_est.elec = Wrap2Pi(PI_OVER_TWO + PI_OVER_FOUR * ctrl.six_pulse_inj.labels_unwrapped[0] + PI_OVER_TWELVE * ctrl.six_pulse_inj.labels_unwrapped[1]);
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