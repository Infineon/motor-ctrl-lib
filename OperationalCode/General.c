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

UVW_t UVW_Zero = { 0.0f,0.0f,0.0f };
UVW_t UVW_One = { 1.0f,1.0f,1.0f };
UVW_t UVW_Half = { 0.5f,0.5f,0.5f };
AB_t AB_Zero = { 0.0f,0.0f };
QD_t QD_Zero = { 0.0f,0.0f };
MINMAX_t MinMax_Zero = { 0.0f,0.0f };
PARK_t Park_Zero = { 0.0f,1.0f };
POLAR_t Polar_Zero = { 0.0f,0.0f };
ELEC_t Elec_Zero = { 0.0f };
ELEC_t Mech_Zero = { 0.0f };
ELEC_MECH_t ElecMech_Zero = { 0.0f,0.0f };

RAMFUNC_BEGIN
void EmptyFcn() {};
RAMFUNC_END

RAMFUNC_BEGIN
bool AlwaysTrue() { return true; };
RAMFUNC_END

void PI_Reset(PI_t* pi)
{
    pi->integ = 0.0f;

    pi->error = 0.0f;
    pi->output = 0.0f;
}

void PI_UpdateParams(PI_t* pi, const float kp, const float ki, const float output_min, const float output_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->output_min = output_min;
    pi->output_max = output_max;
}

RAMFUNC_BEGIN
void PI_Run(PI_t* pi, const float cmd, const float fb, const float ff)
{
    pi->error = cmd - fb;
    pi->ff = ff;
    pi->integ += pi->ki * pi->error;
    float prop_ff = pi->error * pi->kp + pi->ff;
    float output_raw = pi->integ + prop_ff;
    if (output_raw <= pi->output_min)
    {
        pi->output = pi->output_min;
        pi->integ = pi->output_min - prop_ff;
    }
    else if (output_raw >= pi->output_max)
    {
        pi->output = pi->output_max;
        pi->integ = pi->output_max - prop_ff;
    }
    else
    {
        pi->output = output_raw;
    }
}
RAMFUNC_END

void PI_IntegBackCalc(PI_t* pi, const float output, const float error, const float ff)
{
    pi->error = error;
    pi->ff = ff;
    pi->output = output;
    pi->integ = output - (error * pi->kp + ff);
}

void BILINEAR_INTEG_Reset(BILINEAR_INTEG_t* bilinear, const float integ_val)
{
    bilinear->integ = integ_val;
    bilinear->prev_input = 0.0f;
}

RAMFUNC_BEGIN
float BILINEAR_INTEG_Run(BILINEAR_INTEG_t* bilinear, const float input)
{
    bilinear->integ += (bilinear->prev_input + input) * 0.5f;
    bilinear->prev_input = input;
    return bilinear->integ;
}
RAMFUNC_END

RAMFUNC_BEGIN
void ClarkeTransform(const UVW_t* input, AB_t* output)
{
    output->alpha = input->u;
    output->beta = (input->w - input->v) * ONE_OVER_SQRT_THREE;
}
RAMFUNC_END

RAMFUNC_BEGIN
void ClarkeTransformInv(const AB_t* input, UVW_t* output)
{
    output->u = input->alpha;
    output->v = -0.5f * input->alpha - SQRT_THREE_OVER_TWO * input->beta;
    output->w = -0.5f * input->alpha + SQRT_THREE_OVER_TWO * input->beta;
}
RAMFUNC_END

RAMFUNC_BEGIN
void ParkInit(const float angle, PARK_t* park)
{
    int32_t sector = (int32_t)((angle * TWO_OVER_PI) + (angle >= 0.0f ? 0.0f : -1.0f));
    float th = angle - (sector * PI_OVER_TWO);
    uint32_t index_s = (uint32_t)(th * params.sys.lut.sin.th_step_inv);
    uint32_t index_c = (TRIG_LUT_WIDTH - 1U) - index_s;
    float d_th = th - (index_s * params.sys.lut.sin.th_step);

    switch (sector & 0x3)
    {
    default:
    case 0x0:	// sector 0
        park->sine = params.sys.lut.sin.val[index_s] + params.sys.lut.sin.val[index_c] * d_th;
        park->cosine = params.sys.lut.sin.val[index_c] - params.sys.lut.sin.val[index_s] * d_th;
        break;
    case 0x1:	// sector 1
        park->sine = params.sys.lut.sin.val[index_c] - params.sys.lut.sin.val[index_s] * d_th;
        park->cosine = -params.sys.lut.sin.val[index_s] - params.sys.lut.sin.val[index_c] * d_th;
        break;
    case 0x2:	// sector 2
        park->sine = -params.sys.lut.sin.val[index_s] - params.sys.lut.sin.val[index_c] * d_th;
        park->cosine = -params.sys.lut.sin.val[index_c] + params.sys.lut.sin.val[index_s] * d_th;
        break;
    case 0x3:	// sector 3
        park->sine = -params.sys.lut.sin.val[index_c] + params.sys.lut.sin.val[index_s] * d_th;
        park->cosine = params.sys.lut.sin.val[index_s] + params.sys.lut.sin.val[index_c] * d_th;
        break;
    }
}
RAMFUNC_END

RAMFUNC_BEGIN
void ParkTransform(const AB_t* input, const PARK_t* park, QD_t* output)
{
    output->q = input->alpha * park->cosine - input->beta * park->sine;
    output->d = input->alpha * park->sine + input->beta * park->cosine;
}
RAMFUNC_END

RAMFUNC_BEGIN
void ParkTransformInv(const QD_t* input, const PARK_t* park, AB_t* output)
{
    output->alpha = input->q * park->cosine + input->d * park->sine;
    output->beta = -input->q * park->sine + input->d * park->cosine;
}
RAMFUNC_END

RAMFUNC_BEGIN
static inline float InvTrigLUT(const float input, INV_TRIG_LUT_t* lut) // do not use outside of this source file
{
    uint32_t index = SAT(0, INV_TRIG_LUT_WIDTH - 2, (int32_t)(input * lut->step_inv));
    float result = lut->val[index] + (input - index * lut->step) * lut->step_inv * (lut->val[index + 1] - lut->val[index]);
    return result;
}
RAMFUNC_END

RAMFUNC_BEGIN
float ATan2(const float y, const float x)
{
    INV_TRIG_LUT_t* lut = &params.sys.lut.atan;
    uint8_t sector = (IS_POS(y) << 1) | IS_POS(x);
    float theta;

    switch (sector)
    {
    default:
    case 0b11:
        theta = IS_POS(x - y) ? InvTrigLUT(y / x, lut) : PI_OVER_TWO - InvTrigLUT(x / y, lut);
        break;
    case 0b10:
        theta = IS_POS(x + y) ? PI_OVER_TWO + InvTrigLUT((-x) / y, lut) : PI - InvTrigLUT(y / (-x), lut);
        break;
    case 0b00:
        theta = IS_NEG(x - y) ? -PI + InvTrigLUT((-y) / (-x), lut) : -PI_OVER_TWO - InvTrigLUT((-x) / (-y), lut);
        break;
    case 0b01:
        theta = IS_NEG(x + y) ? -PI_OVER_TWO + InvTrigLUT(x / (-y), lut) : -InvTrigLUT((-y) / x, lut);
        break;
    }

    return theta;
}
RAMFUNC_END

RAMFUNC_BEGIN
float ASin(const float y)
{
    INV_TRIG_LUT_t* lut = &params.sys.lut.asin;

    float theta = IS_NEG(y) ? -InvTrigLUT(-y, lut) : +InvTrigLUT(y, lut);

    return theta;
}
RAMFUNC_END

RAMFUNC_BEGIN
float ACos(const float x)
{
    INV_TRIG_LUT_t* lut = &params.sys.lut.asin;

    float theta = IS_NEG(x) ? PI_OVER_TWO + InvTrigLUT(-x, lut) : PI_OVER_TWO - InvTrigLUT(x, lut);

    return theta;
}
RAMFUNC_END


RAMFUNC_BEGIN
void ToPolar(const float x, const float y, POLAR_t* polar)
{
    float rad_squared = x * x + y * y;
    polar->rad = sqrtf(rad_squared);
    polar->theta = ATan2(y, x);
}
RAMFUNC_END

RAMFUNC_BEGIN
float LUT1DInterp(const LUT_1D_t* lut, const float input)
{
    float result;
    if (input < lut->x_min)
    {
        result = lut->y[0];
    }
    else if (input >= lut->x_max)
    {
        result = lut->y[LUT_1D_WIDTH-1U];
    }
    else
    {
        uint32_t index = (uint32_t)((input - lut->x_min) * lut->x_step_inv);
        float x_index = lut->x_min + index * lut->x_step;
        result = lut->y[index] + (input - x_index) * lut->x_step_inv * (lut->y[index + 1] - lut->y[index]);
    }
    return result;
}
RAMFUNC_END

RAMFUNC_BEGIN
float SlopeIntercept(const float slope, const float intercept, const float x)
{
    return (slope * x + intercept);
}
RAMFUNC_END

void ScalarBlend(const float ratio, const float x1, const float x2, float* x)
{
    *x = ratio * x1 + (1.0f - ratio) * x2;
}

void AngleBlend(const float ratio, const float th1, const float th2, float* th)
{
    float th1_uw, th2_uw;	// unwrapped
    if ((th1 >= PI_OVER_TWO) && (th2 < -PI_OVER_TWO))
    {
        th1_uw = th1;
        th2_uw = th2 + TWO_PI;
    }
    else if ((th1 < -PI_OVER_TWO) && (th2 >= PI_OVER_TWO))
    {
        th1_uw = th1 + TWO_PI;
        th2_uw = th2;
    }
    else
    {
        th1_uw = th1;
        th2_uw = th2;
    }

    float th_uw = ratio * th1_uw + (1.0f - ratio) * th2_uw;
    *th = Wrap2Pi(th_uw);
}

void PolarBlend(const float ratio, const POLAR_t* polar1, const POLAR_t* polar2, POLAR_t* result)
{
    ScalarBlend(ratio, polar1->rad, polar2->rad, &result->rad);
    AngleBlend(ratio, polar1->theta, polar2->theta, &result->theta);
}

RAMFUNC_BEGIN
float Wrap2Pi(const float th)
{
    int32_t n = (int32_t)((th * ONE_OVER_TWO_PI) + (th >= 0.0f ? 0.5f : -0.5f));
    return (th - n * TWO_PI);
}
RAMFUNC_END

RAMFUNC_BEGIN
float RateLimit(const float rate, const float target, const float current)
{
    if (current >= target)
    {
        return MAX(current - rate, target);
    }
    else
    {
        return MIN(current + rate, target);
    }
}
RAMFUNC_END

RAMFUNC_BEGIN
void SortUVW(UVW_t* uvw, uint32_t* xyz_idx, uint32_t* uvw_idx)
{
    if (uvw->u >= uvw->v)
    {
        if (uvw->w >= uvw->u)
        {
            *xyz_idx = THREE_BYTES_TO_WORD(2U, 0U, 1U);
            *uvw_idx = THREE_BYTES_TO_WORD(1U, 2U, 0U);
        }
        else if (uvw->w < uvw->v)
        {
            *xyz_idx = THREE_BYTES_TO_WORD(0U, 1U, 2U);
            *uvw_idx = THREE_BYTES_TO_WORD(0U, 1U, 2U);
        }
        else
        {
            *xyz_idx = THREE_BYTES_TO_WORD(0U, 2U, 1U);
            *uvw_idx = THREE_BYTES_TO_WORD(0U, 2U, 1U);
        }
    }
    else // v >= u
    {
        if (uvw->w >= uvw->v)
        {
            *xyz_idx = THREE_BYTES_TO_WORD(2U, 1U, 0U);
            *uvw_idx = THREE_BYTES_TO_WORD(2U, 1U, 0U);
        }
        else if (uvw->w < uvw->u)
        {
            *xyz_idx = THREE_BYTES_TO_WORD(1U, 0U, 2U);
            *uvw_idx = THREE_BYTES_TO_WORD(1U, 0U, 2U);
        }
        else
        {
            *xyz_idx = THREE_BYTES_TO_WORD(1U, 2U, 0U);
            *uvw_idx = THREE_BYTES_TO_WORD(2U, 0U, 1U);
        }
    }
}
RAMFUNC_END

void StopWatchInit(TIMER_t* timer, const float time_thresh, const float run_period)
{
    timer->time_thresh = time_thresh;
    timer->run_period = run_period;
    timer->time_thresh_ticks = (uint32_t)(time_thresh / run_period);
    StopWatchReset(timer);
}

void StopWatchReset(TIMER_t* timer)
{
    timer->time_ticks = 0U;
}

RAMFUNC_BEGIN
void StopWatchRun(TIMER_t* timer)
{
    timer->time_ticks += (timer->time_ticks < timer->time_thresh_ticks);
}
RAMFUNC_END

RAMFUNC_BEGIN
bool StopWatchIsDone(TIMER_t* timer)
{
    return (timer->time_ticks >= timer->time_thresh_ticks);
}
RAMFUNC_END

float StopWatchGetTime(TIMER_t* timer)
{
    return (timer->time_ticks * timer->run_period);
}

void (* const DebounceFiltInit)(TIMER_t* timer, const float time_thresh, const float run_period) = &StopWatchInit;
void (* const DebounceFiltReset)(TIMER_t* timer) = &StopWatchReset;
void (* const DebounceFiltInc)(TIMER_t* timer) = &StopWatchRun;
bool (* const DebounceFiltIsSet)(TIMER_t* timer) = &StopWatchIsDone;
float (* const DebounceFiltGetTime)(TIMER_t* timer) = &StopWatchGetTime;

RAMFUNC_BEGIN
void DebounceFiltDec(TIMER_t* timer)
{
    timer->time_ticks -= (timer->time_ticks > 0U);
}
RAMFUNC_END

RAMFUNC_BEGIN
bool DebounceFiltIsClear(TIMER_t* timer)
{
    return (timer->time_ticks == 0U);
}
RAMFUNC_END

RAMFUNC_BEGIN
void LinearRegressionReset(LIN_REG_t* lin_reg)
{
    lin_reg->cnt = 0.0f;
    lin_reg->sigma_x = 0.0f;
    lin_reg->sigma_xx = 0.0f;
    lin_reg->sigma_xy = 0.0f;
    lin_reg->sigma_y = 0.0f;
    lin_reg->sigma_yy = 0.0f;
}
RAMFUNC_END

RAMFUNC_BEGIN
void LinearRegressionAddDataPoint(LIN_REG_t* lin_reg, const float x, const float y)
{
    lin_reg->cnt++;
    lin_reg->sigma_x += x;
    lin_reg->sigma_xx += POW_TWO(x);
    lin_reg->sigma_xy += x * y;
    lin_reg->sigma_y += y;
    lin_reg->sigma_yy += POW_TWO(y);
}
RAMFUNC_END

RAMFUNC_BEGIN
void LinearRegressionProcessData(LIN_REG_t* lin_reg)
{
    lin_reg->var_xx = lin_reg->cnt * lin_reg->sigma_xx - lin_reg->sigma_x * lin_reg->sigma_x;
    lin_reg->var_yy = lin_reg->cnt * lin_reg->sigma_yy - lin_reg->sigma_y * lin_reg->sigma_y;
    lin_reg->cov_xy = lin_reg->cnt * lin_reg->sigma_xy - lin_reg->sigma_x * lin_reg->sigma_y;
    
    lin_reg->m = lin_reg->cov_xy / lin_reg->var_xx;
    lin_reg->r = lin_reg->cov_xy / sqrtf(lin_reg->var_xx * lin_reg->var_yy);
    lin_reg->c = (lin_reg->sigma_y - lin_reg->m * lin_reg->sigma_x) / lin_reg->cnt;
}
RAMFUNC_END