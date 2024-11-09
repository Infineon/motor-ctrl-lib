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

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>

#define POW_TWO(x)		((x)*(x))
#define POW_THREE(x)	((x)*(x)*(x))
#define MIN(x,y)		(((x)<(y))?(x):(y))
#define MAX(x,y)		(((x)<(y))?(y):(x))
#define SAT(xl,xh,x)	(((x)<(xl))?(xl):(((x)>(xh))?(xh):(x)))
#define ABS(x)			(fabsf(x))	// always compiling to VABS.F32 in Cortex-M4
#define SIGN(x)			(((x)>=0.0f)?(+1.0f):(-1.0f))
#define IS_POS(x)		((x)>0.0f)
#define IS_NEG(x)		((x)<0.0f)

#define RISE_EDGE(previous,current)		((!(previous))&&(current))
#define FALL_EDGE(previous,current)		((previous)&&(!(current)))
#define TRANS_EDGE(previous,current)	(RISE_EDGE(previous,current)||FALL_EDGE(previous,current)) 

#define AVE(x,y)				(((x)+(y))*(0.5f))
#define ABS_BELOW_LIM(x,lim)	(ABS(x)<(lim))	// lim must be positive
#define ABS_ABOVE_LIM(x,lim)	(ABS(x)>(lim))	// lim must be positive
#define ROUND_FLOAT_TO_INT(x)	((int32_t)((x)+(((x)>=0.0f)?(+0.5f):(-0.5f))))
#define QUANTIZE_FLOAT(x,q)		((float)(ROUND_FLOAT_TO_INT((x)/(q)))*(q))

#define SQRT_TWO				(1.414213562373095f)
#define SQRT_THREE				(1.732050807568877f)
#define SQRT_THREE_OVER_TWO		(0.866025403784439f)
#define ONE_OVER_SQRT_TWO		(0.707106781186547f)
#define ONE_OVER_SQRT_THREE		(0.577350269189626f)
#define TWO_OVER_SQRT_THREE		(1.154700538379252f)
#define EXP_ONE					(2.718281828459046f)
#define EXP_MINUS_ONE			(0.367879441171442f)
#define LOG_TWO_E				(1.442695040888963f)
#define LOG_TEN_E				(0.434294481903252f)
#define LN_TWO					(0.693147180559945f)
#define LN_TEN					(2.302585092994046f)
#define TWO_PI					(6.283185307179586f)
#define TWO_PI_OVER_THREE		(2.094395102393195f)
#define FIVE_PI_OVER_SIX		(2.617993877991494f)
#define PI						(3.141592653589793f)
#define PI_OVER_TWO				(1.570796326794897f)
#define PI_OVER_THREE			(1.047197551196598f)
#define PI_OVER_FOUR			(0.785398163397448f)
#define PI_OVER_SIX				(0.523598775598299f)
#define PI_OVER_TWELVE			(0.261799387799149f)
#define ONE_OVER_PI				(0.318309886183791f)
#define ONE_OVER_TWO_PI			(0.159154943091895f)
#define ONE_OVER_THREE_PI		(0.106103295394597f)
#define TWO_OVER_PI				(0.636619772367581f)
#define THREE_OVER_PI			(0.954929658551372f)
#define SIX_OVER_PI				(1.909859317102744f)

#define SCALE_PI_TO_INT32	((float)(INT32_MAX) / PI)
#define SCALE_INT32_TO_DEG	(180.0f / (float)(INT32_MAX))

#define LUT_1D_N			(7)
#define LUT_1D_WIDTH		(1<<LUT_1D_N)

#define TRIG_LUT_N			(6)
#define TRIG_LUT_WIDTH		(1<<TRIG_LUT_N)

#define INV_TRIG_LUT_N		(5)
#define INV_TRIG_LUT_WIDTH  (1<<INV_TRIG_LUT_N)

#define TEMP_SENS_LUT_N     (4)
#define TEMP_SENS_LUT_WIDTH (1<<TEMP_SENS_LUT_N)

#define HALL_SIGNAL_PERMUTATIONS	8U	// only 6 out of 8 possible hall signal permutations are valid

#if defined(PC_TEST)
#define PROF_FREQ_POINTS            11U  // number of profiler's excitation frequency points
#define PROF_SPEED_POINTS           11U  // number of profiler's speed command points, 
#else
#define PROF_FREQ_POINTS            25U  // number of profiler's excitation frequency points
#define PROF_SPEED_POINTS           25U  // number of profiler's speed command points, 
#endif

#define HZ_TO_RADSEC(x)		(TWO_PI*(x))
#define RADSEC_TO_HZ(x)		(ONE_OVER_TWO_PI*(x))
#define TAU_TO_RADSEC(x)	(1.0f/(x))
#define RADSEC_TO_TAU(x)	(1.0f/(x))
#define PERIOD_TO_RADSEC(x)	(TWO_PI/(x))
#define RADSEC_TO_PERIOD(x)	(TWO_PI/(x))
#define MECH_TO_ELEC(x,p)   ((x)*(0.5f)*(p))
#define ELEC_TO_MECH(x,p)   ((x)*(2.0f)/(p))
#define RPM_TO_HZ(x)		((x)*(1.0f/60.0f))
#define HZ_TO_RPM(x)		((x)*(60.0f))
#define DEG_TO_RAD(x)       ((x)*(PI/180.0f))
#define RAD_TO_DEG(x)       ((x)*(180.0f/PI))
#define RMS_TO_PK(x)		((x)*SQRT_TWO)
#define PK_TO_RMS(x)		((x)*ONE_OVER_SQRT_TWO)
#define PHASE_TO_LINE(x)	((x)*SQRT_THREE)
#define LINE_TO_PHASE(x)	((x)*ONE_OVER_SQRT_THREE)
#define BIT_TO_FLOAT(word,bit_mask,one_val,zero_val)	(((word)&(bit_mask))?(one_val):(zero_val))
#define PERC_TO_NORM(x)		((x)*0.01f)
#define NORM_TO_PERC(x)		((x)*100.0f)

#define BYTE_TO_WORD(byte,index)    ((byte)<<((index)*8U))
#define WORD_TO_BYTE(word,index)    (((word)>>((index)*8U))&(0xFF))
#define THREE_BYTES_TO_WORD(byte0,byte1,byte2)  (BYTE_TO_WORD(byte0,0U)|BYTE_TO_WORD(byte1,1U)|BYTE_TO_WORD(byte2,2U))

#define STRUCT_TO_ARRAY(instance)   ((float*)(&instance))   // Use pointer casting carefully

/*Note: RAMFUNC_ENABLE macro definition is done in makefile*/
#if !defined(PC_TEST) && defined(RAMFUNC_ENABLE)
#include "cy_utils.h"
#define RAMFUNC_BEGIN   CY_RAMFUNC_BEGIN
#define RAMFUNC_END     CY_RAMFUNC_END
#else
#define RAMFUNC_BEGIN
#define RAMFUNC_END
#endif

#if defined(SIL_TEST) && !defined(__cplusplus)
#define STATIC_ASSERT(cond,msg) _Static_assert(cond,msg)
#else
#define STATIC_ASSERT(cond,msg) static_assert(cond,msg)
#endif

#pragma pack(push,4)

typedef struct
{
    float u;
    float v;
    float w;
} UVW_t;

typedef struct
{
	float x;
	float y;
	float z;
} XYZ_t;

typedef struct
{
    float alpha;
    float beta;
} AB_t;

typedef struct
{
    float q;
    float d;
} QD_t;

typedef struct
{
    float min;
    float max;
} MINMAX_t;

typedef struct
{
    float sine;
    float cosine;
} PARK_t;

typedef struct
{
    float rad;
    float theta;
} POLAR_t;

#pragma pack(pop)

typedef struct
{
    float x_min;			// min x
    float x_max;			// max x
    float x_step;			// =(x_max-x_min)/(LUT_1D_WIDTH-1)
    float x_step_inv;		// =1/x_step
    float y[LUT_1D_WIDTH];	// y-axis, output
} LUT_1D_t;					// look up table 1 dimension

typedef struct
{
    // PI with back-calculation type anti-windup
    // H(s)=kp+ki/s
    float kp;
    float ki;
    float output_min;
    float output_max;

    float integ;

    float ff;
    float error;
    float output;
} PI_t;

typedef struct
{
    float prev_input;
    float integ;
} BILINEAR_INTEG_t;

typedef enum
{
    SPM = 0,
    IPM = 1,
} MOTOR_TYPE_t;

typedef enum
{
    Dis = 0,
    En = 1,
} EN_DIS_t;

typedef enum
{
    Task_Waiting = 0U,
    Task_Started,
    Task_Finished,
    Task_Error
} TASK_STATUS_t;

typedef enum
{
    Slow = 0U,
    Moderate,
    Fast
} SPEED_ATTRIB_t;

typedef enum
{
    Sensorless = 0,	// either sensorless observer or high-freqeuncy-injection
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    Hall = 1,		// hall sensor
    AqB_Enc = 2,	// A quad B encoder
    //Direct = 3,	// TBD, direct feedback from analog or digital inputs
#endif
} FB_MODE_t;

typedef struct
{
    float elec;
} ELEC_t;

typedef struct
{
    float mech;
} MECH_t;

typedef struct
{
    float elec;
    float mech;
} ELEC_MECH_t;

typedef struct
{
    // theta is in [0,pi/2]
    float th_step;
    float th_step_inv;
    float val[TRIG_LUT_WIDTH];
} TRIG_LUT_t;

typedef struct
{
    // input is in [0,1]
    // output is in [0, pi/4] for atan and [0, pi/2] for asin
    float step;
    float step_inv;
    float val[INV_TRIG_LUT_WIDTH];
} INV_TRIG_LUT_t;

typedef struct
{
    // input is in [step,1-step], excluding {0,1} asymptotic infinites
    float step;
    float step_inv;
    float val[TEMP_SENS_LUT_WIDTH];
} TEMP_SENS_LUT_t;

typedef struct
{
    float time_thresh;			// [sec]
    float run_period;			// [sec]
    uint32_t time_thresh_ticks;	// [#]
    uint32_t time_ticks;		// [#]
} TIMER_t;

typedef struct
{   
    float cnt;
    float sigma_x;
    float sigma_xx;
    float sigma_xy;
    float sigma_y;
    float sigma_yy;
    float var_xx;   // *N^2, variance
    float var_yy;   // *N^2, variance
    float cov_xy;   // *N^2, covariance
    float m;        // slope, y = mx + c
    float c;        // intercept, y = mx + c
    float r;        // correlation
} LIN_REG_t;

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

void EmptyFcn();
bool AlwaysTrue();

void PI_Reset(PI_t* pi);
void PI_UpdateParams(PI_t* pi, const float kp, const float ki, const float output_min, const float output_max);
void PI_Run(PI_t* pi, const float cmd, const float fb, const float ff);	// with feed forward
void PI_IntegBackCalc(PI_t* pi, const float output, const float error, const float ff);

void BILINEAR_INTEG_Reset(BILINEAR_INTEG_t* bilinear, const float integ_val);
float BILINEAR_INTEG_Run(BILINEAR_INTEG_t* bilinear, const float input);

void ClarkeTransform(const UVW_t* input, AB_t* output);
void ClarkeTransformInv(const AB_t* input, UVW_t* output);

void ParkInit(const float angle, PARK_t* park);
void ParkTransform(const AB_t* input, const PARK_t* park, QD_t* output);
void ParkTransformInv(const QD_t* input, const PARK_t* park, AB_t* output);

float ATan2(const float y, const float x);
float ASin(const float y);  // output is in [-pi/2,+pi/2]
float ACos(const float x);  // output is in [0,+pi]

void ToPolar(const float x, const float y, POLAR_t* polar);

float LUT1DInterp(const LUT_1D_t* lut, const float input);

float SlopeIntercept(const float slope, const float intercept, const float x);

void ScalarBlend(const float ratio, const float x1, const float x2, float* x);		// ratio must be within [0-1]
void AngleBlend(const float ratio, const float th1, const float th2, float* th);	// angles must be within [-pi,pi)
void PolarBlend(const float ratio, const POLAR_t* polar1, const POLAR_t* polar2, POLAR_t* result);
float Wrap2Pi(const float th);	// in range [-pi,pi)

float RateLimit(const float rate, const float target, const float current);	// rate must be positive, limits positive and negative slopes

// Assuming X>=Y>=Z, xyz_idx and uvw_idx contain indices for converting to XYZ and UVW, respectively:
void SortUVW(UVW_t* uvw, uint32_t* xyz_idx, uint32_t* uvw_idx);

void StopWatchInit(TIMER_t* timer, const float time_thresh, const float run_period);
void StopWatchReset(TIMER_t* timer);		// resets the timer
void StopWatchRun(TIMER_t* timer);
bool StopWatchIsDone(TIMER_t* timer);		// returns true if it is time
float StopWatchGetTime(TIMER_t* timer);		// get time in [sec]

extern void (* const DebounceFiltInit)(TIMER_t* timer, const float time_thresh, const float run_period);
extern void (* const DebounceFiltReset)(TIMER_t* timer);
extern void (* const DebounceFiltInc)(TIMER_t* timer);
extern bool (* const DebounceFiltIsSet)(TIMER_t* timer);
extern float (* const DebounceFiltGetTime)(TIMER_t* timer);
void DebounceFiltDec(TIMER_t* timer);
bool DebounceFiltIsClear(TIMER_t* timer);

void LinearRegressionReset(LIN_REG_t* lin_reg);
void LinearRegressionAddDataPoint(LIN_REG_t* lin_reg, const float x, const float y);
void LinearRegressionProcessData(LIN_REG_t* lin_reg);