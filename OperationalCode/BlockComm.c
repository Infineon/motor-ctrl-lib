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

#if defined(CTRL_METHOD_TBC)

#define HIZ             (-1.0f)         // In this context, -1.0f is used to indicate HIZ in LUTs
#define IS_HIGHZ(x)     ((x)==(HIZ))  

#define INV_ANG_CAP_TABLE_WIDTH 12U
uint32_t Inv_Angle_Capture_Table[INV_ANG_CAP_TABLE_WIDTH] = \
// Angle input (deg):       (-180)  (-150)  (-120)  (-90)   (-60)   (-30)   (0)     (+30)   (+60)   (+90)   (+120)  (+150)  (+180)
// Angle input + 180 deg:   (0)     (+30)   (+60)   (+90)   (+120)  (+150)  (+180)  (+210)  (+240)  (+270)  (+300)  (+330)  (+360)
// Table indices:           (0)     (1)     (2)     (3)     (4)     (5)     (6)     (7)     (8)     (9)     (10)    (11)    (12)
                        {   0b110,  0b110,  0b100,  0b100,  0b101,  0b101,  0b001,  0b001,  0b011,  0b011,  0b010,  0b010,  };

UVW_SIGNAL_t BLOCK_COMM_EquivHallSignal(ELEC_t th_r)
{
    UVW_SIGNAL_t equiv_hall;
    uint8_t table_index = (uint8_t)((th_r.elec + PI) * SIX_OVER_PI);
    table_index %= INV_ANG_CAP_TABLE_WIDTH;
    equiv_hall.uvw = Inv_Angle_Capture_Table[table_index];
    return equiv_hall;
}

// Hall Signal (WVU):	    0b101   ->  0b001   ->  0b011   ->  0b010   ->  0b110   ->  0b100
// Indices:                 5U      ->  1U      ->  3U      ->  2U      ->  6U      ->  4U
// Phase U:	                1.0f    ->  1.0f    ->  HIZ     ->  0.0f    ->  0.0f    ->  HIZ
// Phase V:	                0.0f    ->  HIZ     ->  1.0f    ->  1.0f    ->  HIZ     ->  0.0f
// Phase W:	                HIZ     ->  0.0f    ->  0.0f    ->  HIZ     ->  1.0f    ->  1.0f
//                                                  0U      1U      2U      3U      4U      5U      6U      7U
float Phase_U_Table[HALL_SIGNAL_PERMUTATIONS] = {  HIZ,    1.0f,   0.0f,   HIZ,    HIZ,    1.0f,   0.0f,   HIZ };
float Phase_V_Table[HALL_SIGNAL_PERMUTATIONS] = {  HIZ,    HIZ,    1.0f,   1.0f,   0.0f,   0.0f,   HIZ,    HIZ };
float Phase_W_Table[HALL_SIGNAL_PERMUTATIONS] = {  HIZ,    0.0f,   HIZ,    0.0f,   1.0f,   HIZ,    1.0f,   HIZ };

RAMFUNC_BEGIN
void BLOCK_COMM_RunCurrSampISR0(MOTOR_t *motor_ptr)
{
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

    vars_ptr->i_s_fb =
        ((ctrl_ptr->block_comm.high_z_state.u) ? 0.0f : (ctrl_ptr->block_comm.i_uvw_coeff.u - 0.5f) * vars_ptr->i_uvw_fb.u) +
        ((ctrl_ptr->block_comm.high_z_state.v) ? 0.0f : (ctrl_ptr->block_comm.i_uvw_coeff.v - 0.5f) * vars_ptr->i_uvw_fb.v) +
        ((ctrl_ptr->block_comm.high_z_state.w) ? 0.0f : (ctrl_ptr->block_comm.i_uvw_coeff.w - 0.5f) * vars_ptr->i_uvw_fb.w);

    vars_ptr->i_s_fb *= SQRT_TWO;
}
RAMFUNC_END

void BLOCK_COMM_Init(MOTOR_t *motor_ptr)
{
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;

    ctrl_ptr->block_comm.hall_equiv.uvw = 0b100;
    ctrl_ptr->block_comm.high_z_state.uvw = 0b000;
    ctrl_ptr->block_comm.high_z_state_prev.uvw = 0b000;

    if (params_ptr->sys.analog.shunt.type == Single_Shunt)
    {
        vars_ptr->d_samp[0] = 0.0f;
        vars_ptr->d_samp[1] = 0.0f;
    }
}

static inline float CalcDutyCycle(bool high_z, bool dir, float i_coeff, float d_ref)
{
#if defined(REGRESSION_TEST)
    static const float High_Z_Duty_Cycle = NAN;
#else
    static const float High_Z_Duty_Cycle = 0.0f;
#endif
    if (high_z)
    {
        return High_Z_Duty_Cycle;
    }
    else if (dir)
    {   // positive direction
        return (i_coeff * d_ref);
    }
    else
    {   // negative direction
        return ((1.0f - i_coeff) * d_ref);
    }
}

RAMFUNC_BEGIN
void BLOCK_COMM_RunVoltModISR0(MOTOR_t *motor_ptr)
{
	CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
	CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
	PARAMS_t* params_ptr = motor_ptr->params_ptr;
	HALL_SENS_t* hall_ptr = motor_ptr->hall_ptr;

    ctrl_ptr->block_comm.hall_equiv = (params_ptr->sys.fb.hall.block_comm_offset_comp == En) ? BLOCK_COMM_EquivHallSignal(hall_ptr->track_loop.th_r_est) : hall_ptr->signal;

    ctrl_ptr->block_comm.i_uvw_coeff.u = Phase_U_Table[ctrl_ptr->block_comm.hall_equiv.uvw];
    ctrl_ptr->block_comm.i_uvw_coeff.v = Phase_V_Table[ctrl_ptr->block_comm.hall_equiv.uvw];
    ctrl_ptr->block_comm.i_uvw_coeff.w = Phase_W_Table[ctrl_ptr->block_comm.hall_equiv.uvw];

    ctrl_ptr->block_comm.high_z_state_prev = ctrl_ptr->block_comm.high_z_state;
    ctrl_ptr->block_comm.high_z_state.u = IS_HIGHZ(ctrl_ptr->block_comm.i_uvw_coeff.u);
    ctrl_ptr->block_comm.high_z_state.v = IS_HIGHZ(ctrl_ptr->block_comm.i_uvw_coeff.v);
    ctrl_ptr->block_comm.high_z_state.w = IS_HIGHZ(ctrl_ptr->block_comm.i_uvw_coeff.w);

    ctrl_ptr->block_comm.v_s_mag = ABS(vars_ptr->v_s_cmd.rad);
    ctrl_ptr->block_comm.v_s_sign = IS_POS(vars_ptr->v_s_cmd.rad);
    ctrl_ptr->block_comm.d_cmd = SAT(0.0f, 1.0f, 2.0f * ctrl_ptr->block_comm.v_s_mag / vars_ptr->v_dc);
#if defined(PC_TEST)
    vars_ptr->test[43] = (float)(ctrl_ptr->block_comm.hall_equiv.u);
    vars_ptr->test[44] = (float)(ctrl_ptr->block_comm.hall_equiv.v);
    vars_ptr->test[45] = (float)(ctrl_ptr->block_comm.hall_equiv.w);
#endif

    vars_ptr->d_uvw_cmd.u = CalcDutyCycle(ctrl_ptr->block_comm.high_z_state.u, ctrl_ptr->block_comm.v_s_sign, ctrl_ptr->block_comm.i_uvw_coeff.u, ctrl_ptr->block_comm.d_cmd);
    vars_ptr->d_uvw_cmd.v = CalcDutyCycle(ctrl_ptr->block_comm.high_z_state.v, ctrl_ptr->block_comm.v_s_sign, ctrl_ptr->block_comm.i_uvw_coeff.v, ctrl_ptr->block_comm.d_cmd);
    vars_ptr->d_uvw_cmd.w = CalcDutyCycle(ctrl_ptr->block_comm.high_z_state.w, ctrl_ptr->block_comm.v_s_sign, ctrl_ptr->block_comm.i_uvw_coeff.w, ctrl_ptr->block_comm.d_cmd);

    ctrl_ptr->block_comm.enter_high_z_flag.u = RISE_EDGE(ctrl_ptr->block_comm.high_z_state_prev.u, ctrl_ptr->block_comm.high_z_state.u) ? true : false;
    ctrl_ptr->block_comm.exit_high_z_flag.u = FALL_EDGE(ctrl_ptr->block_comm.high_z_state_prev.u, ctrl_ptr->block_comm.high_z_state.u) ? true : false;

    ctrl_ptr->block_comm.enter_high_z_flag.v = RISE_EDGE(ctrl_ptr->block_comm.high_z_state_prev.v, ctrl_ptr->block_comm.high_z_state.v) ? true : false;
    ctrl_ptr->block_comm.exit_high_z_flag.v = FALL_EDGE(ctrl_ptr->block_comm.high_z_state_prev.v, ctrl_ptr->block_comm.high_z_state.v) ? true : false;

    ctrl_ptr->block_comm.enter_high_z_flag.w = RISE_EDGE(ctrl_ptr->block_comm.high_z_state_prev.w, ctrl_ptr->block_comm.high_z_state.w) ? true : false;
    ctrl_ptr->block_comm.exit_high_z_flag.w = FALL_EDGE(ctrl_ptr->block_comm.high_z_state_prev.w, ctrl_ptr->block_comm.high_z_state.w) ? true : false;
}
RAMFUNC_END

#endif
