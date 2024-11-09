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
void BLOCK_COMM_RunCurrSampISR0()
{
    vars.i_s_fb =
        ((ctrl.block_comm.high_z_state.u) ? 0.0f : (ctrl.block_comm.i_uvw_coeff.u - 0.5f) * vars.i_uvw_fb.u) +
        ((ctrl.block_comm.high_z_state.v) ? 0.0f : (ctrl.block_comm.i_uvw_coeff.v - 0.5f) * vars.i_uvw_fb.v) +
        ((ctrl.block_comm.high_z_state.w) ? 0.0f : (ctrl.block_comm.i_uvw_coeff.w - 0.5f) * vars.i_uvw_fb.w);

    vars.i_s_fb *= SQRT_TWO;
}
RAMFUNC_END

void BLOCK_COMM_Init()
{
    ctrl.block_comm.hall_equiv.uvw = 0b100;
    ctrl.block_comm.high_z_state.uvw = 0b000;
    ctrl.block_comm.high_z_state_prev.uvw = 0b000;

    if (params.sys.analog.shunt.type == Single_Shunt)
    {
        vars.d_samp[0] = 0.0f;
        vars.d_samp[1] = 0.0f;
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
void BLOCK_COMM_RunVoltModISR0()
{
    ctrl.block_comm.hall_equiv = (params.sys.fb.hall.block_comm_offset_comp == En) ? BLOCK_COMM_EquivHallSignal(hall.track_loop.th_r_est) : hall.signal;

    ctrl.block_comm.i_uvw_coeff.u = Phase_U_Table[ctrl.block_comm.hall_equiv.uvw];
    ctrl.block_comm.i_uvw_coeff.v = Phase_V_Table[ctrl.block_comm.hall_equiv.uvw];
    ctrl.block_comm.i_uvw_coeff.w = Phase_W_Table[ctrl.block_comm.hall_equiv.uvw];

    ctrl.block_comm.high_z_state_prev = ctrl.block_comm.high_z_state;
    ctrl.block_comm.high_z_state.u = IS_HIGHZ(ctrl.block_comm.i_uvw_coeff.u);
    ctrl.block_comm.high_z_state.v = IS_HIGHZ(ctrl.block_comm.i_uvw_coeff.v);
    ctrl.block_comm.high_z_state.w = IS_HIGHZ(ctrl.block_comm.i_uvw_coeff.w);

    ctrl.block_comm.v_s_mag = ABS(vars.v_s_cmd.rad);
    ctrl.block_comm.v_s_sign = IS_POS(vars.v_s_cmd.rad);
    ctrl.block_comm.d_cmd = SAT(0.0f, 1.0f, 2.0f * ctrl.block_comm.v_s_mag / vars.v_dc);
#if defined(PC_TEST)
    vars.test[43] = (float)(ctrl.block_comm.hall_equiv.u);
    vars.test[44] = (float)(ctrl.block_comm.hall_equiv.v);
    vars.test[45] = (float)(ctrl.block_comm.hall_equiv.w);
#endif

    vars.d_uvw_cmd.u = CalcDutyCycle(ctrl.block_comm.high_z_state.u, ctrl.block_comm.v_s_sign, ctrl.block_comm.i_uvw_coeff.u, ctrl.block_comm.d_cmd);
    vars.d_uvw_cmd.v = CalcDutyCycle(ctrl.block_comm.high_z_state.v, ctrl.block_comm.v_s_sign, ctrl.block_comm.i_uvw_coeff.v, ctrl.block_comm.d_cmd);
    vars.d_uvw_cmd.w = CalcDutyCycle(ctrl.block_comm.high_z_state.w, ctrl.block_comm.v_s_sign, ctrl.block_comm.i_uvw_coeff.w, ctrl.block_comm.d_cmd);

    ctrl.block_comm.enter_high_z_flag.u = RISE_EDGE(ctrl.block_comm.high_z_state_prev.u, ctrl.block_comm.high_z_state.u) ? true : false;
    ctrl.block_comm.exit_high_z_flag.u = FALL_EDGE(ctrl.block_comm.high_z_state_prev.u, ctrl.block_comm.high_z_state.u) ? true : false;

    ctrl.block_comm.enter_high_z_flag.v = RISE_EDGE(ctrl.block_comm.high_z_state_prev.v, ctrl.block_comm.high_z_state.v) ? true : false;
    ctrl.block_comm.exit_high_z_flag.v = FALL_EDGE(ctrl.block_comm.high_z_state_prev.v, ctrl.block_comm.high_z_state.v) ? true : false;

    ctrl.block_comm.enter_high_z_flag.w = RISE_EDGE(ctrl.block_comm.high_z_state_prev.w, ctrl.block_comm.high_z_state.w) ? true : false;
    ctrl.block_comm.exit_high_z_flag.w = FALL_EDGE(ctrl.block_comm.high_z_state_prev.w, ctrl.block_comm.high_z_state.w) ? true : false;
}
RAMFUNC_END

#endif
