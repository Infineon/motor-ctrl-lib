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

static void (*NeutPointOrSpaceVectModISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = {EmptyFcn };   // Neutral point or space vector modulation (both single and three shunt)
static void (*HybridModRunISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = { EmptyFcn };   // Hybrid modulation for single shunt configuration
static void (*CurrReconstRunISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = {EmptyFcn };   // Calculating current-reconstruction sample times for single shunt configuration

#else
static void (*NeutPointOrSpaceVectModISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = { [0 ... (MOTOR_CTRL_NO_OF_MOTOR - 1)] = EmptyFcn };   // Neutral point or space vector modulation (both single and three shunt)
static void (*HybridModRunISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = { [0 ... (MOTOR_CTRL_NO_OF_MOTOR - 1)] = EmptyFcn };   // Hybrid modulation for single shunt configuration
static void (*CurrReconstRunISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = { [0 ... (MOTOR_CTRL_NO_OF_MOTOR - 1)] = EmptyFcn };   // Calculating current-reconstruction sample times for single shunt configuration
#endif

RAMFUNC_BEGIN
static void SVMRunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    CTRL_t* ctrl_ptr   = motor_ptr->ctrl_ptr;

    float sector_variable_x = -vars_ptr->v_ab_cmd_tot.beta * TWO_OVER_SQRT_THREE;
    float sector_variable_y = vars_ptr->v_ab_cmd_tot.alpha + vars_ptr->v_ab_cmd_tot.beta * ONE_OVER_SQRT_THREE;
    float sector_variable_z = vars_ptr->v_ab_cmd_tot.alpha - vars_ptr->v_ab_cmd_tot.beta * ONE_OVER_SQRT_THREE;

    if (sector_variable_z >= 0.0f)					// Sectors 1, 2 and 6
    {
        if (sector_variable_y >= 0.0f)				// Sectors 1 and 6
        {
            if (sector_variable_x >= 0.0f)			// Sector 1
            {
                ctrl_ptr->volt_mod.svm.v_first = sector_variable_y;
                ctrl_ptr->volt_mod.svm.v_second = sector_variable_x;
                ctrl_ptr->volt_mod.svm.sector = 1;
            }
            else									//Sector 6
            {
                ctrl_ptr->volt_mod.svm.v_first = sector_variable_z;
                ctrl_ptr->volt_mod.svm.v_second = -sector_variable_x;
                ctrl_ptr->volt_mod.svm.sector = 6;
            }
        }
        else										//Sector 2
        {
            ctrl_ptr->volt_mod.svm.v_first = -sector_variable_y;
            ctrl_ptr->volt_mod.svm.v_second = sector_variable_z;
            ctrl_ptr->volt_mod.svm.sector = 2;
        }
    }
    else   											// Sectors 3, 4 and 5
    {
        if (sector_variable_y <= 0.0f)   			// Sectors 3 and 4
        {
            if (sector_variable_x >= 0.0f)			// Sector 3
            {
                ctrl_ptr->volt_mod.svm.v_first = sector_variable_x;
                ctrl_ptr->volt_mod.svm.v_second = -sector_variable_z;
                ctrl_ptr->volt_mod.svm.sector = 3;
            }
            else									// Sector 4
            {
                ctrl_ptr->volt_mod.svm.v_first = -sector_variable_x;
                ctrl_ptr->volt_mod.svm.v_second = -sector_variable_y;
                ctrl_ptr->volt_mod.svm.sector = 4;
            }
        }
        else										// Sector 5
        {
            ctrl_ptr->volt_mod.svm.v_first = -sector_variable_z;
            ctrl_ptr->volt_mod.svm.v_second = sector_variable_y;
            ctrl_ptr->volt_mod.svm.sector = 5;
        }
    }

    float max_voltage_inv = 1.5f * ctrl_ptr->volt_mod.v_dc_inv;
    ctrl_ptr->volt_mod.svm.duty_first = ctrl_ptr->volt_mod.svm.v_first * max_voltage_inv;
    ctrl_ptr->volt_mod.svm.duty_second = ctrl_ptr->volt_mod.svm.v_second * max_voltage_inv;
    float total_duty = ctrl_ptr->volt_mod.svm.duty_first + ctrl_ptr->volt_mod.svm.duty_second;
    if (total_duty <= 1.0f)
    {
        ctrl_ptr->volt_mod.svm.duty_zero = 1.0f - total_duty;
    }
    else  // overmodulation mode I 
    {
        ctrl_ptr->volt_mod.svm.duty_first = ctrl_ptr->volt_mod.svm.duty_first / total_duty;
        ctrl_ptr->volt_mod.svm.duty_second = 1.0f - ctrl_ptr->volt_mod.svm.duty_first;
        ctrl_ptr->volt_mod.svm.duty_zero = 0.0f;
        // TBD: Figuring out a way to avoid duty_zero = 0 for current sensing.
        // One way to deal with it could be defining the MI and going to overmodulation with minimum a defined on-time for LS FET
    }

    ctrl_ptr->volt_mod.mi = vars_ptr->v_s_cmd.rad * max_voltage_inv;
    ctrl_ptr->volt_mod.mi_filt += (ctrl_ptr->volt_mod.mi - ctrl_ptr->volt_mod.mi_filt) * params_ptr->ctrl.volt.five_seg.w0_filt * params_ptr->sys.samp.ts0;
    if (params_ptr->ctrl.volt.five_seg.en == En)
    {
        // 5-segment vs 7-segment determination
        if (ctrl_ptr->volt_mod.mi_filt > params_ptr->ctrl.volt.five_seg.active_mi)
        {
            ctrl_ptr->volt_mod.svm.five_segment = true;
        }
        else if (ctrl_ptr->volt_mod.mi_filt < params_ptr->ctrl.volt.five_seg.inactive_mi)
        {
            ctrl_ptr->volt_mod.svm.five_segment = false;
        }

        // 5-segment vs 7-segment voltage application
        if (ctrl_ptr->volt_mod.svm.five_segment)
        {
            ctrl_ptr->volt_mod.svm.duty_zero *= 2.0f;
        }
    }

    /***************************************
    *    sector 1: 100 & 110        sector 2: 010 & 110         sector 3: 010 & 011         sector 4: 001 & 011         sector 5: 001 & 101         sector 6: 100 & 101
    * __|--------------------|__  ______|------------|______  __________|----|__________  __________|----|__________  ______|------------|______  __|--------------------|__
    * ______|------------|______  __|--------------------|__  __|--------------------|__  ______|------------|______  __________|----|__________  __________|----|______
    * __________|----|__________  __________|----|__________  ______|------------|______  __|--------------------|__  __|--------------------|__  ______|------------|__________
    *     v1  v2  v0  v2  v1          v1  v2  v0  v2  v1          v1  v2  v0  v2  v1          v1  v2  v0  v2  v1          v1  v2  v0  v2  v1          v1  v2  v0  v2  v1
    * ************************************/

    ctrl_ptr->volt_mod.uvw_idx_prev = ctrl_ptr->volt_mod.uvw_idx;
    switch (ctrl_ptr->volt_mod.svm.sector)
    {
    default:
    case 0x1:	// sector 1
        vars_ptr->d_uvw_cmd.u = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.v = vars_ptr->d_uvw_cmd.u - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.w = vars_ptr->d_uvw_cmd.v - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(0U, 1U, 2U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(0U, 1U, 2U);

        break;
    case 0x2:	// sector 2
        vars_ptr->d_uvw_cmd.v = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.u = vars_ptr->d_uvw_cmd.v - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.w = vars_ptr->d_uvw_cmd.u - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(1U, 0U, 2U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(1U, 0U, 2U);

        break;
    case 0x3:	// sector 3
        vars_ptr->d_uvw_cmd.v = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.w = vars_ptr->d_uvw_cmd.v - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.u = vars_ptr->d_uvw_cmd.w - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(1U, 2U, 0U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(2U, 0U, 1U);

        break;
    case 0x4:	// sector 4
        vars_ptr->d_uvw_cmd.w = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.v = vars_ptr->d_uvw_cmd.w - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.u = vars_ptr->d_uvw_cmd.v - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(2U, 1U, 0U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(2U, 1U, 0U);

        break;
    case 0x5:	// sector 5
        vars_ptr->d_uvw_cmd.w = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.u = vars_ptr->d_uvw_cmd.w - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.v = vars_ptr->d_uvw_cmd.u - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(2U, 0U, 1U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(1U, 2U, 0U);

        break;
    case 0x6:	// sector 6
        vars_ptr->d_uvw_cmd.u = 1.0f - ctrl_ptr->volt_mod.svm.duty_zero * 0.5f;
        vars_ptr->d_uvw_cmd.w = vars_ptr->d_uvw_cmd.u - ctrl_ptr->volt_mod.svm.duty_first;
        vars_ptr->d_uvw_cmd.v = vars_ptr->d_uvw_cmd.w - ctrl_ptr->volt_mod.svm.duty_second;
        ctrl_ptr->volt_mod.xyz_idx = THREE_BYTES_TO_WORD(0U, 2U, 1U);
        ctrl_ptr->volt_mod.uvw_idx = THREE_BYTES_TO_WORD(0U, 2U, 1U);

        break;
    }
}
RAMFUNC_END

RAMFUNC_BEGIN
static void NPMRunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    CTRL_t* ctrl_ptr   = motor_ptr->ctrl_ptr;

    ClarkeTransformInv(&vars_ptr->v_ab_cmd_tot, &vars_ptr->v_uvw_n_cmd);

    ctrl_ptr->volt_mod.uvw_idx_prev = ctrl_ptr->volt_mod.uvw_idx;
    SortUVW(&vars_ptr->v_uvw_n_cmd, &ctrl_ptr->volt_mod.xyz_idx, &ctrl_ptr->volt_mod.uvw_idx);

    float* v_uvw_cmd = STRUCT_TO_ARRAY(vars_ptr->v_uvw_n_cmd);

    ctrl_ptr->volt_mod.npm.v_neutral = -0.5f * (v_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 0U)] + v_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 2U)]);

    vars_ptr->v_uvw_z_cmd.u = vars_ptr->v_uvw_n_cmd.u + ctrl_ptr->volt_mod.npm.v_neutral;
    vars_ptr->v_uvw_z_cmd.v = vars_ptr->v_uvw_n_cmd.v + ctrl_ptr->volt_mod.npm.v_neutral;
    vars_ptr->v_uvw_z_cmd.w = vars_ptr->v_uvw_n_cmd.w + ctrl_ptr->volt_mod.npm.v_neutral;

    vars_ptr->d_uvw_cmd.u = SAT(0.0f, 1.0f, vars_ptr->v_uvw_z_cmd.u * ctrl_ptr->volt_mod.v_dc_inv + 0.5f);
    vars_ptr->d_uvw_cmd.v = SAT(0.0f, 1.0f, vars_ptr->v_uvw_z_cmd.v * ctrl_ptr->volt_mod.v_dc_inv + 0.5f);
    vars_ptr->d_uvw_cmd.w = SAT(0.0f, 1.0f, vars_ptr->v_uvw_z_cmd.w * ctrl_ptr->volt_mod.v_dc_inv + 0.5f);

    ctrl_ptr->volt_mod.mi = vars_ptr->v_s_cmd.rad * ctrl_ptr->volt_mod.v_dc_inv * 1.5f; // 2/3Vdc = 100% modulation
}
RAMFUNC_END


RAMFUNC_BEGIN
void CurrReconstRunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    CTRL_t* ctrl_ptr   = motor_ptr->ctrl_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;

    float* d_uvw_cmd = STRUCT_TO_ARRAY(vars_ptr->d_uvw_cmd);
    vars_ptr->d_samp[1] = params_ptr->sys.analog.shunt.cs_settle_raio * d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 0U)] + (1.0f - params_ptr->sys.analog.shunt.cs_settle_raio) * d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 1U)];
    vars_ptr->d_samp[0] = params_ptr->sys.analog.shunt.cs_settle_raio * d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 1U)] + (1.0f - params_ptr->sys.analog.shunt.cs_settle_raio) * d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 2U)];

#if defined(PC_TEST)
    float d_xyz_cmd[3] = { d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 0U)], d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 1U)], d_uvw_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.xyz_idx, 2U)] };
    vars_ptr->test[77] = d_xyz_cmd[0];
    vars_ptr->test[78] = d_xyz_cmd[1];
    vars_ptr->test[79] = d_xyz_cmd[2];
    vars_ptr->test[80] = d_xyz_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.uvw_idx, 0U)];
    vars_ptr->test[81] = d_xyz_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.uvw_idx, 1U)];
    vars_ptr->test[82] = d_xyz_cmd[WORD_TO_BYTE(ctrl_ptr->volt_mod.uvw_idx, 2U)];
    vars_ptr->test[83] = vars_ptr->d_samp[0];
    vars_ptr->test[84] = vars_ptr->d_samp[1];
    vars_ptr->test[85] = (vars_ptr->d_samp[0] - 0.5f) * vars_ptr->v_dc;
    vars_ptr->test[86] = (vars_ptr->d_samp[1] - 0.5f) * vars_ptr->v_dc;
#endif
}
RAMFUNC_END

RAMFUNC_BEGIN
void VOLT_MOD_RunISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    CTRL_t* ctrl_ptr   = motor_ptr->ctrl_ptr;

	ctrl_ptr->volt_mod.v_dc_inv = 1.0f / vars_ptr->v_dc;
	vars_ptr->v_s_cmd.rad = sqrtf(POW_TWO(vars_ptr->v_ab_cmd_tot.alpha) + POW_TWO(vars_ptr->v_ab_cmd_tot.beta));

    HybridModRunISR0Wrap[motor_ptr->motor_instance](motor_ptr);

    NeutPointOrSpaceVectModISR0Wrap[motor_ptr->motor_instance](motor_ptr);

    CurrReconstRunISR0Wrap[motor_ptr->motor_instance](motor_ptr);
}
RAMFUNC_END

void VOLT_MOD_Init(MOTOR_t *motor_ptr)
{
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;

    // Modulation method:
    switch (params_ptr->ctrl.volt.mod_method)
    {
    default:
    case Neutral_Point_Modulation:
        NeutPointOrSpaceVectModISR0Wrap[motor_ptr->motor_instance] = NPMRunISR0;
        break;
    case Space_Vector_Modulation:
        NeutPointOrSpaceVectModISR0Wrap[motor_ptr->motor_instance] = SVMRunISR0;
        break;
    }

    // Three-shunt or single-shunt:
    if (params_ptr->sys.analog.shunt.type == Single_Shunt)
    {
        VOLT_MOD_EnDisHybMod(motor_ptr,En);
    }
    else
    {
        VOLT_MOD_EnDisHybMod(motor_ptr,Dis);
    }

    ctrl_ptr->volt_mod.mi_filt = 0.0f;
}

void VOLT_MOD_EnDisHybMod(MOTOR_t *motor_ptr,EN_DIS_t en)
{
    CTRL_t* ctrl_ptr = motor_ptr->ctrl_ptr;
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

    if ((ctrl_ptr->volt_mod.hm.status == En) && (en == Dis))
    {
    	ctrl_ptr->volt_mod.hm.status = Dis;
        CurrReconstRunISR0Wrap[motor_ptr->motor_instance] = EmptyFcn;
        HybridModRunISR0Wrap[motor_ptr->motor_instance] = EmptyFcn;
        vars_ptr->d_samp[0] = 1.0f - FLT_EPSILON;
        vars_ptr->d_samp[1] = 1.0f - FLT_EPSILON;
    }
    else if ((ctrl_ptr->volt_mod.hm.status == Dis) && (en == En))
    {
    	ctrl_ptr->volt_mod.hm.status = En;
        CurrReconstRunISR0Wrap[motor_ptr->motor_instance] = CurrReconstRunISR0;
        HybridModRunISR0Wrap[motor_ptr->motor_instance] = HybridModRunISR0;
        ctrl_ptr->volt_mod.hm.th_error = 0.0f;
        ctrl_ptr->volt_mod.hm.th_mod = 0.0f;
    }
}
