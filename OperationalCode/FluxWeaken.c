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

#if defined(CTRL_METHOD_RFO)
void FLUX_WEAKEN_Reset()
{
    ctrl.flux_weaken.integ = 0.0f;
}
#endif

RAMFUNC_BEGIN
void FLUX_WEAKEN_RunISR0()
{
    if (params.ctrl.flux_weaken.en == En)
    {
#if defined(PC_TEST)
        vars.test[29] = vars.v_dc;
#endif
#if defined(CTRL_METHOD_RFO)
        // Voltage limits
        ctrl.flux_weaken.v_q_lim_sq = MAX(POW_TWO(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff) - POW_TWO(vars.v_qd_r_cmd.d), 0.0f);
        ctrl.flux_weaken.v_q_lim = sqrtf(ctrl.flux_weaken.v_q_lim_sq);

        // Integrator
        ctrl.flux_weaken.vq_error = ctrl.flux_weaken.v_q_lim - ABS(vars.v_qd_r_cmd.q);
        ctrl.flux_weaken.integ += ctrl.flux_weaken.vq_error * params.ctrl.flux_weaken.ki;

        // Integrator limits
        ctrl.flux_weaken.id_lim = sqrtf(MAX(POW_TWO(protect.motor.i2t.i_limit) - POW_TWO(vars.i_qd_r_fb.q), 0.0f));
        const float Integ_Lim_Pos_Margin = 0.05f;
        ctrl.flux_weaken.integ_lim.max = MIN(ctrl.flux_weaken.id_lim, Integ_Lim_Pos_Margin * params.motor.id_max) - vars.i_qd_r_ref.d;
        ctrl.flux_weaken.integ_lim.min = MAX(-ctrl.flux_weaken.id_lim, -params.motor.id_max) - vars.i_qd_r_ref.d;
        ctrl.flux_weaken.integ = SAT(ctrl.flux_weaken.integ_lim.min, ctrl.flux_weaken.integ_lim.max, ctrl.flux_weaken.integ);

        // Field weakening current and activation indicator
        if (ctrl.flux_weaken.integ < 0.0f)
        {
            ctrl.flux_weaken.id_fw = ctrl.flux_weaken.integ;
            ctrl.flux_weaken.activated = true;
        }
        else
        {
            ctrl.flux_weaken.id_fw = 0.0f;
            ctrl.flux_weaken.activated = false;
        }
        vars.i_qd_r_cmd.q = vars.i_qd_r_ref.q;		// Not changing speed loop's dynamics
        vars.i_qd_r_cmd.d = vars.i_qd_r_ref.d + ctrl.flux_weaken.id_fw;

#elif defined(CTRL_METHOD_SFO)
        // Voltage limits
        ctrl.flux_weaken.v_q_lim_sq = MAX(POW_TWO(vars.v_dc * params.ctrl.flux_weaken.vdc_coeff) - POW_TWO(vars.v_qd_s_cmd.d), 0.0f);
        ctrl.flux_weaken.v_q_lim = sqrtf(ctrl.flux_weaken.v_q_lim_sq);

        if (vars.w_final_filt_abs.elec > params.ctrl.flux_weaken.w_min.elec)
        {
            ctrl.flux_weaken.la_cmd_lim = (ctrl.flux_weaken.v_q_lim * SIGN(vars.v_qd_s_cmd.q) - params.motor.r * vars.i_qd_s_fb.q) / vars.w_final_filt.elec;
        }
        else
        {
            ctrl.flux_weaken.la_cmd_lim = vars.la_cmd_mtpa;
        }

        // Field weakening flux command and activation indicator
        if (ctrl.flux_weaken.la_cmd_lim < vars.la_cmd_mtpa)
        {
            vars.la_cmd_final = ctrl.flux_weaken.la_cmd_lim;
            ctrl.flux_weaken.activated = true;
        }
        else
        {
            vars.la_cmd_final = vars.la_cmd_mtpa;
            ctrl.flux_weaken.activated = false;
        }

#endif
    }
    else // Disabled
    {
#if defined(CTRL_METHOD_RFO)
        vars.i_qd_r_cmd.q = vars.i_qd_r_ref.q;
        vars.i_qd_r_cmd.d = vars.i_qd_r_ref.d;

#elif defined(CTRL_METHOD_SFO)
        vars.la_cmd_final = vars.la_cmd_mtpa;
#endif
    }

}
RAMFUNC_END

#endif