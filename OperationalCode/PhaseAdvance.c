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


#if defined(CTRL_METHOD_RFO)
#include "Controller.h"

void PHASE_ADV_Init()
{
    static const float DL_Min = 1E-9f;	// [H]
    float dl = params.motor.lq - params.motor.ld;
    if (ABS(dl) < DL_Min)
    {
        ctrl.ph_adv.motor_type = SPM;
        ctrl.ph_adv.lam_over_dl = 0.0f;
    }
    else
    {
        ctrl.ph_adv.motor_type = IPM;
        ctrl.ph_adv.lam_over_dl = params.motor.lam / dl;
    }
    ctrl.ph_adv.dl_over_lam = dl / params.motor.lam;
}

RAMFUNC_BEGIN
void PHASE_ADV_RunISR0()
{
    if (ctrl.ph_adv.motor_type == SPM)
    {
        vars.i_qd_r_ref.d = 0.0f;
        vars.i_qd_r_ref.q = vars.i_cmd_int;
    }
    else // IPM
    {
        vars.i_qd_r_ref.d = 0.25f * (ctrl.ph_adv.lam_over_dl - sqrtf(POW_TWO(ctrl.ph_adv.lam_over_dl) + 8.0f * POW_TWO(vars.i_cmd_int)));
        vars.i_qd_r_ref.q = sqrtf(POW_TWO(vars.i_cmd_int) - POW_TWO(vars.i_qd_r_ref.d)) * SIGN(vars.i_cmd_int);
    }
}
RAMFUNC_END

void PHASE_ADV_CalcOptIs(QD_t* i_qd, float* i_s)
{
    // This is an aproximation. Accruate calculation needs solving a 4th order polynomial.
    float c1 = i_qd->q * (1 - ctrl.ph_adv.dl_over_lam * i_qd->d);
    float c2 = -ctrl.ph_adv.dl_over_lam * c1;
    //float id = c1 * c2;
    *i_s = c1 * sqrtf(POW_TWO(c2) + 1.0f / POW_TWO(1.0f + POW_TWO(c2)));
}

#endif
