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


#if defined(CTRL_METHOD_SFO)
#include "Controller.h"

void FLUX_CTRL_Init(const float bw_red_ratio)
{
    PI_UpdateParams(&ctrl.flux.pi, params.ctrl.flux.kp * bw_red_ratio, params.ctrl.flux.ki * POW_TWO(bw_red_ratio) * params.sys.samp.ts0, -params.ctrl.flux.vd_max, params.ctrl.flux.vd_max);
}

void FLUX_CTRL_Reset()
{
    PI_Reset(&ctrl.flux.pi);
}

RAMFUNC_BEGIN
void FLUX_CTRL_RunISR0()
{
    PI_Run(&ctrl.flux.pi, vars.la_cmd_final, vars.la_qd_s_est.d, 0.0f);
    vars.v_qd_s_cmd.d = ctrl.flux.pi.output;
}
RAMFUNC_END

#endif