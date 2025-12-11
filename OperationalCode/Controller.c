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
CTRL_t ctrl[MOTOR_CTRL_NO_OF_MOTOR]= { 0 };
CTRL_VARS_t vars[MOTOR_CTRL_NO_OF_MOTOR] = { 0 };

/*Static library function variables declaration */
INC_ENCODER_t inc_encoder[MOTOR_CTRL_NO_OF_MOTOR] = { 0 };
OBS_t obs[MOTOR_CTRL_NO_OF_MOTOR]= { 0 };
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
PROFILER_t profiler[MOTOR_CTRL_NO_OF_MOTOR] = { 0 };
#endif


void CTRL_ResetWcmdInt(MOTOR_t *motor_ptr, const ELEC_t w0)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

    vars_ptr->w_cmd_int.elec = w0.elec * vars_ptr->dir;
}

RAMFUNC_BEGIN
void CTRL_UpdateWcmdIntISR1(MOTOR_t *motor_ptr,const ELEC_MECH_t w_target, float rate_lim)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;

	vars_ptr->w_cmd_int.elec = RateLimit(rate_lim * params_ptr->sys.samp.ts1, w_target.elec, vars_ptr->w_cmd_int.elec);
}
RAMFUNC_END
