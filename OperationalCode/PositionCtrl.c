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

#include "General.h"
#if defined(CTRL_METHOD_RFO)

#include "Controller.h"

void POSITION_CTRL_Init(MOTOR_t *motor_ptr)
{
    CTRL_t* ctrl_ptr   = motor_ptr->ctrl_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;
    
   PI_UpdateParams(&ctrl_ptr->position.pi, params_ptr->ctrl.position.kp, params_ptr->ctrl.position.ki * params_ptr->sys.samp.ts1, -params_ptr->ctrl.position.pi_output_limit, params_ptr->ctrl.position.pi_output_limit);
   ctrl_ptr->position.ff_conv_coeff =  params_ptr->sys.samp.fs1*(params_ptr->motor.P/2);

}

void POSITION_CTRL_Reset(MOTOR_t *motor_ptr)
{
   CTRL_t* ctrl_ptr   = motor_ptr->ctrl_ptr;
   CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;

   vars_ptr->p_cmd_int =0;
   ctrl_ptr->position.th_r_mechanical_prev =0;
   ctrl_ptr->position.th_r_position= 0;
   ctrl_ptr->position.rotation = 0;
 
   PI_Reset(&ctrl_ptr->position.pi);
}

void POSITION_CTRL_cmdInt(MOTOR_t *motor_ptr,float angle)
{
   CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
   CTRL_t* ctrl_ptr   = motor_ptr->ctrl_ptr;

   vars_ptr->p_cmd_int =angle;
   ctrl_ptr->position.cmd_prev = angle;
   ctrl_ptr->position.th_r_mechanical_prev =angle;
   ctrl_ptr->position.th_r_position= angle;
}

RAMFUNC_BEGIN
void POSITION_CTRL_RunISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t* vars_ptr = motor_ptr->vars_ptr;
    CTRL_t* ctrl_ptr   = motor_ptr->ctrl_ptr;
    PARAMS_t* params_ptr = motor_ptr->params_ptr;

    float cmd_change;
    float error;
    float delta; 

    /*Feed forward calculation*/ 
    cmd_change = vars_ptr->p_cmd_int-ctrl_ptr->position.cmd_prev;
    ctrl_ptr->position.ff = cmd_change*ctrl_ptr->position.ff_conv_coeff*params_ptr->ctrl.position.ff_coef;
    ctrl_ptr->position.cmd_prev =  vars_ptr->p_cmd_int;

    /*Execution of position PI*/ 
    error = Wrap2Pi(vars_ptr->p_cmd_int- vars_ptr->th_r_final.mech);
    PI_Run(&ctrl_ptr->position.pi, error,0, ctrl_ptr->position.ff);

    /*Rotor position and number of rotation calcuation */ 
    delta = vars_ptr->th_r_final.mech- ctrl_ptr->position.th_r_mechanical_prev;
     /*Note : Limitation ,  change of angle should be less than 180 degree with in one ISR1 Cycle*/ 
    if(delta< -PI) { ctrl_ptr->position.rotation++;} /*Angle change from PI to -PI, in CW direction*/
    else if(delta> PI){ctrl_ptr->position.rotation--;} /*Angle change from -PI to PI, in CCW direction*/
 
    ctrl_ptr->position.th_r_position = vars_ptr->th_r_final.mech+ (ctrl_ptr->position.rotation*TWO_PI);
    ctrl_ptr->position.th_r_mechanical_prev = vars_ptr->th_r_final.mech;
   
}
RAMFUNC_END
#endif