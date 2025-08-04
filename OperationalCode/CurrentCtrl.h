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

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)

#include "General.h"

typedef struct
{
    bool en_ff;
#if defined(CTRL_METHOD_RFO)
    QD_t ff;
    PI_t pi_q;
    PI_t pi_d;
#elif defined(CTRL_METHOD_TBC)
    float ff;
    PI_t pi;
#endif
} CURRENT_CTRL_t;

#if defined(CTRL_METHOD_RFO)
void CURRENT_CTRL_Init(MOTOR_t *motor_ptr, const float bw_red_coeff);
#elif defined(CTRL_METHOD_TBC)
void CURRENT_CTRL_Init(MOTOR_t *motor_ptr);
#endif
void CURRENT_CTRL_Reset(MOTOR_t *motor_ptr);
void CURRENT_CTRL_RunISR0(MOTOR_t *motor_ptr);



#endif



