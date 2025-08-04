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

#include "Params.h"

typedef struct
{
    float filt_coeff;
    float raw;
    float calibrated;
    float filt;
} ANALOG_SENSOR_t;

typedef union
{
    struct
    {
        uint32_t dir : 1;		// direction switch
        uint32_t brk : 1;		// brake switch
        uint32_t fault : 1;		// fault pin
        uint32_t padding : 29;	// padding
    };
    uint32_t all;
} DIGITAL_INPUT_t;

typedef struct
{
    ANALOG_SENSOR_t i_samp_0;
    ANALOG_SENSOR_t i_samp_1;
    ANALOG_SENSOR_t i_samp_2;
    float i_xyz[3];
    uint32_t* uvw_idx;
    ANALOG_SENSOR_t i_u;
    ANALOG_SENSOR_t i_v;
    ANALOG_SENSOR_t i_w;
    UVW_t i_uvw_offset_null;		// offset-nulling values
    float offset_null_loop_gain;	// offset-nulling loop gain
    ANALOG_SENSOR_t v_uz;
    ANALOG_SENSOR_t v_vz;
    ANALOG_SENSOR_t v_wz;
    ANALOG_SENSOR_t v_dc;
    ANALOG_SENSOR_t temp_ps;		// power stage temperature
    ANALOG_SENSOR_t pot;			// potentiometer
    DIGITAL_INPUT_t digital;
} SENSOR_IFACE_t;

extern SENSOR_IFACE_t sensor_iface[MOTOR_CTRL_NO_OF_MOTOR];

void SENSOR_IFACE_Init(MOTOR_t *motor_ptr);
void SENSOR_IFACE_Reset(MOTOR_t *motor_ptr);
void SENSOR_IFACE_RunISR0(MOTOR_t *motor_ptr);	// for fast variables
void SENSOR_IFACE_RunISR1(MOTOR_t *motor_ptr);	// for slow variables
void SENSOR_IFACE_OffsetNullISR0(MOTOR_t *motor_ptr);	// Current-sensor offset nulling in init state
