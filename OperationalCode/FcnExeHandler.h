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
#include "General.h"

// NOTE: Function labels determine their execution priority, 0U has the highest priority
typedef enum
{
    Auto_Calc_Params = 0U,	    // auto-calculate parameters
    Reset_Modules,			    // reset all modules after any parameter change
    Flash_Params,			    // store ram parameters to flash
    Set_Tuning_Params_Slow,     // set tuning parameters to preset values for slow control reponse
    Set_Tuning_Params_Moderate, // set tuning parameters to preset values for moderate control reponse
    Set_Tuning_Params_Fast,     // set tuning parameters to preset values for fast control reponse
    Max_Fcn_Count
} FCN_EXE_LABEL_t;

typedef struct
{
  union
  {
    struct
    {
      uint32_t auto_cal_fault     : 1;
      uint32_t write_flash_fault  : 1;
     };
     uint32_t word_access;
  };

}FCN_FAULTFLAG_t;


typedef uint32_t FCN_EXE_REG_t;
STATIC_ASSERT(Max_Fcn_Count <= (sizeof(FCN_EXE_REG_t) * 8U), "Change definition of FCN_EXE_REG_t to uint64_t or larger.");

typedef void(*FCN_EXE_CALLBACK_t)();

typedef struct
{
    FCN_EXE_REG_t req;		// request,			written by GUI,		read by FW
    FCN_EXE_REG_t ack;		// acknowledge,		read by GUI,		written by FW
    FCN_EXE_REG_t done;		// inidicates execution completion,		written/read by FW only
    FCN_EXE_CALLBACK_t callback[Max_Fcn_Count];		// callback functions
    FCN_FAULTFLAG_t fault_flag;
} FCN_EXE_HANDLER_t;

extern FCN_EXE_HANDLER_t fcn_exe_handler;

void FCN_EXE_HANDLER_Init();
void FCN_EXE_HANDLER_Reset();
void FCN_EXE_HANDLER_RunISR1();

FCN_EXE_REG_t FCN_EXE_HANDLER_Mask(FCN_EXE_LABEL_t fcn_label);
