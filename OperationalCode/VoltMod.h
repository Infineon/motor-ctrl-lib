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

typedef struct
{	// space vector modulation
    int32_t sector;			// [#]
    float v_first;			// [V]
    float v_second;			// [V]
    float duty_first;		// [%]
    float duty_second;		// [%]
    float duty_zero;		// [%]
    bool five_segment;
} SVM_t;

typedef struct
{	// neutral point modulation
    float v_neutral;	// [V]
} NPM_t;

typedef struct
{   // hybrid modulation, for single shunt configuration
    EN_DIS_t status;        // [#]
    float kv;               // [V/V]
    float th_min;           // [Ra-elec]
    float th_error;         // [Ra-elec]
    float th_comp;          // [Ra-elec]
    float th_tot;           // [Ra-elec]
    float th_shifted;       // [Ra-elec]
    float th_base;          // [Ra-elec]
    float th_sector;        // [Ra-elec]
    float th_mod;           // [Ra-elec]
    PARK_t park;            // [#]
} HM_t;

typedef struct
{
    NPM_t npm;
    SVM_t svm;
    HM_t hm;
    float v_dc_inv;         // [1/V]
    float mi;			    // [%], modulation index, defined as Vpeak/(2Vdc/3)
    float mi_filt;          // [%], Filtered value of modulation index
    uint32_t xyz_idx;       // [%], Indices for converting from UVW to XYZ
    uint32_t uvw_idx;       // [%], Indices for converting from XYZ to UVW
    uint32_t uvw_idx_prev;  // [%], Indices for converting from XYZ to UVW
} VOLT_MOD_t;

void VOLT_MOD_Init();
void VOLT_MOD_RunISR0();
void VOLT_MOD_EnDisHybMod(EN_DIS_t en);
void HybridModRunISR0();
