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

typedef struct
{
    // Controlled externally (e.g. GUI):
    bool en;					// Disable
    bool em_stop;				// Emergency stop
    // Controlled internally (e.g. potentiometer) or externally (e.g. GUI):
    bool brk;					// Brake switch
    bool clr_faults;             // Clear faults button
    float dir;					// Direction switch, {-1,+1}

    float cmd_int;				// Internal command value
    float cmd_ext;				// External command value
    float cmd_final;			// Final command value

    ELEC_MECH_t w_cmd_ext;		// External
    ELEC_t w_cmd_int;			// Internal, after applying rate limiter
    ELEC_MECH_t w_fb;			// Directly-sensed feedback value
    ELEC_t w_hall;				// Estimated by hall-sensor's loop
    ELEC_t w_enc;				// Estimated by encoder loop
    ELEC_t w_est;				// Estimated by observer
    ELEC_t w_final;				// Final value, either sensed, estimated, or commanded
    ELEC_t w_final_filt;		// Final value filtered
    ELEC_t w_final_filt_abs;	// Final value filtered, absolute
    ELEC_t acc_cmd_int;			// Final value, estimated from command value

    ELEC_t th_r_cmd;			// For voltage control only
    ELEC_MECH_t th_r_fb;		// Directly-sensed feedback value
    ELEC_t th_r_hall;			// Estimated by hall-sensor's loop
    ELEC_MECH_t th_r_enc;		// Estimated by encoder loop
    ELEC_t th_r_est;			// Estimated by observer
    ELEC_MECH_t th_r_final;			// Final value, either sensed, estimated, or commanded
    
    
#if defined(CTRL_METHOD_SFO)
    ELEC_t th_s_est;			// Estimated by observer, SFO
    ELEC_t delta_cmd;			// Load angle command, SFO
    ELEC_t delta_est;			// Estimated load angle, SFO
#if defined(PC_TEST)
    ELEC_t delta_fb;			// Directly-sensed feedback value
#endif
#endif

    PARK_t park_r;
#if defined(CTRL_METHOD_SFO)
    PARK_t park_s;				// SFO
#endif

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    float i_cmd_ext;			// External, RFO/TBC
    float i_cmd_spd;			// From speed loop, RFO/TBC
    float i_cmd_prot;			// Capped by protection limits, RFO/TBC
    float i_cmd_int;			// Internal, after applying protections and rate limiter, RFO/TBC
#endif
#if defined(CTRL_METHOD_RFO)
    QD_t i_qd_r_ref;			// After applying phase advance, RFO
    QD_t i_qd_r_cmd;			// After applying field weakening, RFO
#endif

#if defined(CTRL_METHOD_SFO)
    float la_cmd_mtpa;	// SFO
    float la_cmd_final;	// SFO
    QD_t la_qd_s_est;	// SFO
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    QD_t la_qd_r_est;	// RFO, SFO
#endif

#if defined(CTRL_METHOD_SFO)
    float T_cmd_ext;	// SFO
    float T_cmd_spd;	// SFO
    float T_cmd_prot;	// SFO
    float T_cmd_int;	// SFO
    float T_cmd_mtpv;	// SFO
    float T_cmd_final;	// SFO
#endif
    float T_est;
    float T_est_filt;

    QD_t i_qd_r_fb;
    QD_t v_qd_r_cmd;
#if defined(CTRL_METHOD_SFO)
    QD_t i_qd_s_fb;		// SFO
    QD_t v_qd_s_cmd;	// SFO
#endif

    UVW_t i_uvw_fb;
    AB_t i_ab_fb_tot;	// Including high frequency components
    AB_t i_ab_fb;		// Excluding high freqeuncy components
    float i_s_fb;
    float i_s_fb_sq;

    AB_t v_ab_cmd;		// Excluding high freqeuncy components
    AB_t v_ab_cmd_tot;	// Including high frequency components
    AB_t v_ab_fb;
    AB_t* v_ab_obs;
    POLAR_t v_s_cmd;
    UVW_t v_uvw_n_cmd;
    UVW_t v_uvw_z_cmd;
    UVW_t v_uvw_z_fb;
    UVW_t v_uvw_n_fb;
    float v_nz_fb;
    UVW_t d_uvw_cmd;
    float d_samp[2];    // For current sampling using a single shunt

    float v_dc;

    float temp_ps;		// Analog input

#if defined(PC_TEST)
    float test[128];
#endif
#if defined(CTRL_METHOD_RFO)
   float p_cmd_ext;
   float p_cmd_int;
#endif
} CTRL_VARS_t;

extern CTRL_VARS_t vars[MOTOR_CTRL_NO_OF_MOTOR];

