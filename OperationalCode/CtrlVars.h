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

/**
 * @file CtrlVars.h
 * @brief Control variables structure definition
 *
 * Defines the CTRL_VARS_t structure containing all control variables including
 * commands, feedback signals, estimated values, and internal states. This structure
 * maintains the runtime state for motor control including speed, position, current,
 * voltage, and torque variables in various coordinate frames.
 */

#pragma once

#include "General.h"

/**
 * @brief Control variables structure
 *
 * Contains all runtime control variables including commands, feedback, estimates,
 * and intermediate calculations. Variables are organized by control domain
 * (speed, position, current, voltage, torque) and coordinate frame (UVW, AB, QD).
 */
typedef struct
{
    // Controlled externally (e.g. GUI):
    bool en;      /**< Enable disable flag */
    bool em_stop; /**< Emergency stop flag */
    // Controlled internally (e.g. potentiometer) or externally (e.g. GUI):
    bool brk;        /**< Brake switch */
    bool clr_faults; /**< Clear faults button */
    float dir;       /**< Direction switch, {-1,+1} */

    float cmd_int;   /**< Internal command value */
    float cmd_ext;   /**< External command value */
    float cmd_final; /**< Final command value */

    ELEC_MECH_t w_cmd_ext;   /**< External speed command */
    ELEC_t w_cmd_int;        /**< Internal speed command, after rate limiter */
    ELEC_MECH_t w_fb;        /**< Directly-sensed speed feedback */
    ELEC_t w_hall;           /**< Speed estimated by Hall sensor loop */
    ELEC_t w_enc;            /**< Speed estimated by encoder loop */
    ELEC_t w_est;            /**< Speed estimated by observer */
    ELEC_t w_final;          /**< Final speed (sensed, estimated, or commanded) */
    ELEC_t w_final_filt;     /**< Final speed filtered */
    ELEC_t w_final_filt_abs; /**< Final speed filtered, absolute value */
    ELEC_t acc_cmd_int;      /**< Acceleration estimated from command */

    ELEC_t th_r_cmd;        /**< Rotor position command (voltage control only) */
    ELEC_MECH_t th_r_fb;    /**< Directly-sensed rotor position feedback */
    ELEC_t th_r_hall;       /**< Rotor position estimated by Hall sensor */
    ELEC_MECH_t th_r_enc;   /**< Rotor position estimated by encoder */
    ELEC_t th_r_est;        /**< Rotor position estimated by observer */
    ELEC_MECH_t th_r_final; /**< Final rotor position (sensed, estimated, or commanded) */

#if defined(CTRL_METHOD_SFO)
    ELEC_t th_s_est;  /**< Stator flux position estimated by observer (SFO) */
    ELEC_t delta_cmd; /**< Load angle command (SFO) */
    ELEC_t delta_est; /**< Estimated load angle (SFO) */
#if defined(PC_TEST)
    ELEC_t delta_fb; /**< Directly-sensed load angle feedback (SFO, testing only) */
#endif
#endif

    PARK_t park_r; /**< Park transform in rotor frame */
#if defined(CTRL_METHOD_SFO)
    PARK_t park_s; /**< Park transform in stator flux frame (SFO) */
#endif

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    float i_cmd_ext;  /**< External current command (RFO/TBC) */
    float i_cmd_spd;  /**< Current command from speed loop (RFO/TBC) */
    float i_cmd_prot; /**< Current command capped by protection limits (RFO/TBC) */
    float i_cmd_int;  /**< Internal current command after protections and rate limiter (RFO/TBC) */
#endif
#if defined(CTRL_METHOD_RFO)
    QD_t i_qd_r_ref; /**< Current reference after phase advance (RFO) */
    QD_t i_qd_r_cmd; /**< Current command after field weakening (RFO) */
#endif

#if defined(CTRL_METHOD_SFO)
    float la_cmd_mtpa;  /**< Flux command for MTPA (SFO) */
    float la_cmd_final; /**< Final flux command (SFO) */
    QD_t la_qd_s_est;   /**< Estimated flux in stator flux frame (SFO) */
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    QD_t la_qd_r_est; /**< Estimated flux in rotor frame (RFO, SFO) */
#endif

#if defined(CTRL_METHOD_SFO)
    float T_cmd_ext;   /**< External torque command (SFO) */
    float T_cmd_spd;   /**< Torque command from speed loop (SFO) */
    float T_cmd_prot;  /**< Torque command capped by protection (SFO) */
    float T_cmd_int;   /**< Internal torque command (SFO) */
    float T_cmd_mtpv;  /**< Torque command for MTPV (SFO) */
    float T_cmd_final; /**< Final torque command (SFO) */
#endif
    float T_est;      /**< Estimated torque */
    float T_est_filt; /**< Estimated torque filtered */

    QD_t i_qd_r_fb;  /**< Current feedback in rotor frame */
    QD_t v_qd_r_cmd; /**< Voltage command in rotor frame */
#if defined(CTRL_METHOD_SFO)
    QD_t i_qd_s_fb;  /**< Current feedback in stator flux frame (SFO) */
    QD_t v_qd_s_cmd; /**< Voltage command in stator flux frame (SFO) */
#endif

    UVW_t i_uvw_fb;   /**< Current feedback in UVW frame */
    AB_t i_ab_fb_tot; /**< Current feedback in AB frame (including HF components) */
    AB_t i_ab_fb;     /**< Current feedback in AB frame (excluding HF components) */
    float i_s_fb;     /**< Stator current magnitude feedback */
    float i_s_fb_sq;  /**< Stator current magnitude squared */

    AB_t v_ab_cmd;        /**< Voltage command in AB frame (excluding HF) */
    AB_t v_ab_cmd_tot;    /**< Voltage command in AB frame (including HF) */
    AB_t v_ab_fb;         /**< Voltage feedback in AB frame */
    AB_t *v_ab_obs;       /**< Voltage for observer (pointer) */
    POLAR_t v_s_cmd;      /**< Stator voltage command in polar form */
    UVW_t v_uvw_n_cmd;    /**< Phase voltage command (N-referenced) */
    UVW_t v_uvw_z_cmd;    /**< Phase voltage command (Z-referenced) */
    UVW_t v_uvw_z_fb;     /**< Phase voltage feedback (Z-referenced) */
    UVW_t v_uvw_n_fb;     /**< Phase voltage feedback (N-referenced) */
    float v_nz_fb;        /**< Neutral to zero voltage feedback */
    UVW_t d_uvw_cmd;      /**< Duty cycle command for raise */
    UVW_t d_uvw_cmd_fall; /**< Duty cycle command for fall */
    float d_samp[2];      /**< Duty cycles for single shunt current sampling */

    float v_dc; /**< DC bus voltage */

    float temp_ps; /**< Power stage temperature (analog input) */

#if defined(PC_TEST)
    float test[128]; /**< Test array for debugging (PC testing only) */
#endif
#if defined(CTRL_METHOD_RFO)
    float p_cmd_ext; /**< External position command (RFO) */
    float p_cmd_int; /**< Internal position command (RFO) */
#endif
} CTRL_VARS_t;

extern CTRL_VARS_t vars[MOTOR_CTRL_NO_OF_MOTOR];
