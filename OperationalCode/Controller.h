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

// Note: PC_TEST = REGRESSION_TEST || SIL_TEST

#include "Biquad.h"
#include "CtrlFilts.h"
#include "CtrlVars.h"
#include "FaultProtect.h"
#include "FcnExeHandler.h"
#include "General.h"
#include "HallSensor.h"
#include "Observer.h"
#include "Params.h"
#include "PLL.h"
#include "ResonantFilt.h"
#include "SensorIface.h"
#include "SpeedCtrl.h"
#include "StateMachine.h"
#include "VoltCtrl.h"
#include "Trq.h"
#include "VoltMod.h"
#include "Profiler.h"
#include "AdapTrackLoop.h"
#include "IncEncoder.h"

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
#include "FluxWeaken.h"
#include "HighFreqInj.h"
#include "SixPulseInj.h"
#endif

#if defined(CTRL_METHOD_TBC)
#include "BlockComm.h"
#include "TrapComm.h"
#include "CurrentCtrl.h"
#elif defined(CTRL_METHOD_RFO)
#include "PhaseAdvance.h"
#include "CurrentCtrl.h"
#elif defined(CTRL_METHOD_SFO)
#include "DeltaCtrl.h"
#include "FluxCtrl.h"
#endif

typedef struct
{
    CTRL_FILTS_t filt;
    SPEED_CTRL_t speed;
#if defined(CTRL_METHOD_RFO)
    PHASE_ADV_t ph_adv;
#elif defined(CTRL_METHOD_SFO)
    TRQ_t trq;
    FLUX_CTRL_t flux;
    DELTA_CTRL_t delta;
#elif defined(CTRL_METHOD_TBC)
	BLOCK_COMM_t block_comm;
	TRAP_COMM_t trap_comm;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    CURRENT_CTRL_t curr;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    FLUX_WEAKEN_t flux_weaken;
    SIX_PULSE_INJ_t six_pulse_inj;
    HIGH_FREQ_INJ_t high_freq_inj;
#endif
    VOLT_MOD_t volt_mod;
} CTRL_t;

extern CTRL_t ctrl;

typedef struct
{	// hardware interface function pointers
    void (*HardwareIfaceInit)();
    void (*EnterCriticalSection)();
    void (*ExitCriticalSection)();
    void (*GateDriverEnterHighZ)();
    void (*GateDriverExitHighZ)();
    void (*StartPeripherals)();			// PWMs, ADCs, DMA, ISRs
    void (*StopPeripherals)();			// PWMs, ADCs, DMA, ISRs
    bool (*FlashRead)(PARAMS_ID_t id, PARAMS_t* ram_data);
    bool (*FlashWrite)(PARAMS_t* ram_data);
    bool (*ArePhaseVoltagesMeasured)();	// can change based on high-z state
} HW_FCN_t;

extern HW_FCN_t hw_fcn;

void CTRL_ResetWcmdInt(const ELEC_t w0);
void CTRL_UpdateWcmdIntISR0(const ELEC_MECH_t w_target);