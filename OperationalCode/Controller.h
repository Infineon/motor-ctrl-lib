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


struct MOTOR_str;
typedef struct MOTOR_str MOTOR_t;

#include "Params.h"
#include "Biquad.h"
#include "CtrlFilts.h"
#include "CtrlVars.h"
#include "FaultProtect.h"
#include "FcnExeHandler.h"
#include "General.h"
#include "HallSensor.h"
#include "Observer.h"
#include "PLL.h"
#include "ResonantFilt.h"
#include "SensorIface.h"
#include "NotchFilt.h"
#include "SpeedCtrl.h"
#include "StateMachine.h"
#include "VoltCtrl.h"
#include "Trq.h"
#include "VoltMod.h"
#include "Profiler.h"
#include "AdapTrackLoop.h"
#include "IncEncoder.h"
#include "PositionCtrl.h"

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
    POSITION_CTRL_t position;
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


extern CTRL_t ctrl[MOTOR_CTRL_NO_OF_MOTOR];

typedef struct
{	// hardware interface function pointers
    void (*HardwareIfaceInit)(uint8_t motor_id);
    void (*EnterCriticalSection)();
    void (*ExitCriticalSection)();
    void (*GateDriverEnterHighZ)(uint8_t motor_id);
    void (*GateDriverExitHighZ)(uint8_t motor_id);
    void (*StartPeripherals)(uint8_t motor_id);			// PWMs, ADCs, DMA, ISRs
    void (*StopPeripherals)(uint8_t motor_id);			// PWMs, ADCs, DMA, ISRs
    bool (*FlashRead)(uint8_t motor_id, PARAMS_ID_t id, PARAMS_t* ram_data);
    bool (*FlashWrite)(uint8_t motor_id, PARAMS_t* ram_data);
    bool (*ArePhaseVoltagesMeasured)(uint8_t motor_id);	// can change based on high-z state
} HW_FCN_t;

struct MOTOR_str
{
    const uint8_t motor_instance;
    STATE_MACHINE_t* const sm_ptr;
    CTRL_VARS_t* const vars_ptr;
    CTRL_t* const ctrl_ptr;
    PARAMS_t* const params_ptr;
    OBS_t* const obs_ptr;
    FAULTS_t* const faults_ptr;
    PROTECT_t* const protect_ptr;
    HALL_SENS_t* const hall_ptr;
    SENSOR_IFACE_t*  const sensor_iface_ptr;
    INC_ENCODER_t* const inc_encoder_ptr;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
	PROFILER_t* const profiler_ptr;
#endif

};

extern HW_FCN_t hw_fcn;

extern MOTOR_t motor[MOTOR_CTRL_NO_OF_MOTOR];

void CTRL_ResetWcmdInt(MOTOR_t *Motor_ptr,const ELEC_t w0);
void CTRL_UpdateWcmdIntISR1(MOTOR_t *Motor_ptr,const ELEC_MECH_t w_target, float rate_lim);

