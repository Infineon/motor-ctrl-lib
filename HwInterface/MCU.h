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

#include "cybsp.h"
#include "stdio.h"
#if defined(CY_USING_HAL)
#include "cyhal.h"
#endif
#if defined(COMPONENT_CAT1)
#include "cy_em_eeprom.h"
#elif defined(COMPONENT_CAT3)
#include "xmc4_e_eeprom.h"
#endif
#include "Controller.h"

typedef struct
{
    float i_uvw;    // [A/ticks]
    float v_uvw;    // [V/ticks]
    float v_dc;     // [V/ticks]
    float v_pot;    // [%/ticks]
    float temp_ps;  // [Celsius/ticks] for active IC, [1/ticks] for passive NTC
} MCU_ADC_SCALE_t;

typedef struct
{
    uint32_t count;         // [ticks]
    uint32_t period;        // [ticks]
    float duty_cycle_coeff; // [ticks/%]
} MCU_TIMER_t;

typedef struct
{    // For time multiplexing ADC measurements and analog routing
    bool en;        // only enabled in high-z state to save computation time when switching
    uint8_t seq;    // mux sequence
    uint8_t idx_isamp[3];	// indices for sampled currents inside dma-results array
} MCU_ADC_MUX_t;

typedef struct
{
    // Must use int32_t so overflow would be OK (using 32bit TCPWM)
    int32_t start;              // [ticks]
    int32_t stop;               // [ticks]
    int32_t duration_ticks;     // [ticks]

    float duration_sec;         // [sec]
    float util;                 // [%], utilization percentage

    float sec_per_tick;         // [sec/tick], based on timer's frequency
    float inv_max_time;         // [1/sec], inverse of max. time corresponding to 100% utilization
} MCU_TIME_CAP_t;

typedef struct
{
	bool init_done;
#if defined(COMPONENT_CAT1)
	cy_stc_eeprom_config_t config;
    cy_en_em_eeprom_status_t status;
    cy_stc_eeprom_context_t context; 
#elif defined(COMPONENT_CAT3)
    E_EEPROM_XMC4_STATUS_t status;
    E_EEPROM_XMC4_t handle;
#endif
} MCU_EEPROM_t;

typedef struct
{
    IRQn_Type nvic_dma_adc_0;
    IRQn_Type nvic_dma_adc_1;
    IRQn_Type nvic_dma_adc_2;
    IRQn_Type nvic_sync_isr1;
    uint32_t state;    
} MCU_INT_t;  // interrupts

typedef struct
{
  float tcpwm;
  float hall;
  float encoder;
} MCU_CLK_FRQ_t;  // [Hz]

typedef struct
{
    MCU_CLK_FRQ_t clk;
    MCU_ADC_SCALE_t adc_scale;
    MCU_TIMER_t pwm;
    MCU_TIMER_t isr0;
    MCU_TIMER_t isr1;
    MCU_EEPROM_t eeprom;
    MCU_INT_t interrupt;
    int32_t dma_results[ADC_MAX];
#if defined(CY_USING_HAL)
    cyhal_wdt_t wdt_obj;
#endif
    MCU_TIME_CAP_t isr0_exe;
    MCU_TIME_CAP_t isr1_exe;
    MCU_ADC_MUX_t adc_mux;
} MCU_t;

// Initializations
void MCU_Init();
void MCU_InitChipInfo();
void MCU_InitInterrupts();
void MCU_InitADCs();
void MCU_InitAnalogRouting();
void MCU_InitDMAs();
void MCU_InitTimers();
void MCU_InitPosInterface();
void MCU_InitWatchdog();

// FLash read/write
void MCU_FlashInit();
bool MCU_FlashReadParams(PARAMS_ID_t id, PARAMS_t *ram_data);
bool MCU_FlashWriteParams(PARAMS_t *ram_data);

// High-Z enter
void MCU_GateDriverEnterHighZ();
void MCU_PhaseUEnterHighZ();
void MCU_PhaseVEnterHighZ();
void MCU_PhaseWEnterHighZ();

// High-Z exit
void MCU_GateDriverExitHighZ();
void MCU_PhaseUExitHighZ();
void MCU_PhaseVExitHighZ();
void MCU_PhaseWExitHighZ();

// Analog routing MUX configurations
#if defined(ANALOG_ROUTING_MUX_RUNTIME) // Routing MUX'ed at runtime, when simultaneous routing of all analog pins to adc channels is not feasible)
enum { Analog_Routing_MUX_0 = 0, Analog_Routing_MUX_1 = 1 }; // Runtime MUX
extern void MCU_RoutingConfigMUXA0(); // Routing ADCs, ADC0::[ISAMP0,ISAMP2,X] & ADC1::[ISAMP1,X,X] , {ISAMP0,ISAMP1,ISAMP2}={IU,IV,IW}
extern void MCU_RoutingConfigMUXA1(); // Routing ADCs, ADC0::[ISAMP0,ISAMP2,Y] & ADC1::[ISAMP1,Y,Y] , {ISAMP0,ISAMP1,ISAMP2}={IU,IV,IW}
extern void MCU_RoutingConfigMUXB0(); // Routing ADCs, ADC0::[ISAMP0,ISAMP2,X] & ADC1::[ISAMP1,X,X] , {ISAMP0,ISAMP1,ISAMP2}={I_DC_LINK,I_DC_LINK,I_DC_LINK}
extern void MCU_RoutingConfigMUXB1(); // Routing ADCs, ADC0::[ISAMP0,ISAMP2,Y] & ADC1::[ISAMP1,Y,Y] , {ISAMP0,ISAMP1,ISAMP2}={I_DC_LINK,I_DC_LINK,I_DC_LINK}
extern void (*MCU_RoutingConfigMUX0Wrap)();   // Either MUXA0 or MUXB0
extern void (*MCU_RoutingConfigMUX1Wrap)();   // Either MUXA1 or MUXB1

#else
extern void MCU_RoutingConfigMUXA();  // Routing ADCs, ADC0::[ISAMP0,X,X] & ADC1::[ISAMP1,X,X] & ADC2::[ISAMP2,X,X], {ISAMP0,ISAMP1,ISAMP2}={IU,IV,IW}
extern void MCU_RoutingConfigMUXB();  // Routing ADCs, ADC0::[ISAMP0,X,X] & ADC1::[ISAMP1,X,X] & ADC2::[ISAMP2,X,X], {ISAMP0,ISAMP1,ISAMP2}={I_DC_LINK,I_DC_LINK,-}
extern void (*MCU_RoutingConfigMUXWrap)();    // Either MUXA or MUXB

#endif

// Critical section
void MCU_EnterCriticalSection();
void MCU_ExitCriticalSection();

// Start/stop PWMs, ADCs, DMA, ISRs
void MCU_StartPeripherals();
void MCU_StopPeripherals();

// Handling execution time capture measurements
void MCU_StartTimeCap(MCU_TIME_CAP_t *time_cap);
void MCU_StopTimeCap(MCU_TIME_CAP_t *time_cap);
void MCU_ProcessTimeCapISR1(MCU_TIME_CAP_t *time_cap);

// Indicating whether phase voltages are currently being sampled
bool MCU_ArePhaseVoltagesMeasured();

// Enable/disable timer reloads
extern void MCU_EnableTimerReload();
extern void MCU_DisableTimerReload();

// Temperature sensor calculations
float MCU_TempSensorCalc();

// Time-ciritcal control interrupts
void MCU_RunISR0(); // Fast, highest priority
void MCU_RunISR1(); // Slow, second highest priority

