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
 * @file StateMachine.c
 * @brief Motor control state machine implementation
 *
 * Implements the complete motor control state machine with entry/exit functions
 * for each state and state transition logic.
 */

#include "Controller.h"
#include "General.h"
#include "Params.h"

STATE_MACHINE_t sm[MOTOR_CTRL_NO_OF_MOTOR] = {0};

static void (*CommonISR0Wrap)() = EmptyFcn;
static void (*CommonISR1Wrap)() = EmptyFcn;
#if defined(PC_TEST)
static void (*FeedbackISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = {OBS_RunISR0};
#else
static void (*FeedbackISR0Wrap[MOTOR_CTRL_NO_OF_MOTOR])() = {[0 ...(MOTOR_CTRL_NO_OF_MOTOR - 1)] = OBS_RunISR0};
#endif

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_TBC)
static inline bool Mode(PARAMS_t *params_ptr, CTRL_MODE_t ctrl_mode)
{
    return (params_ptr->ctrl.mode == ctrl_mode);
}
#endif
/**
 * @brief Fast common ISR executed every ISR0 cycle regardless of state
 *
 * Runs sensor acquisition and, once current-sensor offset nulling is complete,
 * fault protection. Gated on offset_null_done to avoid false fault trips
 * during the ADC calibration phase.
 */
RAMFUNC_BEGIN
static void CommonISR0(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    // Always acquire sensor readings (ADC results, filtering)
    SENSOR_IFACE_RunISR0(motor_ptr);
    // Only evaluate fault conditions after offset-nulling is complete;
    // premature evaluation would misread current offsets as overcurrents
    if (sm_ptr->vars.init.offset_null_done) /*Current calculation  is not executed, if current offset is not done*/
    {
        FAULT_PROTECT_RunISR0(motor_ptr);
    }
}
RAMFUNC_END

/**
 * @brief Slow common ISR executed every ISR1 cycle regardless of state
 *
 * Runs slow-rate sensor processing, slow-rate fault protection checks,
 * and the function-execution handler for the first motor instance.
 */
static void CommonISR1(MOTOR_t *motor_ptr)
{
    SENSOR_IFACE_RunISR1(motor_ptr);
    FAULT_PROTECT_RunISR1(motor_ptr);

    if (motor_ptr->motor_instance == 0)
    {
        FCN_EXE_HANDLER_RunISR1();
    }
}

/**
 * @brief Enable or disable runtime fault protection response
 *
 * Globally enable or disable the runtime fault-reaction path. In Profiler_Mode
 * (RFO/SFO) phase-loss detection is always disabled regardless of the 'en'
 * argument. For all other modes, enables or disables phase-loss detection
 * and resets its debounce timer on enable.
 *
 * @param motor_ptr  Pointer to motor structure
 * @param en         En to arm fault detection, Dis to disarm
 */
void SM_RuntimeFaultEnDis(MOTOR_t *motor_ptr, EN_DIS_t en)
{
    FAULTS_t *faults_ptr = motor_ptr->faults_ptr;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    if (Mode(params_ptr, Profiler_Mode))
    {
        faults_ptr->phase_loss.enable = Dis;
    }
    else
#endif
    {
        faults_ptr->phase_loss.enable = (en == En);

        if (faults_ptr->phase_loss.enable == true)
        {
            StopWatchReset(&faults_ptr->phase_loss.timer);
        }
    }
}
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
/**
 * @brief Sensorless feedback ISR0 with high-frequency injection blending
 *
 * Manages the transition from HFI startup to pure observer-based control:
 * - While speed is below the lock-in threshold: runs the HFI demodulator,
 *   the observer, and the HFI PLL correction in sequence.
 * - Once speed crosses the threshold (hysteresis): disables HFI, re-initialises
 *   the current/flux controller at full bandwidth, and runs the observer alone.
 */
RAMFUNC_BEGIN
static void SensorlessFeedbackISR0(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

#if defined(CTRL_METHOD_SFO) || defined(PC_TEST)
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
#if defined(PC_TEST)
    OBS_t *obs_ptr = motor_ptr->obs_ptr;
#endif
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    // Hysteresis check: disengage HFI once filtered speed exceeds the lock-in
    // threshold so pure sensorless operation takes over at higher speeds
    if ((sm_ptr->vars.high_freq.used) && ABS_ABOVE_LIM(vars_ptr->w_final_filt.elec, params_ptr->obs.w_thresh.elec))
    {
        sm_ptr->vars.high_freq.used = false;
        // Re-initialise at full bandwidth now that HFI disturbances are no longer injected
#if defined(CTRL_METHOD_RFO)
        CURRENT_CTRL_Init(motor_ptr, 1.0f);
#elif defined(CTRL_METHOD_SFO)
        FLUX_CTRL_Init(motor_ptr, 1.0f);
        ctrl_ptr->delta.bw_red_coeff = 1.0f;
#endif
    }

    if (!sm_ptr->vars.high_freq.used)
    {
        // Pure observer: no injection needed at this speed
        OBS_RunISR0(motor_ptr);
    }
    else // running both observer and high frequency injection
    {
        // HFI + observer blended sequence:
        // 1. Demodulate HFI response to extract rotor position error signal
        HIGH_FREQ_INJ_RunFiltISR0(motor_ptr);
        // 2. Run observer (uses HFI-corrected angle as additional input)
        OBS_RunISR0(motor_ptr);
        // 3. Update HFI PLL to track the error; generates next injection voltage
        HIGH_FREQ_INJ_RunCtrlISR0(motor_ptr);
    }

#if defined(PC_TEST)
    vars_ptr->test[34] = sm_ptr->vars.high_freq.used;
    vars_ptr->test[35] = obs_ptr->pll_r.th.elec;
    vars_ptr->test[36] = ctrl_ptr->high_freq_inj.integ_pll_r.integ;
    vars_ptr->test[37] = obs_ptr->pll_r.w.elec;
    vars_ptr->test[38] = ctrl_ptr->high_freq_inj.pi_pll_r.output;
#endif
}
RAMFUNC_END
#endif

/**
 * @brief Re-initialise and reset every control module
 *
 * Called on first power-up and whenever a full module reload is requested
 * (e.g. after a GUI parameter update). Sequence:
 * 1. Re-initialise all modules with current parameters (gains, filters, tables)
 * 2. Reset CommonISR wrappers and clear sensor/fault states
 * 3. Select the correct feedback ISR based on control mode and sensor type
 */
void STATE_MACHINE_ResetAllModules(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_TBC)
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
#endif
    // --- Step 1: Re-initialise all modules with up-to-date parameters ---
    SENSOR_IFACE_Init(motor_ptr);
    FAULT_PROTECT_Init(motor_ptr);
    OBS_Init(motor_ptr);
    CTRL_FILTS_Init(motor_ptr);
    SPEED_CTRL_Init(motor_ptr);
    VOLT_MOD_Init(motor_ptr);
#if defined(CTRL_METHOD_RFO)
    PHASE_ADV_Init(motor_ptr);
    POSITION_CTRL_Init(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Init(motor_ptr, 1.0f);
    TRQ_Init(motor_ptr);
    DELTA_CTRL_Init(motor_ptr);
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    SIX_PULSE_INJ_Init(motor_ptr);
    HIGH_FREQ_INJ_Init(motor_ptr);
    PROFILER_Init(motor_ptr);
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    HALL_SENSOR_Init(motor_ptr);
    INC_ENCODER_Init(motor_ptr);
#endif
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Init(motor_ptr, 1.0f);
#endif
#if defined(CTRL_METHOD_TBC)
    CURRENT_CTRL_Init(motor_ptr);
#endif
    // --- Step 2: Reset state variables and re-arm the common ISR wrappers ---
    SENSOR_IFACE_Reset(motor_ptr);
    FAULT_PROTECT_Reset(motor_ptr);
    CommonISR0Wrap = CommonISR0;
    CommonISR1Wrap = CommonISR1;
    // --- Step 3: Select feedback ISR based on control mode and sensor type ---
#if defined(CTRL_METHOD_RFO)
    if (Mode(params_ptr, Speed_Mode_FOC_Sensorless_HighFreq_Startup) || Mode(params_ptr, Curr_Mode_FOC_Sensorless_HighFreq_Startup))
    {
        // HFI startup: blended HFI + observer until speed crosses lock-in threshold
        FeedbackISR0Wrap[motor_ptr->motor_instance] = SensorlessFeedbackISR0;
        sm_ptr->vars.high_freq.used = true;
    }
    else if (params_ptr->sys.fb.mode == Hall)
    {
        // Hall sensor: sector edge detection for angle and speed
        HALL_SENSOR_Reset(motor_ptr);
        FeedbackISR0Wrap[motor_ptr->motor_instance] = HALL_SENSOR_RunISR0;
        sm_ptr->vars.high_freq.used = false;
    }
    else if (params_ptr->sys.fb.mode == AqB_Enc)
    {
        // Incremental encoder: quadrature pulse counting
        FeedbackISR0Wrap[motor_ptr->motor_instance] = INC_ENCODER_RunISR0;
        sm_ptr->vars.high_freq.used = false;
    }
    else if (params_ptr->sys.fb.mode == Direct)
    {
        // Direct injection of angle/speed from external source (e.g. dyno test)
        FeedbackISR0Wrap[motor_ptr->motor_instance] = EmptyFcn_PtrArgument;
        sm_ptr->vars.high_freq.used = false;
    }
    else /*Sensor-less*/
    {
        // Pure sensorless: BEMF observer only
        FeedbackISR0Wrap[motor_ptr->motor_instance] = OBS_RunISR0;
        sm_ptr->vars.high_freq.used = false;
    }
#elif defined(CTRL_METHOD_SFO)
    if (Mode(params_ptr, Speed_Mode_FOC_Sensorless_HighFreq_Startup) || Mode(params_ptr, Trq_Mode_FOC_Sensorless_HighFreq_Startup))
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance] = SensorlessFeedbackISR0;
        sm_ptr->vars.high_freq.used = true;
    }
    else
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance] = OBS_RunISR0;
        sm_ptr->vars.high_freq.used = false;
    }
#elif defined(CTRL_METHOD_TBC)
    if (params_ptr->sys.fb.mode == Hall)
    {
        HALL_SENSOR_Reset(motor_ptr);
        FeedbackISR0Wrap[motor_ptr->motor_instance] = HALL_SENSOR_RunISR0;
        BLOCK_COMM_Init(motor_ptr);
        TRAP_COMM_Init(motor_ptr);
    }
#endif
}
/**
 * @brief Reset inter-state control variables to safe initial values
 *
 * Called on every transition back toward Brake_Boot / Init so that
 * integrator states, references, and filter histories do not cause
 * transients when re-entering control states.
 * Does NOT re-initialise module parameters (use ResetAllModules for that).
 */
void STATE_MACHINE_ResetVariable(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_TBC)
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
#endif
    ELEC_t w0 = Elec_Zero;

    CTRL_FILTS_Reset(motor_ptr);
    FAULT_PROTECT_Reset(motor_ptr);
    SPEED_CTRL_Reset(motor_ptr);
    VOLT_CTRL_Reset(motor_ptr);
    CTRL_ResetWcmdInt(motor_ptr, w0);

    vars_ptr->v_qd_r_cmd.d = 0.0f;
    vars_ptr->v_qd_r_cmd.q = 0.0f;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    ELEC_t th0 = {PI_OVER_TWO};
    if (params_ptr->sys.fb.mode == Sensorless)
    {
        AB_t la_ab_lead = (AB_t){(params_ptr->motor.lam + params_ptr->motor.ld * params_ptr->ctrl.align.voltage / params_ptr->motor.r), 0.0f};
        OBS_Reset(motor_ptr, &la_ab_lead, &w0, &th0);
    }
    SIX_PULSE_INJ_Reset(motor_ptr);
    TRQ_Reset(motor_ptr);
    HIGH_FREQ_INJ_Reset(motor_ptr, Elec_Zero, motor_ptr->ctrl_ptr->six_pulse_inj.th_r_est);
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    if (params_ptr->sys.fb.mode == Hall)
    {
        HALL_SENSOR_Reset(motor_ptr);
    }
    CURRENT_CTRL_Reset(motor_ptr);
    vars_ptr->i_cmd_int = 0.0f;

#endif
#if defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);
    vars_ptr->T_cmd_int = 0.0f;
    vars_ptr->delta_cmd.elec = 0.0f;
    vars_ptr->T_cmd_final = 0.0f;
    vars_ptr->la_cmd_final = 0.0f;
    vars_ptr->la_qd_s_est.d = 0.0f;
    vars_ptr->v_qd_s_cmd.d = 0.0f;
    vars_ptr->v_qd_s_cmd.q = 0.0f;
#endif
#if defined(CTRL_METHOD_RFO)
    if (params_ptr->sys.fb.mode == AqB_Enc)
    {
        INC_ENCODER_Reset(motor_ptr, th0);
    }
    FLUX_WEAKEN_Reset(motor_ptr);
    vars_ptr->i_qd_r_cmd.d = 0.0f;
    vars_ptr->i_qd_r_cmd.q = 0.0f;

    vars_ptr->i_qd_r_fb.d = 0.0f;
    vars_ptr->i_qd_r_fb.q = 0.0f;
    POSITION_CTRL_Reset(motor_ptr);
#endif
#if defined(CTRL_METHOD_TBC)
    TRAP_COMM_Reset(motor_ptr);
    vars_ptr->v_s_cmd.rad = 0.0f;
#endif
}
/**
 * @brief Cycle bootstrap charge through all three gate-driver phases
 *
 * Called each ISR0 cycle during Boot_Only mode to charge the bootstrap
 * capacitors on all three high-side gate drivers before enabling the inverter.
 * Each call enables exactly one phase (exits high-Z) and places the other two
 * in high-Z, cycling U -> V -> W -> U on successive calls via scheudle_counter.
 *
 * @param motor_ptr Pointer to motor instance
 */
void BootStrapChargePerPhase(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    switch (sm_ptr->vars.brake_boot.scheudle_counter)
    {
    default:
    case 0:                                                            // Charge phase U high-side bootstrap capacitor
        hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance | 0x20);  // Phase U
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance | 0x40); // Phase V
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance | 0x80); // Phase W
        sm_ptr->vars.brake_boot.scheudle_counter = 1;
        break;
    case 1:                                                            // Charge phase V high-side bootstrap capacitor
        hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance | 0x40);  // Phase V
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance | 0x20); // Phase U
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance | 0x80); // Phase W
        sm_ptr->vars.brake_boot.scheudle_counter = 2;
        break;
    case 2:                                                            // Charge phase W high-side bootstrap capacitor
        hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance | 0x80);  // Phase W
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance | 0x20); // Phase U
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance | 0x40); // Phase V
        sm_ptr->vars.brake_boot.scheudle_counter = 0;
        break;
    }
}

/**
 * @brief Entry function for the Init state
 *
 * On first power-up (param_init_done == false):
 *   1. Disables all gate drivers and peripherals
 *   2. Silences the common ISRs to prevent spurious sensor/fault activity
 *   3. Loads motor parameters and re-initialises all hardware peripherals
 *   4. Resets all control modules to a known state
 * On every entry (including fault-clear re-entries):
 *   - Resets all control variables
 *   - Arms the ADC offset-nulling timer (current sensor calibration window)
 */
static void InitEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    // Immediately disable inverter output for safety
    hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance);

    if (!sm_ptr->vars.init.param_init_done) // only after booting up
    {
        // Silence all ISRs and PWM outputs before touching hardware configuration
        hw_fcn.StopPeripherals(motor_ptr->motor_instance); // disable all ISRs, PWMs, ADCs, etc.

        CommonISR0Wrap = EmptyFcn;
        CommonISR1Wrap = EmptyFcn;
        FeedbackISR0Wrap[motor_ptr->motor_instance] = OBS_RunISR0;
#if !defined(PC_TEST)
        // Load motor parameters from flash / user-defined InitManual function
        PARAMS_Init(motor_ptr);
#endif
        // Re-initialise hardware (timers, ADC, PWM) with the newly loaded parameters
        hw_fcn.HardwareIfaceInit(motor_ptr->motor_instance); // all peripherals must stop before re-initializing
        STATE_MACHINE_ResetAllModules(motor_ptr);
        sm_ptr->vars.init.param_init_done = true;
        vars_ptr->en = true;
    }
    // Always reset control variables on every Init entry (including fault-clear)
    STATE_MACHINE_ResetVariable(motor_ptr);

    // Arm the offset-nulling window: current references held at zero while
    // the ADC accumulates and cancels DC offsets on all current channels
    // TBD: check offset nulling timer
    StopWatchInit(&sm_ptr->vars.init.timer, params_ptr->sys.analog.offset_null_time, params_ptr->sys.samp.ts0);
    sm_ptr->vars.speed_reset_required = false;
    sm_ptr->vars.init.offset_null_done = false;
    SM_RuntimeFaultEnDis(motor_ptr, Dis);
}

/**
 * @brief ISR0 handler for the Init state — ADC offset nulling
 *
 * Counts down the offset-nulling timer. While it runs,
 * calls the offset-nulling routine each cycle so current-sensor
 * DC offsets are learned iteratively. Once done, sets offset_null_done
 * which unblocks fault protection and enables the Brake_Boot transition.
 */
RAMFUNC_BEGIN
static void InitISR0(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    if (sm_ptr->vars.init.param_init_done)
    {
        StopWatchRun(&sm_ptr->vars.init.timer);
        if (StopWatchIsDone(&sm_ptr->vars.init.timer))
        {
            // Nulling complete: release the CommonISR0 gate on fault protection
            sm_ptr->vars.init.offset_null_done = true;
        }
        else
        {
            // Still accumulating: refine current-sensor DC offset estimates
            SENSOR_IFACE_OffsetNullISR0(motor_ptr);
        }
    }
}
RAMFUNC_END

/**
 * @brief Entry function for the Brake_Boot state
 *
 * Prepares the inverter for the brake/bootstrap phase:
 * - Clears all duty-cycle commands and speed/current integrators
 * - Arms the boot timer
 * - In Boot_and_Brake mode: enables all phases (active braking)
 * - In Boot_Only mode: places all phases in high-Z and starts cycling
 *   bootstrap capacitor charging via BootStrapChargePerPhase()
 * Critical writes are wrapped in a critical section to prevent ISR0
 * from reading a partially-updated state.
 */
static void BrakeBootEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    StopWatchInit(&sm_ptr->vars.brake_boot.timer, params_ptr->sys.boot_time, params_ptr->sys.samp.ts0);

    hw_fcn.EnterCriticalSection(); // --------------------
    // atomic operations needed for struct writes and no more modification until next state
    CTRL_FILTS_Reset(motor_ptr);
    vars_ptr->d_uvw_cmd = UVW_Zero;
    vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
    vars_ptr->w_cmd_int.elec = 0.0f;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    vars_ptr->i_cmd_int = 0.0f;
#elif defined(CTRL_METHOD_SFO)
    vars_ptr->T_cmd_int = 0.0f;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    if (Mode(params_ptr, Profiler_Mode))
    {
        sm_ptr->add_callback.RunISR0 = EmptyFcn;
    }
#endif
    sm_ptr->current = sm_ptr->next; // must be in critical section for brake-boot entry
    SM_RuntimeFaultEnDis(motor_ptr, Dis);

    if (params_ptr->sys.boot_mode == Boot_and_Brake)
    {
        hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
    }
    else
    {
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance);
    }

    hw_fcn.ExitCriticalSection(); // --------------------
}

/**
 * @brief ISR0 handler for the Brake_Boot state
 *
 * Advances the boot timer each cycle. In Hall-sensor feedback mode (RFO/TBC)
 * runs the Hall-sensor ISR so speed is tracked during braking. In Boot_Only
 * mode cycles bootstrap capacitor charging across all three phases via
 * BootStrapChargePerPhase().
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void BrakeBootISR0(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
#endif
    StopWatchRun(&sm_ptr->vars.brake_boot.timer);
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    if (params_ptr->sys.fb.mode == Hall)
    {
        FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
        vars_ptr->w_final.elec = vars_ptr->w_hall.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
    }

    if (params_ptr->sys.boot_mode == Boot_Only)
    {
        BootStrapChargePerPhase(motor_ptr);
    }
#endif
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Brake_Boot state
 *
 * Clears the speed_reset_required flag once the external speed command
 * drops below the V/Hz low threshold with hysteresis, allowing a clean
 * re-entry into a speed-controlled state on the next command assertion.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void BrakeBootISR1(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    if (sm_ptr->vars.speed_reset_required && ABS_BELOW_LIM(vars_ptr->w_cmd_ext.elec, params_ptr->ctrl.volt.w_thresh.elec - params_ptr->ctrl.volt.w_hyst.elec))
    {
        sm_ptr->vars.speed_reset_required = false;
    }
}

/**
 * @brief Exit function for the Brake_Boot state
 *
 * In Boot_Only mode: exits gate-driver high-Z and pre-loads all three PWM
 * compare registers to 50 % duty so the inverter starts from a balanced
 * mid-rail state when the next control state takes over.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void BrakeBootExit(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    if (params_ptr->sys.boot_mode == Boot_Only)
    {
        hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
        vars_ptr->d_uvw_cmd = UVW_Half;
        vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
    }
}

/**
 * @brief Entry function for the Volt_OL (open-loop V/Hz) state
 *
 * Prepares for open-loop voltage control:
 * - Resets the speed integrator to the V/Hz threshold for bumpless entry
 * - Clears d-axis voltage and resets the V/Hz controller state
 * - Disables hybrid modulation for single-shunt (needs minimum zero-vector
 *   time for current sampling)
 * - In Profiler mode (SFO): registers the profiler as the ISR0 add-callback
 * - Enables phase-loss detection once flux is established
 */
static void VoltOLEntry(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    CTRL_ResetWcmdInt(motor_ptr, params_ptr->ctrl.volt.w_thresh);
    vars_ptr->v_qd_r_cmd.d = 0.0f;
    VOLT_CTRL_Reset(motor_ptr);
    hw_fcn.EnterCriticalSection(); // --------------------
    if ((params_ptr->sys.analog.shunt.type == Single_Shunt) && (params_ptr->sys.analog.shunt.single_shunt.type == Hyb_Mod))
    {
        VOLT_MOD_EnDisHybMod(motor_ptr, Dis);
    }
#if defined(CTRL_METHOD_SFO)
    if (Mode(params_ptr, Profiler_Mode))
    {
        sm_ptr->add_callback.RunISR0 = PROFILER_RunISR0;
    }
#endif

    if (params_ptr->sys.rate_lim.w_ol_cmd.elec == 0) /* to maintain backward compataility */
    {
        params_ptr->sys.rate_lim.w_ol_cmd.elec = params_ptr->sys.rate_lim.w_cmd.elec;
    }
    sm_ptr->current = sm_ptr->next; // must be in critical section
    SM_RuntimeFaultEnDis(motor_ptr, En);
    hw_fcn.ExitCriticalSection(); // --------------------
}

/**
 * @brief Exit function for the Volt_OL state
 *
 * Re-enables single-shunt hybrid modulation (if configured) so that
 * current reconstruction is available in the subsequent closed-loop state.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void VoltOLExit(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    if ((params_ptr->sys.analog.shunt.type == Single_Shunt) && (params_ptr->sys.analog.shunt.single_shunt.type == Hyb_Mod))
    {
        VOLT_MOD_EnDisHybMod(motor_ptr, En);
    }
}

/**
 * @brief ISR0 handler for the Volt_OL state
 *
 * Runs the open-loop V/Hz angle integrator and voltage ramp, then converts
 * the resulting voltage vector to PWM duty cycles.
 */
RAMFUNC_BEGIN
static void VoltOLISR0(MOTOR_t *motor_ptr)
{
    VOLT_CTRL_RunISR0(motor_ptr); // advance angle integrator, compute v_alpha/beta
    VOLT_MOD_RunISR0(motor_ptr);  // SVPWM: convert v_ab to PWM duty cycles
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Volt_OL state
 *
 * Slow-rate tasks:
 * - Rate-limits and integrates the external speed command (separate OL limit)
 * - Applies the V/Hz law: Vq = max(V_min + |w| * V/Hz_ratio, 0) * direction
 * - Updates all slow-rate control filters
 */
static void VoltOLISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    // Rate-limit the open-loop speed reference (uses dedicated OL rate limit)
    CTRL_UpdateWcmdIntISR1(motor_ptr, vars_ptr->w_cmd_ext, params_ptr->sys.rate_lim.w_ol_cmd.elec);

    // V/Hz law: voltage proportional to frequency, clamped to V_min to
    // maintain sufficient flux at very low speeds
    vars_ptr->v_qd_r_cmd.q = MAX(params_ptr->ctrl.volt.v_min + ABS(vars_ptr->w_cmd_int.elec) * params_ptr->ctrl.volt.v_to_f_ratio, 0.0f) * vars_ptr->dir;
    CTRL_FILTS_RunAllISR1(motor_ptr);
}

/**
 * @brief Entry function for the Speed_CL (closed-loop speed) state
 *
 * Conditions speed/current integrators for bumpless transfer:
 * - From Align/Six_Pulse/High_Freq/Brake_Boot: resets speed integrator to
 *   the appropriate threshold (observer lock-in or V/Hz threshold)
 *   and clears current/flux integrators.
 * - From Catch_Spin (RFO): primes the speed integrator with the measured
 *   coasting speed to avoid a reference step.
 * - From Align/Six_Pulse/High_Freq (SFO): resets at the High_Freq threshold.
 * - From Brake_Boot (TBC): resets commutation and current controllers.
 */
static void SpeedCLEntry(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_TBC)
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
#endif

#if defined(CTRL_METHOD_RFO)
    ELEC_t w0;
    if ((sm_ptr->current == Align) || (sm_ptr->current == Six_Pulse) || (sm_ptr->current == High_Freq) || (sm_ptr->current == Brake_Boot))
    {
        w0 = (((sm_ptr->current == Align) || (sm_ptr->current == Six_Pulse)) && (params_ptr->sys.fb.mode == Sensorless)) ? params_ptr->obs.w_thresh : params_ptr->ctrl.volt.w_thresh;
        CTRL_ResetWcmdInt(motor_ptr,w0);
        SPEED_CTRL_Reset(motor_ptr);
        vars_ptr->i_cmd_int = 0.0f;
        FLUX_WEAKEN_Reset(motor_ptr);
    }
    else if (sm_ptr->current == Catch_Spin)
    {
        vars_ptr->w_cmd_int.elec = vars_ptr->w_final_filt.elec; // directly setting the cmd int value to avoid sign change
        SPEED_CTRL_Reset(motor_ptr);
        vars_ptr->i_cmd_int = 0.0f;
        FLUX_WEAKEN_Reset(motor_ptr);
    }
#elif defined(CTRL_METHOD_SFO)
    if ((sm_ptr->current == Align) || (sm_ptr->current == Six_Pulse) || (sm_ptr->current == High_Freq))
    {
        CTRL_ResetWcmdInt(motor_ptr, (sm_ptr->current == High_Freq) ? params_ptr->ctrl.volt.w_thresh : params_ptr->obs.w_thresh);
        SPEED_CTRL_Reset(motor_ptr);
        vars_ptr->T_cmd_int = 0.0f;
    }
#elif defined(CTRL_METHOD_TBC)
    if (sm_ptr->current == Brake_Boot)
    {
        CTRL_ResetWcmdInt(motor_ptr, params_ptr->ctrl.volt.w_thresh);
        SPEED_CTRL_Reset(motor_ptr);
        vars_ptr->i_cmd_int = 0.0f;
        CURRENT_CTRL_Reset(motor_ptr);
        TRAP_COMM_Reset(motor_ptr);
    }
#endif
    SM_RuntimeFaultEnDis(motor_ptr, En);
}

#if defined(CTRL_METHOD_SFO)
/**
 * @brief Superimpose HFI excitation voltage onto the alpha-beta command (SFO)
 *
 * If high-frequency injection is currently active (sm_vars.high_freq.used),
 * adds the HFI excitation vector (v_ab_cmd from the HFI controller) to the
 * total alpha-beta voltage command. Called from the SFO speed/torque CL ISR0
 * path after the inverse Park transform.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static inline void AddHighFreqVoltsIfNeeded(MOTOR_t *motor_ptr)
{ // adding high frequency excitation voltage components if needed
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    if (sm_ptr->vars.high_freq.used)
    {
        vars_ptr->v_ab_cmd_tot.alpha += ctrl_ptr->high_freq_inj.v_ab_cmd.alpha;
        vars_ptr->v_ab_cmd_tot.beta += ctrl_ptr->high_freq_inj.v_ab_cmd.beta;
    }
}
RAMFUNC_END
#endif

/**
 * @brief ISR0 handler for the Speed_CL state
 *
 * Full closed-loop speed control fast path:
 * 1. Run the selected feedback source (observer/HFI/Hall/encoder/direct)
 * 2. Route feedback to unified w_final / th_r_final variables
 * 3. Run the method-specific control path:
 *    - RFO: current controller -> torque observer -> voltage modulator
 *    - SFO: flux -> torque observer -> torque ctrl -> delta ctrl ->
 *           inverse Park -> (add HFI voltage if active) -> voltage modulator
 *    - TBC: block or trapezoidal commutation -> torque observer
 */
RAMFUNC_BEGIN
static void SpeedCLISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    // Step 1: update angle and speed estimates from the configured sensor
    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    // Step 2: route sensor output to the common w_final / th_r_final interface
    switch (params_ptr->sys.fb.mode)
    {
    default:
    case Sensorless:
        vars_ptr->w_final.elec = vars_ptr->w_est.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
        break;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    case Hall:
        vars_ptr->w_final.elec = vars_ptr->w_hall.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
        break;
    case AqB_Enc:
        vars_ptr->w_final.elec = vars_ptr->w_enc.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_enc.elec;
        break;
    case Direct:
        vars_ptr->w_final.elec = vars_ptr->w_fb.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_fb.elec;
        break;
#endif
    }
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_RunISR0(motor_ptr);
    TRQ_RunObsISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_RunISR0(motor_ptr);
    TRQ_RunObsISR0(motor_ptr);
    TRQ_RunCtrlISR0(motor_ptr);
    DELTA_CTRL_RunISR0(motor_ptr);
    ParkTransformInv(&vars_ptr->v_qd_s_cmd, &vars_ptr->park_s, &vars_ptr->v_ab_cmd);
    vars_ptr->v_ab_cmd_tot = vars_ptr->v_ab_cmd;
    AddHighFreqVoltsIfNeeded(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);

#elif defined(CTRL_METHOD_TBC)
    switch (params_ptr->ctrl.tbc.mode)
    {
    case Block_Commutation:
    default:
        CURRENT_CTRL_RunISR0(motor_ptr);
        BLOCK_COMM_RunVoltModISR0(motor_ptr);
        break;
    case Trapezoidal_Commutation:
        TRAP_COMM_RunISR0(motor_ptr);
        break;
    }
    TRQ_RunObsISR0(motor_ptr);
#endif
}
RAMFUNC_END
/**
 * @brief ISR1 handler for the Speed_CL state
 *
 * Slow-rate tasks for closed-loop speed control:
 * 1. Rate-limits and integrates the external speed command
 * 2. Runs all slow-rate control filters
 * 3. Runs the speed PI controller -> produces i_cmd_spd / T_cmd_spd
 * 4. Applies protection limits (I2T or torque) and rate-limits the reference
 * 5. Method-specific adjustments (phase advance, MTPA look-up, flux weakening)
 */
static void SpeedCLISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_TBC)
    PROTECT_t *protect_ptr = motor_ptr->protect_ptr;
#endif

    // Step 1: rate-limit and advance the closed-loop speed reference
    CTRL_UpdateWcmdIntISR1(motor_ptr, vars_ptr->w_cmd_ext, params_ptr->sys.rate_lim.w_cmd.elec);
    CTRL_FILTS_RunAllISR1(motor_ptr);
    // Step 2: run speed PI -> produces i_cmd_spd / T_cmd_spd
    SPEED_CTRL_RunISR1(motor_ptr);
    // Step 3: apply protection limits and rate-limit the current/torque reference
#if defined(CTRL_METHOD_RFO)
    vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_spd);
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
    PHASE_ADV_RunISR1(motor_ptr);
    FLUX_WEAKEN_RunISR1(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FAULT_PROTECT_RunTrqLimitCtrlISR1(motor_ptr);
    vars_ptr->T_cmd_prot = SAT(-protect_ptr->motor.T_lmt, protect_ptr->motor.T_lmt, vars_ptr->T_cmd_spd);
    vars_ptr->T_cmd_int = RateLimit(params_ptr->sys.rate_lim.T_cmd * params_ptr->sys.samp.ts1, vars_ptr->T_cmd_prot, vars_ptr->T_cmd_int);
    vars_ptr->la_cmd_mtpa = LUT1DInterp(&params_ptr->motor.mtpa_lut, ABS(vars_ptr->T_cmd_int));
    FLUX_WEAKEN_RunISR1(motor_ptr);
#elif defined(CTRL_METHOD_TBC)
    vars_ptr->i_cmd_prot = (params_ptr->ctrl.curr.bypass == false) ? SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_spd) : vars_ptr->i_cmd_spd;
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
#endif
}

#if defined(CTRL_METHOD_RFO)

/**
 * @brief Entry function for the Position_CL (closed-loop position) state (RFO)
 *
 * Seeds the position command integrator with the current measured angle to
 * avoid a reference step on entry. From Align or Brake_Boot, resets the
 * speed integrator to zero and clears the current and flux-weakening
 * integrators before the position loop takes over.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void PositionCLEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    switch (params_ptr->sys.fb.mode)
    {
    default:
    case AqB_Enc:
        vars_ptr->th_r_final.mech = vars_ptr->th_r_enc.mech;
        break;
    case Direct:
        vars_ptr->th_r_final.mech = vars_ptr->th_r_fb.mech;
        break;
    }

    POSITION_CTRL_cmdInt(motor_ptr, vars_ptr->th_r_final.mech);

    if ((sm_ptr->current == Align) || (sm_ptr->current == Brake_Boot))
    {
        ELEC_t w0 = Elec_Zero;
        CTRL_ResetWcmdInt(motor_ptr, w0);
        SPEED_CTRL_Reset(motor_ptr);
        vars_ptr->i_cmd_int = 0.0f;
        FLUX_WEAKEN_Reset(motor_ptr);
    }
}

/**
 * @brief ISR0 handler for the Position_CL state (RFO)
 *
 * Fast closed-loop position control path:
 * 1. Reads angle and speed from the configured feedback source
 * 2. Routes feedback to w_final, th_r_final.elec, and th_r_final.mech
 * 3. Runs the current controller, torque observer, and voltage modulator
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void PositionCLISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    switch (params_ptr->sys.fb.mode)
    {
    default:
    case AqB_Enc:
        vars_ptr->w_final.elec = vars_ptr->w_enc.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_enc.elec;
        vars_ptr->th_r_final.mech = vars_ptr->th_r_enc.mech;
        break;
    case Direct:
        vars_ptr->w_final.elec = vars_ptr->w_fb.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_fb.elec;
        vars_ptr->th_r_final.mech = vars_ptr->th_r_fb.mech;
        break;
    }

    CURRENT_CTRL_RunISR0(motor_ptr);
    TRQ_RunObsISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Position_CL state (RFO)
 *
 * Slow-rate tasks for closed-loop position control:
 * - Rate-limits the external position reference
 * - Runs the position PI controller to produce an inner speed reference
 * - Runs all slow-rate control filters
 * - Runs the speed PI controller and applies I2T current-limit protection
 * - Rate-limits the current reference, runs phase advance and flux weakening
 *
 * @param motor_ptr Pointer to motor structure
 */
static void PositionCLISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    PROTECT_t *protect_ptr = motor_ptr->protect_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    vars_ptr->p_cmd_int = RateLimit(params_ptr->sys.rate_lim.p_cmd * params_ptr->sys.samp.ts1, vars_ptr->p_cmd_ext, vars_ptr->p_cmd_int);

    POSITION_CTRL_RunISR1(motor_ptr);

#if (0) // if speed ramp
    vars_ptr->w_cmd_ext.elec = ctrl_ptr->position.pi.output;
    CTRL_UpdateWcmdIntISR1(motor_ptr, vars_ptr->w_cmd_ext, params_ptr->sys.rate_lim.w_cmd.elec);
#else
    vars_ptr->w_cmd_int.elec = ctrl_ptr->position.pi.output;
#endif

    CTRL_FILTS_RunAllISR1(motor_ptr);

    SPEED_CTRL_RunISR1(motor_ptr);

    vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_spd);
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
    PHASE_ADV_RunISR1(motor_ptr);
    FLUX_WEAKEN_RunISR1(motor_ptr);
}

/**
 * @brief Entry function for the Catch_Spin state (RFO)
 *
 * Prepares for re-synchronisation to a coasting motor:
 * - In sensorless / zero-current-control mode: resets the current controller
 *   (feed-forward disabled) and observer, selects v_ab_cmd as the observation
 *   voltage source, and zeroes PWM duties.
 * - In direct BEMF measurement mode: enters gate-driver high-Z and selects
 *   v_ab_fb as the observation source.
 * - Arms the catch-spin timer and flags catch as not yet complete.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void CatchSpinEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    sm_ptr->vars.catch_spin.done = false;
    StopWatchInit(&sm_ptr->vars.catch_spin.timer, params_ptr->sys.catch_spin.time, params_ptr->sys.samp.ts1);
    sm_ptr->vars.catch_spin.bemf_measurement = (params_ptr->sys.catch_spin.mode == Direct_Bemf_Measure);

    hw_fcn.EnterCriticalSection(); // --------------------

    if (params_ptr->sys.fb.mode == Sensorless)
    {
        // atomic operations needed for struct writes and no more modification until next state
        vars_ptr->i_cmd_int = 0.0f;
        CURRENT_CTRL_Reset(motor_ptr);
        ctrl_ptr->curr.en_ff = false;
        vars_ptr->i_qd_r_cmd.d = 0.0f;
        vars_ptr->i_qd_r_cmd.q = 0.0f;

        switch (params_ptr->sys.catch_spin.mode)
        {
        default:
        case Zero_Current_Control:
            vars_ptr->v_ab_obs = &vars_ptr->v_ab_cmd;
            break;
        case Direct_Bemf_Measure:
            vars_ptr->v_ab_obs = &vars_ptr->v_ab_fb;
            hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance);
            break;
        }
        vars_ptr->d_uvw_cmd = UVW_Zero;
        vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
        AB_t la_ab_lead = AB_Zero;
        ELEC_t w0 = Elec_Zero;
        ELEC_t th0 = Elec_Zero;
        OBS_Reset(motor_ptr, &la_ab_lead, &w0, &th0);
    }
    CTRL_FILTS_Reset(motor_ptr);

    sm_ptr->current = sm_ptr->next; // must be in critical section for dyno_lock entry

    hw_fcn.ExitCriticalSection();
}

/**
 * @brief ISR0 handler for the Catch_Spin state (RFO)
 *
 * Runs the observer feedback and routes output to w_final / th_r_final.
 * In sensorless zero-current-control mode also runs the current controller
 * and voltage modulator to maintain zero current while tracking the BEMF.
 * In direct BEMF measurement mode the inverter stays in high-Z; no PWM.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void CatchSpinISR0(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);

    switch (params_ptr->sys.fb.mode)
    {
    default:
    case Sensorless:
        vars_ptr->w_final.elec = vars_ptr->w_est.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
        if (sm_ptr->vars.catch_spin.bemf_measurement == false /*Zero_Current_Control*/)
        {
            CURRENT_CTRL_RunISR0(motor_ptr);
            VOLT_MOD_RunISR0(motor_ptr);
        }
        break;
    case Direct:
        vars_ptr->w_final.elec = vars_ptr->w_fb.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_fb.elec;
        break;
    case Hall:
        vars_ptr->w_final.elec = vars_ptr->w_hall.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
        break;
    }
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Catch_Spin state (RFO)
 *
 * Once the catch timer expires, determines the outcome:
 * - In direct BEMF mode and speed above threshold: atomically switches the
 *   voltage observation source back to v_ab_cmd and re-enables gate drivers.
 * - Otherwise: sets catch_spin.done to signal the condition checker that
 *   the catch attempt is complete and a state transition can occur.
 * Always advances the catch timer and runs all slow-rate filters.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void CatchSpinISR1(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    CTRL_FILTS_RunAllISR1(motor_ptr);

    if (StopWatchIsDone(&sm_ptr->vars.catch_spin.timer))
    {
	    if ((sm_ptr->vars.catch_spin.bemf_measurement == true ) && (ABS(vars_ptr->w_final_filt.elec)>=params_ptr->sys.catch_spin.w_thresh.elec ) && (params_ptr->sys.fb.mode == Sensorless))
        {   
		    hw_fcn.EnterCriticalSection();		// --------------------
            StopWatchReset(&sm_ptr->vars.catch_spin.timer);
            vars_ptr->v_ab_cmd = vars_ptr->v_ab_fb; // atomic operations needed
            vars_ptr->v_ab_obs = &vars_ptr->v_ab_cmd;
            sm_ptr->vars.catch_spin.bemf_measurement = false;
            hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
            hw_fcn.ExitCriticalSection(); // --------------------
        }
        else
        {
            sm_ptr->vars.catch_spin.done = true;
        }
    }
    StopWatchRun(&sm_ptr->vars.catch_spin.timer);
}

/**
 * @brief Exit function for the Catch_Spin state (RFO)
 *
 * In sensorless mode re-enables the current-controller feed-forward path.
 * If still in direct BEMF measurement mode at exit, atomically copies
 * v_ab_fb to v_ab_cmd and exits gate-driver high-Z so the next state
 * starts driving the inverter with a conditioned voltage reference.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void CatchSpinExit(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    if (params_ptr->sys.fb.mode == Sensorless)
    {
        ctrl_ptr->curr.en_ff = true;

        if (sm_ptr->vars.catch_spin.bemf_measurement == true)
        {
            hw_fcn.EnterCriticalSection();          // --------------------
            vars_ptr->v_ab_cmd = vars_ptr->v_ab_fb; // atomic operations needed
            vars_ptr->v_ab_obs = &vars_ptr->v_ab_cmd;
            hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
            hw_fcn.ExitCriticalSection();
        }
    }
}
#endif

/**
 * @brief Entry function for the Fault state
 *
 * Applies the configured fault reaction:
 * - Short_Motor: exits gate-driver high-Z and drives all three phases to the
 *   short method (Low-side = 0, High-side = 1, Alternate = 0.5 duty cycle).
 * - High_Z: enters gate-driver high-Z on all phases.
 * Atomically resets filter state, writes the duty-cycle commands, and updates
 * the current state ID inside a critical section. Disables runtime fault
 * protection and arms the fault-clear retry timer.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void FaultEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    FAULTS_t *faults_ptr = motor_ptr->faults_ptr;

#if defined(CTRL_METHOD_TBC)
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
    UVW_t d_uvw_cmd = UVW_Zero;
    if (faults_ptr->reaction == Short_Motor)
    {
        hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
#if defined(CTRL_METHOD_TBC)
        ctrl_ptr->block_comm.exit_high_z_flag.u = true;
        ctrl_ptr->block_comm.exit_high_z_flag.v = true;
        ctrl_ptr->block_comm.exit_high_z_flag.w = true;
#endif
        switch (params_ptr->sys.faults.short_method)
        {
        case Low_Side_Short:
            d_uvw_cmd = UVW_Zero;
            break;
        case High_Side_Short:
            d_uvw_cmd = UVW_One;
            break;
        case Alternate_Short:
        default:
            d_uvw_cmd = UVW_Half;
            break;
        }
    }
    else if (faults_ptr->reaction == High_Z)
    {
        hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance);
#if defined(CTRL_METHOD_TBC)
        ctrl_ptr->block_comm.enter_high_z_flag.w = true;
        ctrl_ptr->block_comm.enter_high_z_flag.v = true;
        ctrl_ptr->block_comm.enter_high_z_flag.w = true;
#endif
    } // "No_Reaction" is not reachable

    sm_ptr->vars.fault.clr_success = false;
    sm_ptr->vars.fault.clr_request = false;

    hw_fcn.EnterCriticalSection(); // --------------------
    CTRL_FILTS_Reset(motor_ptr);
    vars_ptr->d_uvw_cmd = d_uvw_cmd;
    vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
    sm_ptr->current = sm_ptr->next; // must be in critical section
    SM_RuntimeFaultEnDis(motor_ptr, Dis);
    hw_fcn.ExitCriticalSection(); // --------------------
    vars_ptr->clr_faults = false;

    StopWatchInit(&sm_ptr->vars.fault.clr_try_timer, params_ptr->sys.faults.clr_try_period, params_ptr->sys.samp.ts1);
}

/**
 * @brief ISR1 handler for the Fault state — fault-clear management
 *
 * Handles four fault-clear paths each ISR1 cycle:
 * 1. Pending clear request: executes FAULT_PROTECT_ClearFaults and marks success.
 * 2. Latched brk/em_stop removal: issues a clear request when the hardware
 *    signal is deasserted.
 * 3. User-initiated clear: requests a clear when vars_ptr->clr_faults is set
 *    and no brk/em_stop latch is active.
 * 4. Auto-clear retry: counts attempts against max_clr_tries and issues a
 *    clear request at each timer expiry when the command is below the threshold.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void FaultISR1(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    FAULTS_t *faults_ptr = motor_ptr->faults_ptr;

    bool clr_faults_request = vars_ptr->clr_faults && (!faults_ptr->flags_latched.sw.brk) && (!faults_ptr->flags_latched.sw.em_stop);

    bool clr_faults_and_count =  (params_ptr->sys.cmd.source == Internal) && (vars_ptr->cmd_final < params_ptr->sys.faults.cmd_clr_thresh)&&  (!faults_ptr->flags_latched.sw.brk) && (!faults_ptr->flags_latched.sw.em_stop);

    /* Move SM from Fault to init state, while remove the emergency stop or brake*/
    bool clr_faults_no_count = ((!faults_ptr->flags.sw.brk) && (faults_ptr->flags_latched.sw.brk)) || ((!faults_ptr->flags.sw.em_stop) && (faults_ptr->flags_latched.sw.em_stop));

    /* If a fault clear request is already pending, execute the clear:
     * deassert the request flag, invoke the fault protection clear routine,
     * and mark the clear operation as successful that is used to transit to init state. */
    if (sm_ptr->vars.fault.clr_request)
    {
        sm_ptr->vars.fault.clr_request = false;
        FAULT_PROTECT_ClearFaults(motor_ptr);
        sm_ptr->vars.fault.clr_success = true;
    }
    /* If a latched brake or emergency-stop condition has been removed, request a fault
     * clear. */
    else if (clr_faults_no_count)
    {
        sm_ptr->vars.fault.clr_request = true;
    }
    /* If an external (user-initiated) clear-faults command is active and neither
     * brake nor emergency-stop latched faults are present, request a fault clear. */
    else if (clr_faults_request)
    {
        sm_ptr->vars.fault.clr_request = true;
    }
    /* If the auto-clear conditions are met (internal command source, command below
     * the clear threshold, no latched brk/em_stop) and the retry count has not
     * reached the configured maximum (max_clr_tries == 0xFFFFFFFF means unlimited),
     * run the retry timer and issue a clear request once the timer expires, then
     * reset the timer for the next attempt. */
    else if ((clr_faults_and_count) && ((sm_ptr->vars.fault.clr_try_cnt < params_ptr->sys.faults.max_clr_tries) || (params_ptr->sys.faults.max_clr_tries == 0xFFFFFFFF)))
    {
        StopWatchRun(&sm_ptr->vars.fault.clr_try_timer);

        if (StopWatchIsDone(&sm_ptr->vars.fault.clr_try_timer))
        {
            sm_ptr->vars.fault.clr_request = true;
            ++sm_ptr->vars.fault.clr_try_cnt;
            StopWatchReset(&sm_ptr->vars.fault.clr_try_timer);
        }
    }
}

/**
 * @brief Exit function for the Fault state
 *
 * Exits gate-driver high-Z (re-enables all phases) so the subsequent Init
 * state can drive the inverter. In TBC mode also sets the block-commutation
 * exit-high-Z flags for all three phases.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void FaultExit(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_TBC)
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
#endif

    hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
#if defined(CTRL_METHOD_TBC)
    ctrl_ptr->block_comm.exit_high_z_flag.u = true;
    ctrl_ptr->block_comm.exit_high_z_flag.v = true;
    ctrl_ptr->block_comm.exit_high_z_flag.w = true;
#endif
}

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)

/**
 * @brief Entry function for the Align state (RFO/SFO)
 *
 * Forces the rotor to a known electrical angle by applying a fixed voltage:
 * - Resets the V/Hz controller.
 * - Sets th_r_cmd to -90° (CW) or +90° (CCW) for the first half of alignment.
 * - Applies the configured alignment voltage on the Q-axis (D-axis = 0).
 * - Arms the alignment timer.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void AlignEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    VOLT_CTRL_Reset(motor_ptr);
#if defined(PC_TEST)
    vars_ptr->th_r_cmd.elec = 0;
#else
    vars_ptr->th_r_cmd.elec = vars_ptr->dir * -1.0f * PI_OVER_TWO; // Align twice, first align at -90 [deg] for CW or  90 [deg] for CCW
#endif
    vars_ptr->v_qd_r_cmd = (QD_t){params_ptr->ctrl.align.voltage, 0.0f};
    StopWatchInit(&sm_ptr->vars.align.timer, params_ptr->ctrl.align.time, params_ptr->sys.samp.ts0);
}

/**
 * @brief ISR0 handler for the Align state (RFO/SFO)
 *
 * Runs the V/Hz voltage controller and voltage modulator each ISR0 cycle.
 * Halfway through the alignment time, steps th_r_cmd to 0° to complete
 * the second phase of alignment before transitioning out.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void AlignISR0(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    VOLT_CTRL_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
    StopWatchRun(&sm_ptr->vars.align.timer);

    if ((StopWatchGetTime(&sm_ptr->vars.align.timer) * 2) >= params_ptr->ctrl.align.time)
    {
        vars_ptr->th_r_cmd.elec = 0;
    }
}
RAMFUNC_END

/**
 * @brief Exit function for the Align state (RFO/SFO)
 *
 * Prepares controllers for bumpless transfer to the next state:
 * - RFO: resets the current controller; resets the incremental encoder
 *   angle reference in encoder-feedback mode.
 * - SFO: resets the flux, torque, and delta controllers.
 * In sensorless mode resets the observer with the expected flux-linkage
 * vector at the aligned angle (th = PI/2) so it starts with a good estimate.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void AlignExit(MOTOR_t *motor_ptr)
{

    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = {PI_OVER_TWO};

#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Reset(motor_ptr);
    if (params_ptr->sys.fb.mode == AqB_Enc)
    {
        INC_ENCODER_Reset(motor_ptr, th0);
    }
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Reset(motor_ptr);
    TRQ_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);
#endif
    if (params_ptr->sys.fb.mode == Sensorless)
    {
        AB_t la_ab_lead = (AB_t){(params_ptr->motor.lam + params_ptr->motor.ld * params_ptr->ctrl.align.voltage / params_ptr->motor.r), 0.0f};
        OBS_Reset(motor_ptr, &la_ab_lead, &w0, &th0);
    }
}

static void SixPulseEntry(MOTOR_t *motor_ptr)
{
    SIX_PULSE_INJ_Reset(motor_ptr);
}

/**
 * @brief ISR0 handler for the Six_Pulse state (RFO/SFO)
 *
 * Delegates to the six-pulse injection fast ISR which applies voltage pulses
 * on each axis and records the resulting current-peak response.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void SixPulseISR0(MOTOR_t *motor_ptr)
{
    SIX_PULSE_INJ_RunISR0(motor_ptr);
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Six_Pulse state (RFO/SFO)
 *
 * Runs the slow-rate six-pulse injection tasks: evaluates the current-peak
 * results for each pulse axis and advances the injection sequence state machine.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void SixPulseISR1(MOTOR_t *motor_ptr)
{
    SIX_PULSE_INJ_RunISR1(motor_ptr);
}

/**
 * @brief Exit function for the Six_Pulse state (RFO/SFO)
 *
 * Resets the current (RFO) or flux/torque/delta (SFO) controllers and
 * initialises the BEMF observer with the flux-linkage vector at the
 * detected rotor angle so the subsequent sensorless state starts with
 * a valid angle and flux estimate.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void SixPulseExit(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Reset(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Reset(motor_ptr);
    TRQ_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);
#endif

    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = ctrl_ptr->six_pulse_inj.th_r_est;
    PARK_t park_r;
    ParkInit(th0.elec, &park_r);

    QD_t la_qd_r = (QD_t){0.0f, params_ptr->motor.lam};
    AB_t la_ab_lead;
    ParkTransformInv(&la_qd_r, &park_r, &la_ab_lead);

    OBS_Reset(motor_ptr, &la_ab_lead, &w0, &th0);
}

/**
 * @brief Entry function for the High_Freq state (RFO/SFO)
 *
 * Initialises the HFI controller at the six-pulse estimated rotor angle,
 * arms the HFI lock-time timer, sets high_freq.used = true, clears the
 * alpha-beta voltage command, and enables runtime fault protection.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void HighFreqEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    HIGH_FREQ_INJ_Reset(motor_ptr, Elec_Zero, ctrl_ptr->six_pulse_inj.th_r_est);
    StopWatchInit(&sm_ptr->vars.high_freq.timer, params_ptr->ctrl.high_freq_inj.lock_time, params_ptr->sys.samp.ts1);
    sm_ptr->vars.high_freq.used = true;
    vars_ptr->v_ab_cmd = AB_Zero;
    SM_RuntimeFaultEnDis(motor_ptr, En);
}

/**
 * @brief ISR0 handler for the High_Freq state (RFO/SFO)
 *
 * Runs the blended HFI + observer feedback (via SensorlessFeedbackISR0),
 * routes the observer output to w_final / th_r_final, applies the HFI
 * excitation vector as the full alpha-beta voltage command, and runs
 * the voltage modulator.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void HighFreqISR0(MOTOR_t *motor_ptr)
{

    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    vars_ptr->w_final.elec = vars_ptr->w_est.elec;
    vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
    vars_ptr->v_ab_cmd_tot = ctrl_ptr->high_freq_inj.v_ab_cmd;
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END

static void HighFreqISR1(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    StopWatchRun(&sm_ptr->vars.high_freq.timer);
    CTRL_FILTS_RunSpeedISR1(motor_ptr);
}
static void HighFreqExit(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
#if defined(CTRL_METHOD_SFO)
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_Init(motor_ptr, params_ptr->ctrl.high_freq_inj.bw_red_coeff);
    CURRENT_CTRL_Reset(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    FLUX_CTRL_Init(motor_ptr, params_ptr->ctrl.high_freq_inj.bw_red_coeff);
    ctrl_ptr->delta.bw_red_coeff = params_ptr->ctrl.high_freq_inj.bw_red_coeff;
#endif
}

/**
 * @brief Entry function for the Speed_OL_To_CL transition state (RFO/SFO)
 *
 * Resets the BEMF observer at the current commanded speed and direction,
 * resets the torque observer, arms the observer lock-time timer, and
 * enables runtime fault protection. Allows the observer to converge before
 * the state machine transfers to full closed-loop speed control.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void SpeedOLToCLEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    AB_t la_ab_lead = AB_Zero;
    ELEC_t w0 = (ELEC_t){params_ptr->obs.w_thresh.elec * vars_ptr->dir};
    ELEC_t th0 = Elec_Zero;
    OBS_Reset(motor_ptr, &la_ab_lead, &w0, &th0);
    TRQ_Reset(motor_ptr);
    StopWatchInit(&sm_ptr->vars.speed_ol_to_cl.timer, params_ptr->obs.lock_time, params_ptr->sys.samp.ts1);
    SM_RuntimeFaultEnDis(motor_ptr, En);
}

/**
 * @brief ISR0 handler for the Speed_OL_To_CL transition state (RFO/SFO)
 *
 * Runs the feedback observer and routes estimates to w_final / th_r_final.
 * Depending on the startup mode:
 * - Volt_OL startup: Park-transforms current feedback at the observer angle
 *   and runs the V/Hz voltage controller to maintain flux during transition.
 * - Curr_OL startup (RFO): advances the open-loop angle integrator and runs
 *   the current controller.
 * Always runs the torque observer and voltage modulator.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void SpeedOLToCLISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
#if defined(CTRL_METHOD_RFO)
    bool prev_state_volt_ol = Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup);
    bool prev_state_curr_ol = Mode(params_ptr, Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr, Profiler_Mode);
#elif defined(CTRL_METHOD_SFO)
    bool prev_state_volt_ol = Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr, Profiler_Mode);
    // bool prev_state_curr_ol = false;
#endif

    if (prev_state_volt_ol)
    {
        ELEC_t th_r_trq = {vars_ptr->th_r_est.elec};
        PARK_t park_r_trq;
        ParkInit(th_r_trq.elec, &park_r_trq);
        ParkTransform(&vars_ptr->i_ab_fb, &park_r_trq, &vars_ptr->i_qd_r_fb);

        VOLT_CTRL_RunISR0(motor_ptr);
    }
#if defined(CTRL_METHOD_RFO)
    else if (prev_state_curr_ol)
    {
        vars_ptr->th_r_cmd.elec = Wrap2Pi(vars_ptr->th_r_cmd.elec + vars_ptr->w_cmd_int.elec * params_ptr->sys.samp.ts0);
        vars_ptr->th_r_final.elec = vars_ptr->th_r_cmd.elec;
        vars_ptr->w_final.elec = vars_ptr->w_cmd_int.elec;

        CURRENT_CTRL_RunISR0(motor_ptr);
    }
#endif
    TRQ_RunObsISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Speed_OL_To_CL transition state (RFO/SFO)
 *
 * Rate-limits the speed command reference. Depending on startup mode
 * continues calculating the V/Hz voltage law (Volt_OL) or the open-loop
 * current reference (Curr_OL / RFO). Runs all slow-rate control filters
 * and advances the observer lock-time timer.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void SpeedOLToCLISR1(MOTOR_t *motor_ptr)
{
#if defined(CTRL_METHOD_RFO)
    PROTECT_t *protect_ptr = motor_ptr->protect_ptr;
#endif
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    CTRL_UpdateWcmdIntISR1(motor_ptr, vars_ptr->w_cmd_ext, params_ptr->sys.rate_lim.w_cmd.elec);

#if defined(CTRL_METHOD_RFO)
    bool prev_state_volt_ol = Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup);
    bool prev_state_curr_ol = Mode(params_ptr, Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr, Profiler_Mode);
#elif defined(CTRL_METHOD_SFO)
    bool prev_state_volt_ol = Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr, Profiler_Mode);
    // bool prev_state_curr_ol = false;
#endif

    if (prev_state_volt_ol)
    {
        vars_ptr->v_qd_r_cmd.q = MAX(params_ptr->ctrl.volt.v_min + ABS(vars_ptr->w_cmd_int.elec) * params_ptr->ctrl.volt.v_to_f_ratio, 0.0f) * vars_ptr->dir;
    }
#if defined(CTRL_METHOD_RFO)
    else if (prev_state_curr_ol)
    {
        vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_ext);
        vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
        vars_ptr->i_qd_r_ref = (QD_t){vars_ptr->i_cmd_int, 0.0f}; // No phase advance in open loop
        vars_ptr->i_qd_r_cmd = vars_ptr->i_qd_r_ref;              // No flux weakening in current control mode
    }
#endif
    CTRL_FILTS_RunAllISR1(motor_ptr);
    StopWatchRun(&sm_ptr->vars.speed_ol_to_cl.timer);
}

/**
 * @brief Exit function for the Speed_OL_To_CL transition state (RFO/SFO)
 *
 * Performs bumpless transfer into closed-loop speed control:
 * - Computes the optimal current (RFO) or torque (SFO) from current feedback
 *   at the present observer angle, scales by ol_cl_tr_coeff, and
 *   back-calculates the speed integrator to eliminate a torque/current step.
 * - Resets flux weakening (RFO) or flux/delta controllers (SFO).
 * - Manages feed-forward and controller state for the specific startup path.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void SpeedOLToCLExit(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
#if defined(CTRL_METHOD_RFO)
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
    SPEED_CTRL_Reset(motor_ptr);

    PARK_t park_r;
    QD_t i_qd_r_fb;

    hw_fcn.EnterCriticalSection(); // --------------------
    // atomic operations needed for struct copies and all data must have the same time stamp
    AB_t i_ab_fb = vars_ptr->i_ab_fb;
    float th_r_est = vars_ptr->th_r_est.elec;
    hw_fcn.ExitCriticalSection(); // --------------------

    ParkInit(th_r_est, &park_r);
    ParkTransform(&i_ab_fb, &park_r, &i_qd_r_fb);

#if defined(CTRL_METHOD_RFO)
    float i_cmd_spd;
    PHASE_ADV_CalcOptIs(motor_ptr, &i_qd_r_fb, &i_cmd_spd);
    vars_ptr->i_cmd_int = i_cmd_spd * params_ptr->ctrl.speed.ol_cl_tr_coeff;
    SPEED_CTRL_IntegBackCalc(motor_ptr, vars_ptr->i_cmd_int);

#if defined(PC_TEST)
    vars_ptr->test[30] = i_qd_r_fb.q;
    vars_ptr->test[31] = i_qd_r_fb.d;
    vars_ptr->test[32] = i_cmd_spd;
#endif

    FLUX_WEAKEN_Reset(motor_ptr);
    if (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup))
    {
        CURRENT_CTRL_Reset(motor_ptr);
    }
    else if (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr, Profiler_Mode))
    {
        ctrl_ptr->curr.en_ff = true;
    }

#elif defined(CTRL_METHOD_SFO)
    float T_cmd_spd;
    TRQ_CalcTrq(motor_ptr, &i_qd_r_fb, &T_cmd_spd);
    vars_ptr->T_cmd_int = T_cmd_spd * params_ptr->ctrl.speed.ol_cl_tr_coeff;
    SPEED_CTRL_IntegBackCalc(motor_ptr, vars_ptr->T_cmd_int);

    FLUX_CTRL_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);

#endif
}

/**
 * @brief Entry function for the Dyno_Lock state (RFO/SFO)
 *
 * Atomically places the inverter in high-Z, zeroes PWM duties, resets the
 * BEMF observer at zero speed, resets all control filters, and arms the
 * dyno-lock timer. Switches the voltage observation pointer to v_ab_fb so
 * the observer tracks back-EMF during the high-Z coasting window.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void DynoLockEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    StopWatchInit(&sm_ptr->vars.dyno_lock.timer, params_ptr->sys.dyno_lock_time, params_ptr->sys.samp.ts1);

    hw_fcn.EnterCriticalSection(); // --------------------
    // atomic operations needed for struct writes and no more modification until next state

    hw_fcn.GateDriverEnterHighZ(motor_ptr->motor_instance);
    vars_ptr->d_uvw_cmd = UVW_Zero;
    vars_ptr->d_uvw_cmd_fall = vars_ptr->d_uvw_cmd;
    vars_ptr->v_ab_obs = &vars_ptr->v_ab_fb;
    AB_t la_ab_lead = AB_Zero;
    ELEC_t w0 = Elec_Zero;
    ELEC_t th0 = Elec_Zero;
    OBS_Reset(motor_ptr, &la_ab_lead, &w0, &th0);
    CTRL_FILTS_Reset(motor_ptr);

    sm_ptr->current = sm_ptr->next; // must be in critical section for dyno_lock entry

    hw_fcn.ExitCriticalSection(); // --------------------
}

/**
 * @brief ISR0 handler for the Dyno_Lock state (RFO/SFO)
 *
 * Runs the BEMF observer and routes its output to w_final / th_r_final.
 * The inverter remains in high-Z throughout this state; no PWM is applied.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void DynoLockISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);

    switch (params_ptr->sys.fb.mode)
    {
    default:
    case Sensorless:
        vars_ptr->w_final.elec = vars_ptr->w_est.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
        break;
#if defined(CTRL_METHOD_RFO)
    case Hall:
        vars_ptr->w_final.elec = vars_ptr->w_hall.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
        break;
#endif
    }
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Dyno_Lock state (RFO/SFO)
 *
 * Advances the dyno-lock timer and runs the speed filter so a stable
 * speed estimate is available at the end of the lock window.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void DynoLockISR1(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    StopWatchRun(&sm_ptr->vars.dyno_lock.timer);
    CTRL_FILTS_RunSpeedISR1(motor_ptr);
}

/**
 * @brief Exit function for the Dyno_Lock state (RFO/SFO)
 *
 * Prepares the controllers for bumpless transfer to torque/current control:
 * - Resets the torque observer.
 * - RFO: resets the current controller and zeroes i_cmd_int.
 * - SFO: resets the flux and delta controllers; back-calculates the delta-PI
 *   integrator to lam*w so the first modulated voltage is continuous.
 * Atomically copies v_ab_fb to v_ab_cmd and re-enables the gate drivers.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void DynoLockExit(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
#if defined(CTRL_METHOD_SFO)
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
#endif
#if defined(CTRL_METHOD_SFO)
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
#endif

    TRQ_Reset(motor_ptr);
#if defined(CTRL_METHOD_RFO)
    //	Ideal case in RFO:
    //		1) i_qd_r_cmd == i_qd_r_fb == {0.0f,0.0f}
    //		2) v_qd_r_cmd == v_qd_r_fb == {lam*w, 0.0f}
    //		3) v_ab_cmd == v_ab_fb == ParkInv(v_qd_r_fb, th_r)
    //		4) curr.pi_q.error == curr.pi_d.error == curr.pi_q.output == curr.pi_d.output == 0.0f (when curr.ff_coef
    //== 1.0f)
    vars_ptr->i_cmd_int = 0.0f;
    CURRENT_CTRL_Reset(motor_ptr);
#elif defined(CTRL_METHOD_SFO)
    //	Ideal case in SFO:
    //		1) i_qd_s_cmd == i_qd_s_fb == {0.0f,0.0f}
    //		2) delta == 0.0f, th_s = th_r
    //		3) v_qd_s_cmd == v_qd_s_fb == {lam*w, 0.0f}
    //		4) v_ab_cmd == v_ab_fb == ParkInv(v_qd_s_fb, th_s)
    //		5) trq.pi.error == trq.pi.output == flux.pi.error == flux.pi.output == 0.0f
    //		6) delta.pi.output = v_qd_s_cmd.q = lam*w
    vars_ptr->T_cmd_int = 0.0f;
    FLUX_CTRL_Reset(motor_ptr);
    DELTA_CTRL_Reset(motor_ptr);
    ctrl_ptr->delta.pi.output = params_ptr->motor.lam * vars_ptr->w_final_filt.elec;
    PI_IntegBackCalc(&ctrl_ptr->delta.pi, ctrl_ptr->delta.pi.output, 0.0f, 0.0f);
#endif
    hw_fcn.EnterCriticalSection();          // --------------------
    vars_ptr->v_ab_cmd = vars_ptr->v_ab_fb; // atomic operations needed
    vars_ptr->v_ab_obs = &vars_ptr->v_ab_cmd;
    hw_fcn.GateDriverExitHighZ(motor_ptr->motor_instance);
    hw_fcn.ExitCriticalSection(); // --------------------
}

/**
 * @brief Entry function for all motor-profiling states (Prof_Rot_Lock..Prof_Finished)
 *
 * Calls PROFILER_Entry inside a critical section and atomically updates the
 * current state ID so ISR0 sees the new state before the next execution.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void MotorProfEntry(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    hw_fcn.EnterCriticalSection(); // --------------------
    PROFILER_Entry(motor_ptr);
    sm_ptr->current = sm_ptr->next; // must be in critical section
    hw_fcn.ExitCriticalSection();   // --------------------
}

/**
 * @brief ISR0 handler for all motor-profiling states
 *
 * Executes one profiling measurement step via PROFILER_RunISR0,
 * then runs the voltage modulator to apply the profiling voltage vector.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void MotorProfISR0(MOTOR_t *motor_ptr)
{
    PROFILER_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}

/**
 * @brief Exit function for all motor-profiling states
 *
 * Calls PROFILER_Exit to finalise and store the results of the completed
 * profiling step before the state machine advances to the next state.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void MotorProfExit(MOTOR_t *motor_ptr)
{
    PROFILER_Exit(motor_ptr);
}

#endif

#if defined(CTRL_METHOD_SFO)

/**
 * @brief Entry function for the Torque_CL (closed-loop torque) state (SFO)
 *
 * Seeds the torque command integrator at the configured torque threshold
 * in the current demanded direction and enables runtime fault protection.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void TorqueCLEntry(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    vars_ptr->T_cmd_int = params_ptr->ctrl.trq.T_cmd_thresh * vars_ptr->dir;
    SM_RuntimeFaultEnDis(motor_ptr, En);
}

/**
 * @brief ISR0 handler for the Torque_CL state (SFO)
 *
 * Full closed-loop torque control fast path:
 * 1. Runs the sensorless observer and routes estimates to w_final / th_r_final
 * 2. Flux controller -> torque observer -> torque controller -> delta controller
 * 3. Inverse Park transform to alpha-beta voltage command
 * 4. Superimposes HFI excitation voltage if high-frequency injection is active
 * 5. Voltage modulator
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void TorqueCLISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    vars_ptr->w_final.elec = vars_ptr->w_est.elec;
    vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
    FLUX_CTRL_RunISR0(motor_ptr);
    TRQ_RunObsISR0(motor_ptr);
    TRQ_RunCtrlISR0(motor_ptr);
    DELTA_CTRL_RunISR0(motor_ptr);
    ParkTransformInv(&vars_ptr->v_qd_s_cmd, &vars_ptr->park_s, &vars_ptr->v_ab_cmd);
    vars_ptr->v_ab_cmd_tot = vars_ptr->v_ab_cmd;
    AddHighFreqVoltsIfNeeded(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Torque_CL state (SFO)
 *
 * Slow-rate tasks: runs the speed filter and torque-limit protection,
 * applies the protection limit to the external torque command, rate-limits
 * the torque reference, and updates the MTPA flux setpoint from the LUT.
 * Flux weakening is not applied in direct torque control mode.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void TorqueCLISR1(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    PROTECT_t *protect_ptr = motor_ptr->protect_ptr;

    CTRL_FILTS_RunSpeedISR1(motor_ptr);
    FAULT_PROTECT_RunTrqLimitCtrlISR1(motor_ptr);
    vars_ptr->T_cmd_prot = SAT(-protect_ptr->motor.T_lmt, protect_ptr->motor.T_lmt, vars_ptr->T_cmd_ext);
    vars_ptr->T_cmd_int = RateLimit(params_ptr->sys.rate_lim.T_cmd * params_ptr->sys.samp.ts1, vars_ptr->T_cmd_prot, vars_ptr->T_cmd_int);
    vars_ptr->la_cmd_mtpa = LUT1DInterp(&params_ptr->motor.mtpa_lut, ABS(vars_ptr->T_cmd_int));
    vars_ptr->la_cmd_final = vars_ptr->la_cmd_mtpa; // No flux weakening in torque control mode
}
#elif defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)

/**
 * @brief Entry function for the Current_CL (closed-loop current) state (RFO/TBC)
 *
 * Seeds the current command integrator at the configured current threshold
 * in the current demanded direction and enables runtime fault protection.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void CurrentCLEntry(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    vars_ptr->i_cmd_int = params_ptr->ctrl.curr.i_cmd_thresh * vars_ptr->dir;
    SM_RuntimeFaultEnDis(motor_ptr, En);
}

/**
 * @brief ISR0 handler for the Current_CL state (RFO/TBC)
 *
 * Full closed-loop current control fast path:
 * 1. Runs the configured feedback source and routes output to w_final / th_r_final
 * 2. RFO: current controller then voltage modulator
 * 3. TBC block commutation: current controller then block-commutation modulator
 * 3. TBC trapezoidal commutation: trapezoidal commutation controller
 * 4. Torque observer
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void CurrentCLISR0(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;

    FeedbackISR0Wrap[motor_ptr->motor_instance](motor_ptr);
    switch (params_ptr->sys.fb.mode)
    {
    default:
    case Sensorless:
        vars_ptr->w_final.elec = vars_ptr->w_est.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_est.elec;
        break;
    case Hall:
        vars_ptr->w_final.elec = vars_ptr->w_hall.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_hall.elec;
        break;
    case AqB_Enc:
        vars_ptr->w_final.elec = vars_ptr->w_enc.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_enc.elec;
        break;
    case Direct:
        vars_ptr->w_final.elec = vars_ptr->w_fb.elec;
        vars_ptr->th_r_final.elec = vars_ptr->th_r_fb.elec;
        break;
    }
#if defined(CTRL_METHOD_RFO)
    CURRENT_CTRL_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
#elif defined(CTRL_METHOD_TBC)
    switch (params_ptr->ctrl.tbc.mode)
    {
    case Block_Commutation:
    default:
        CURRENT_CTRL_RunISR0(motor_ptr);
        BLOCK_COMM_RunVoltModISR0(motor_ptr);
        break;
    case Trapezoidal_Commutation:
        TRAP_COMM_RunISR0(motor_ptr);
        break;
    }
#endif
    TRQ_RunObsISR0(motor_ptr);
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Current_CL state (RFO/TBC)
 *
 * Slow-rate tasks: runs the speed filter, applies I2T current-limit
 * protection, and rate-limits the current reference. In RFO mode also
 * runs phase advance; flux weakening is bypassed in current control mode.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void CurrentCLISR1(MOTOR_t *motor_ptr)
{
    PROTECT_t *protect_ptr = motor_ptr->protect_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    CTRL_FILTS_RunSpeedISR1(motor_ptr);
    vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_ext);
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
#if defined(CTRL_METHOD_RFO)
    PHASE_ADV_RunISR1(motor_ptr);
    vars_ptr->i_qd_r_cmd = vars_ptr->i_qd_r_ref; // No flux weakening in current control mode
#endif
}
#endif

#if defined(CTRL_METHOD_RFO)

/**
 * @brief Entry function for the Current_OL (open-loop current) state (RFO)
 *
 * Initialises open-loop current control:
 * - Resets the speed integrator to the V/Hz threshold.
 * - Loads the open-loop current reference from params, resets the current
 *   controller, and disables the feed-forward path.
 * - In Profiler_Mode registers the profiler as the ISR0 add-on callback.
 * - Atomically updates the state ID and enables runtime fault protection.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void CurrentOLEntry(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;

    CTRL_ResetWcmdInt(motor_ptr, params_ptr->ctrl.volt.w_thresh);
    vars_ptr->th_r_cmd.elec = 0.0f;

    vars_ptr->i_cmd_ext = params_ptr->ctrl.curr.i_cmd_ol;
    vars_ptr->i_cmd_int = 0.0f;
    CURRENT_CTRL_Reset(motor_ptr);
    ctrl_ptr->curr.en_ff = false;
    hw_fcn.EnterCriticalSection(); // --------------------
    if (Mode(params_ptr, Profiler_Mode))
    {
        sm_ptr->add_callback.RunISR0 = PROFILER_RunISR0;
    }
    if (params_ptr->sys.rate_lim.w_ol_cmd.elec == 0) /* to maintain backward compataility */
    {
        params_ptr->sys.rate_lim.w_ol_cmd.elec = params_ptr->sys.rate_lim.w_cmd.elec;
    }
    sm_ptr->current = sm_ptr->next; // must be in critical section
    SM_RuntimeFaultEnDis(motor_ptr, En);
    hw_fcn.ExitCriticalSection(); // --------------------
}

/**
 * @brief Exit function for the Current_OL state (RFO)
 *
 * Re-enables the current-controller feed-forward path when leaving
 * Curr_Mode_Open_Loop so subsequent closed-loop states benefit from FF.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void CurrentOLExit(MOTOR_t *motor_ptr)
{
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;

    if (Mode(params_ptr, Curr_Mode_Open_Loop))
    {
        ctrl_ptr->curr.en_ff = true;
    }
}

/**
 * @brief ISR0 handler for the Current_OL state (RFO)
 *
 * Advances the open-loop angle integrator (th_r_cmd) at the commanded
 * speed, presents the open-loop angle and speed as w_final / th_r_final,
 * runs the current controller, and runs the voltage modulator.
 *
 * @param motor_ptr Pointer to motor structure
 */
RAMFUNC_BEGIN
static void CurrentOLISR0(MOTOR_t *motor_ptr)
{
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    vars_ptr->th_r_cmd.elec = Wrap2Pi(vars_ptr->th_r_cmd.elec + vars_ptr->w_cmd_int.elec * params_ptr->sys.samp.ts0);
    vars_ptr->th_r_final.elec = vars_ptr->th_r_cmd.elec;
    vars_ptr->w_final.elec = vars_ptr->w_cmd_int.elec;

    CURRENT_CTRL_RunISR0(motor_ptr);
    VOLT_MOD_RunISR0(motor_ptr);
}
RAMFUNC_END

/**
 * @brief ISR1 handler for the Current_OL state (RFO)
 *
 * Slow-rate tasks: rate-limits the speed reference using the dedicated OL
 * rate limit, runs the speed filter, applies I2T protection, rate-limits
 * the current reference, and computes the open-loop QD current command
 * (Q = i_cmd_int, D = 0; no phase advance or flux weakening in OL mode).
 *
 * @param motor_ptr Pointer to motor structure
 */
static void CurrentOLISR1(MOTOR_t *motor_ptr)
{
    PROTECT_t *protect_ptr = motor_ptr->protect_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;

    CTRL_UpdateWcmdIntISR1(motor_ptr, vars_ptr->w_cmd_ext, params_ptr->sys.rate_lim.w_ol_cmd.elec);
    CTRL_FILTS_RunSpeedISR1(motor_ptr);
    vars_ptr->i_cmd_prot = SAT(-protect_ptr->motor.i2t.i_limit, protect_ptr->motor.i2t.i_limit, vars_ptr->i_cmd_ext);
    vars_ptr->i_cmd_int = RateLimit(params_ptr->sys.rate_lim.i_cmd * params_ptr->sys.samp.ts1, vars_ptr->i_cmd_prot, vars_ptr->i_cmd_int);
    vars_ptr->i_qd_r_ref = (QD_t){vars_ptr->i_cmd_int, 0.0f}; // No phase advance in open loop
    vars_ptr->i_qd_r_cmd = vars_ptr->i_qd_r_ref;              // No flux weakening in current control mode
}
#endif

/**
 * @brief Evaluate all state-transition conditions and update sm_ptr->next (ISR1)
 *
 * Called every ISR1 cycle from STATE_MACHINE_RunISR1. Based on the current
 * state, control mode, command threshold comparisons, timer expirations,
 * observer/profiler states, and fault triggers, determines whether a state
 * transition is needed. Sets sm_ptr->next to the target state if a transition
 * is warranted; leaves it equal to sm_ptr->current otherwise.
 *
 * @param motor_ptr Pointer to motor structure
 */
static void ConditionCheck(MOTOR_t *motor_ptr)
{
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    CTRL_VARS_t *vars_ptr = motor_ptr->vars_ptr;
    PARAMS_t *params_ptr = motor_ptr->params_ptr;
    FAULTS_t *faults_ptr = motor_ptr->faults_ptr;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    PROFILER_t *profiler_ptr = motor_ptr->profiler_ptr;
    CTRL_t *ctrl_ptr = motor_ptr->ctrl_ptr;
#endif
    STATE_ID_t current = sm_ptr->current;
    STATE_ID_t next = current;

    bool fault_trigger = (faults_ptr->reaction != No_Reaction);
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO) || defined(CTRL_METHOD_TBC)
    bool no_speed_reset_required = !sm_ptr->vars.speed_reset_required;
#endif
    float w_cmd_ext_abs = ABS(vars_ptr->w_cmd_ext.elec);
    float w_cmd_int_abs = ABS(vars_ptr->w_cmd_int.elec);
    float w_thresh_above_low = params_ptr->ctrl.volt.w_thresh.elec;
    float w_thresh_below_low = params_ptr->ctrl.volt.w_thresh.elec - params_ptr->ctrl.volt.w_hyst.elec;
    float w_thresh_above_high = params_ptr->obs.w_thresh.elec;
    float w_thresh_below_high = params_ptr->obs.w_thresh.elec - params_ptr->obs.w_hyst.elec;

    bool w_cmd_ext_above_thresh_low = w_cmd_ext_abs > w_thresh_above_low;
    bool w_cmd_ext_below_thresh_low = w_cmd_ext_abs < w_thresh_below_low;
    bool w_cmd_int_above_thresh_low = w_cmd_int_abs > w_thresh_above_low;
    bool w_cmd_int_below_thresh_low = w_cmd_int_abs < w_thresh_below_low;

    bool w_cmd_ext_above_thresh_high = w_cmd_ext_abs > w_thresh_above_high;
    bool w_cmd_ext_below_thresh_high = w_cmd_ext_abs < w_thresh_below_high;
    bool w_cmd_int_above_thresh_high = w_cmd_int_abs > w_thresh_above_high;
    bool w_cmd_int_below_thresh_high = w_cmd_int_abs < w_thresh_below_high;

    (void)w_cmd_ext_above_thresh_low;
    (void)w_cmd_ext_below_thresh_low;
    (void)w_cmd_int_above_thresh_low;
    (void)w_cmd_int_below_thresh_low;
    (void)w_cmd_ext_above_thresh_high;
    (void)w_cmd_ext_below_thresh_high;
    (void)w_cmd_int_above_thresh_high;
    (void)w_cmd_int_below_thresh_high;

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    float i_cmd_ext_abs = ABS(vars_ptr->i_cmd_ext);
    float i_cmd_int_abs = ABS(vars_ptr->i_cmd_int);
    float i_thresh_above = params_ptr->ctrl.curr.i_cmd_thresh;
    float i_thresh_below = params_ptr->ctrl.curr.i_cmd_thresh - params_ptr->ctrl.curr.i_cmd_hyst;

    bool i_cmd_ext_above_thresh = i_cmd_ext_abs > i_thresh_above;
    bool i_cmd_ext_below_thresh = i_cmd_ext_abs < i_thresh_below;
    bool i_cmd_int_above_thresh = i_cmd_int_abs > i_thresh_above;
    bool i_cmd_int_below_thresh = i_cmd_int_abs < i_thresh_below;

    (void)i_cmd_ext_above_thresh;
    (void)i_cmd_ext_below_thresh;
    (void)i_cmd_int_above_thresh;
    (void)i_cmd_int_below_thresh;

#elif defined(CTRL_METHOD_SFO)
    float T_cmd_ext_abs = ABS(vars_ptr->T_cmd_ext);
    float T_cmd_int_abs = ABS(vars_ptr->T_cmd_int);
    float T_thresh_above = params_ptr->ctrl.trq.T_cmd_thresh;
    float T_thresh_below = params_ptr->ctrl.trq.T_cmd_thresh - params_ptr->ctrl.trq.T_cmd_hyst;

    bool T_cmd_ext_above_thresh = T_cmd_ext_abs > T_thresh_above;
    bool T_cmd_ext_below_thresh = T_cmd_ext_abs < T_thresh_below;
    bool T_cmd_int_above_thresh = T_cmd_int_abs > T_thresh_above;
    bool T_cmd_int_below_thresh = T_cmd_int_abs < T_thresh_below;

    (void)T_cmd_ext_above_thresh;
    (void)T_cmd_ext_below_thresh;
    (void)T_cmd_int_above_thresh;
    (void)T_cmd_int_below_thresh;
#endif

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    float cmd_thresh_above = params_ptr->profiler.cmd_thresh;
    float cmd_thresh_below = params_ptr->profiler.cmd_thresh - params_ptr->profiler.cmd_hyst;

    bool cmd_above_thresh = vars_ptr->cmd_final > cmd_thresh_above;
    bool cmd_below_thresh = vars_ptr->cmd_final < cmd_thresh_below;

    (void)cmd_above_thresh;
    (void)cmd_below_thresh;
#endif

    switch (current)
    {
    default:
    case Init:
        if (fault_trigger)
        {
            next = Fault;
        }
        else if (sm_ptr->vars.init.param_init_done && sm_ptr->vars.init.offset_null_done && vars_ptr->en)
        {
#if defined(CTRL_METHOD_RFO)
            if (Mode(params_ptr, Curr_Mode_FOC_Sensorless_Dyno))
            {
                next = Dyno_Lock;
            }
            else
            {
                next = Brake_Boot;
            }
#elif defined(CTRL_METHOD_SFO)
            if (Mode(params_ptr, Trq_Mode_FOC_Sensorless_Dyno))
            {
                next = Dyno_Lock;
            }
            else
            {
                next = Brake_Boot;
            }
#elif defined(CTRL_METHOD_TBC)
            next = Brake_Boot;
#endif
        }
        break;
    case Brake_Boot:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.brake_boot.timer))
        {
#if defined(CTRL_METHOD_RFO)
            if (cmd_above_thresh && Mode(params_ptr, Profiler_Mode))
            {
                next = Prof_Rot_Lock;
            }
            else if (w_cmd_ext_above_thresh_low && params_ptr->sys.catch_spin.time && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup)||Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup)|| Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup)|| Mode(params_ptr, Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr, Speed_Mode_FOC_Hall)))
            {
                next = Catch_Spin;
            }
            else if (w_cmd_ext_above_thresh_low && (Mode(params_ptr, Volt_Mode_Open_Loop) || (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup) && no_speed_reset_required)))
            {
                next = Volt_OL;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Encoder_Align_Startup))) ||
                (i_cmd_ext_above_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Curr_Mode_FOC_Encoder_Align_Startup))))
            {
                next = Align;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (i_cmd_ext_above_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Six_Pulse;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr, Curr_Mode_FOC_Hall))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(params_ptr, Speed_Mode_FOC_Hall))
            {
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && (Mode(params_ptr, Curr_Mode_Open_Loop) || (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Curr_Startup) && no_speed_reset_required)))
            {
                next = Current_OL;
            }
            else if (Mode(params_ptr, Position_Mode_FOC_Encoder_Align_Startup))
            {
                if (params_ptr->ctrl.align.time == 0)
                {
                    sm_ptr->vars.speed_reset_required = true;
                    next = Position_CL;
                }
                else
                {
                    next = Align;
                }
            }
#elif defined(CTRL_METHOD_SFO)
            if (cmd_above_thresh && Mode(params_ptr, Profiler_Mode))
            {
                next = Prof_Rot_Lock;
            }
            else if (w_cmd_ext_above_thresh_low && (Mode(params_ptr, Volt_Mode_Open_Loop) || (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup) && no_speed_reset_required)))
            {
                next = Volt_OL;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup)) || (T_cmd_ext_above_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_Align_Startup)))
            {
                next = Align;
            }
            else if ((w_cmd_ext_above_thresh_low && no_speed_reset_required && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (T_cmd_ext_above_thresh && (Mode(params_ptr,Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Six_Pulse;
            }
#elif defined(CTRL_METHOD_TBC)
            if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Volt_Mode_Open_Loop))
            {
                next = Volt_OL;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr, Curr_Mode_Block_Comm_Hall))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && no_speed_reset_required && Mode(params_ptr, Speed_Mode_Block_Comm_Hall))
            {
                next = Speed_CL;
            }
#endif
        }
        break;
    case Volt_OL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
#if defined(CTRL_METHOD_RFO)
        else if (w_cmd_int_below_thresh_low && (Mode(params_ptr, Volt_Mode_Open_Loop) || Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup)))
        {
            next = Brake_Boot;
        }
        else if (w_cmd_int_above_thresh_high && Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup))
        {
            sm_ptr->vars.speed_reset_required = true;
            next = Speed_OL_To_CL;
        }
#elif defined(CTRL_METHOD_SFO)
        else if ((w_cmd_int_below_thresh_low && (Mode(params_ptr,Volt_Mode_Open_Loop) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup)))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (w_cmd_int_above_thresh_high && (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr, Profiler_Mode)))
        {
            sm_ptr->vars.speed_reset_required = true;
            next = Speed_OL_To_CL;
        }
#elif defined(CTRL_METHOD_TBC)
        else if (w_cmd_int_below_thresh_low && Mode(params_ptr, Volt_Mode_Open_Loop))
        {
            next = Brake_Boot;
        }
#endif
        break;

#if defined(CTRL_METHOD_RFO)
    case Current_OL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((w_cmd_int_below_thresh_low && (Mode(params_ptr,Curr_Mode_Open_Loop) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup)))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (w_cmd_int_above_thresh_high && (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Curr_Startup) || Mode(params_ptr, Profiler_Mode)))
        {
            sm_ptr->vars.speed_reset_required = true;
            next = Speed_OL_To_CL;
        }
        break;
    case Align:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.align.timer))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr, Speed_Mode_FOC_Encoder_Align_Startup))) ||
                (i_cmd_ext_below_thresh && (Mode(params_ptr, Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr, Curr_Mode_FOC_Encoder_Align_Startup))))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && (Mode(params_ptr, Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr, Curr_Mode_FOC_Encoder_Align_Startup)))
            {
                next = Current_CL;
            }
            else if ((w_cmd_ext_above_thresh_high && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup)) || (w_cmd_ext_above_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Encoder_Align_Startup)))
            {
                sm_ptr->vars.speed_reset_required = (Mode(params_ptr, Speed_Mode_FOC_Sensorless_Align_Startup) == true);
                next = Speed_CL;
            }
            else if (Mode(params_ptr, Position_Mode_FOC_Encoder_Align_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Position_CL;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Align:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.align.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup)) || (T_cmd_ext_below_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_Align_Startup)))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(params_ptr, Trq_Mode_FOC_Sensorless_Align_Startup))
            {
                next = Torque_CL;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(params_ptr, Speed_Mode_FOC_Sensorless_Align_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Six_Pulse:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((ctrl_ptr->six_pulse_inj.state == Finished_Success) || (ctrl_ptr->six_pulse_inj.state == Finished_Ambiguous))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(params_ptr, Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr, Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (i_cmd_ext_below_thresh && (Mode(params_ptr, Curr_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr, Curr_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr, Curr_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                next = Current_CL;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr, Curr_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(params_ptr, Speed_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Six_Pulse:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((ctrl_ptr->six_pulse_inj.state == Finished_Success) || (ctrl_ptr->six_pulse_inj.state == Finished_Ambiguous))
        {
            if ((w_cmd_ext_below_thresh_low && (Mode(params_ptr, Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr, Speed_Mode_FOC_Sensorless_HighFreq_Startup))) ||
                (T_cmd_ext_below_thresh && (Mode(params_ptr, Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr, Trq_Mode_FOC_Sensorless_HighFreq_Startup))))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(params_ptr, Trq_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                next = Torque_CL;
            }
            else if (T_cmd_ext_above_thresh && Mode(params_ptr, Trq_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
            else if (w_cmd_ext_above_thresh_high && Mode(params_ptr, Speed_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = High_Freq;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case High_Freq:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.high_freq.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup)) || (i_cmd_ext_below_thresh && Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup)))
            {
                next = Brake_Boot;
            }
            else if (i_cmd_ext_above_thresh && Mode(params_ptr, Curr_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = Current_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case High_Freq:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.high_freq.timer))
        {
            if ((w_cmd_ext_below_thresh_low && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup)) || (T_cmd_ext_below_thresh && Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup)))
            {
                next = Brake_Boot;
            }
            else if (T_cmd_ext_above_thresh && Mode(params_ptr, Trq_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                next = Torque_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            {
                sm_ptr->vars.speed_reset_required = true;
                next = Speed_CL;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Speed_OL_To_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((w_cmd_int_below_thresh_high && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup)))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.speed_ol_to_cl.timer))
        {
            next = Speed_CL;
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Speed_OL_To_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if ((w_cmd_int_below_thresh_high && Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.speed_ol_to_cl.timer))
        {
            next = Speed_CL;
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Dyno_Lock:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.dyno_lock.timer))
        {
            if (i_cmd_ext_above_thresh && Mode(params_ptr, Curr_Mode_FOC_Sensorless_Dyno))
            {
                next = Current_CL;
            }
        }
        break;
#elif defined(CTRL_METHOD_SFO)
    case Dyno_Lock:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (StopWatchIsDone(&sm_ptr->vars.dyno_lock.timer))
        {
            if (T_cmd_ext_above_thresh && Mode(params_ptr, Trq_Mode_FOC_Sensorless_Dyno))
            {
                next = Torque_CL;
            }
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
    case Prof_Rot_Lock:
    case Prof_R:
    case Prof_Ld:
    case Prof_Lq:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        if (cmd_below_thresh)
        {
            next = Brake_Boot;
        }
        else if (StopWatchIsDone(&profiler_ptr->timer)) // timer's period is dynamically set
        {
            next = (STATE_ID_t)(((uint8_t)(current)) + 1U);
        }
        break;
    case Prof_Finished:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        if (cmd_below_thresh)
        {
            next = Brake_Boot;
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Current_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (i_cmd_int_below_thresh && (Mode(params_ptr,Curr_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Curr_Mode_FOC_Sensorless_SixPulse_Startup) ||
            Mode(params_ptr,Curr_Mode_FOC_Sensorless_HighFreq_Startup) || Mode(params_ptr,Curr_Mode_FOC_Encoder_Align_Startup) || Mode(params_ptr,Curr_Mode_FOC_Hall)))
        {
            next = Brake_Boot;
        }
        else if (i_cmd_int_below_thresh && Mode(params_ptr, Curr_Mode_FOC_Sensorless_Dyno))
        {
            next = Dyno_Lock;
        }
        break;
#elif defined(CTRL_METHOD_TBC)
    case Current_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (i_cmd_int_below_thresh && Mode(params_ptr, Curr_Mode_Block_Comm_Hall))
        {
            next = Brake_Boot;
        }
        break;
#endif
#if defined(CTRL_METHOD_SFO)
    case Torque_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (T_cmd_int_below_thresh && (Mode(params_ptr,Trq_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Trq_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Trq_Mode_FOC_Sensorless_HighFreq_Startup)))
        {
            next = Brake_Boot;
        }
        else if (T_cmd_int_below_thresh && Mode(params_ptr, Trq_Mode_FOC_Sensorless_Dyno))
        {
            next = Dyno_Lock;
        }
        break;
#endif
    case Speed_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
#if defined(CTRL_METHOD_RFO)
        else if ((w_cmd_int_below_thresh_high && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Curr_Startup)))
            || ((sm_ptr->vars.high_freq.used ? w_cmd_int_below_thresh_low : w_cmd_int_below_thresh_high) && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            || (w_cmd_int_below_thresh_low && (Mode(params_ptr, Speed_Mode_FOC_Encoder_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Hall)))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if ((profiler_ptr->ramp_down_status == Task_Finished) && Mode(params_ptr, Profiler_Mode))
        {
            next = Prof_Finished;
        }
        break;
#elif defined(CTRL_METHOD_SFO)
        else if ((w_cmd_int_below_thresh_high && (Mode(params_ptr,Speed_Mode_FOC_Sensorless_Align_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_SixPulse_Startup) || Mode(params_ptr,Speed_Mode_FOC_Sensorless_Volt_Startup)))
            || ((sm_ptr->vars.high_freq.used ? w_cmd_int_below_thresh_low : w_cmd_int_below_thresh_high) && Mode(params_ptr,Speed_Mode_FOC_Sensorless_HighFreq_Startup))
            || (cmd_below_thresh && Mode(params_ptr,Profiler_Mode)))
        {
            next = Brake_Boot;
        }
        else if ((profiler_ptr->ramp_down_status == Task_Finished) && Mode(params_ptr, Profiler_Mode))
        {
            next = Prof_Finished;
        }
        break;
#elif defined(CTRL_METHOD_TBC)
        else if (w_cmd_int_below_thresh_low && Mode(params_ptr, Speed_Mode_Block_Comm_Hall))
        {
            next = Brake_Boot;
        }
        break;
#else
        else
        {
        }
        break;
#endif
#if defined(CTRL_METHOD_RFO)
    case Position_CL:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        break;

    case Catch_Spin:
        if (!vars_ptr->en)
        {
            next = Init;
        }
        else if (fault_trigger)
        {
            next = Fault;
        }
        else if (sm_ptr->vars.catch_spin.done && StopWatchIsDone(&sm_ptr->vars.catch_spin.timer))
        {
            if (ABS(vars_ptr->w_final_filt.elec) >= params_ptr->sys.catch_spin.w_thresh.elec)
            {
                next = Speed_CL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Sensorless_Volt_Startup))
            {
                next = Volt_OL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Sensorless_Align_Startup))
            {
                next = Align;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Sensorless_SixPulse_Startup))
            {
                next = Six_Pulse;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Sensorless_Curr_Startup))
            {
                next = Current_OL;
            }
            else if (w_cmd_ext_above_thresh_low && Mode(params_ptr, Speed_Mode_FOC_Hall))
            {
                next = Speed_CL;
            }
            else if (w_cmd_ext_below_thresh_low)
            {
                next = Brake_Boot;
            }
        }
        break;
#endif
    case Fault:
        if (sm_ptr->vars.fault.clr_success)
        {
            next = Init;
        }
        break;
    }
    sm_ptr->next = next;
}

/** @brief Initialise the state machine for all motor instances: build trigonometric LUTs,
 *         register Entry/Exit/RunISR0/RunISR1 handler pointers for every state, trigger
 *         the Init state Entry to load parameters and configure hardware, and start peripherals. */
void STATE_MACHINE_Init()
{
    PARAMS_UpdateLookupTable();

    for (uint32_t motor_ins = 0; motor_ins < MOTOR_CTRL_NO_OF_MOTOR; motor_ins++)
    {
        motor[motor_ins].sm_ptr->add_callback.RunISR0 = EmptyFcn;
        motor[motor_ins].sm_ptr->add_callback.RunISR1 = EmptyFcn;

        FeedbackISR0Wrap[motor_ins] = OBS_RunISR0;

        //									                        Entry(),
        // Exit(),				    RunISR0(),			    RunISR1()

        motor[motor_ins].sm_ptr->states[Init] = (STATE_t){&InitEntry, &EmptyFcn, &InitISR0, &EmptyFcn};
        motor[motor_ins].sm_ptr->states[Brake_Boot] = (STATE_t){&BrakeBootEntry, &BrakeBootExit, &BrakeBootISR0, &BrakeBootISR1};
        motor[motor_ins].sm_ptr->states[Volt_OL] = (STATE_t){&VoltOLEntry, &VoltOLExit, &VoltOLISR0, &VoltOLISR1};
        motor[motor_ins].sm_ptr->states[Speed_CL] = (STATE_t){&SpeedCLEntry, &EmptyFcn, &SpeedCLISR0, &SpeedCLISR1};
        motor[motor_ins].sm_ptr->states[Fault] = (STATE_t){&FaultEntry, &FaultExit, &EmptyFcn, &FaultISR1};
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_SFO)
        motor[motor_ins].sm_ptr->states[Align] = (STATE_t){&AlignEntry, &AlignExit, &AlignISR0, &EmptyFcn};
        motor[motor_ins].sm_ptr->states[Six_Pulse] = (STATE_t){&SixPulseEntry, &SixPulseExit, &SixPulseISR0, &SixPulseISR1};
        motor[motor_ins].sm_ptr->states[High_Freq] = (STATE_t){&HighFreqEntry, &HighFreqExit, &HighFreqISR0, &HighFreqISR1};
        motor[motor_ins].sm_ptr->states[Speed_OL_To_CL] = (STATE_t){&SpeedOLToCLEntry, &SpeedOLToCLExit, &SpeedOLToCLISR0, &SpeedOLToCLISR1};
        motor[motor_ins].sm_ptr->states[Dyno_Lock] = (STATE_t){&DynoLockEntry, &DynoLockExit, &DynoLockISR0, &DynoLockISR1};
        motor[motor_ins].sm_ptr->states[Prof_Rot_Lock] = (STATE_t){&MotorProfEntry, &MotorProfExit, &MotorProfISR0, &EmptyFcn};
        motor[motor_ins].sm_ptr->states[Prof_R] = (STATE_t){&MotorProfEntry, &MotorProfExit, &MotorProfISR0, &EmptyFcn};
        motor[motor_ins].sm_ptr->states[Prof_Ld] = (STATE_t){&MotorProfEntry, &MotorProfExit, &MotorProfISR0, &EmptyFcn};
        motor[motor_ins].sm_ptr->states[Prof_Lq] = (STATE_t){&MotorProfEntry, &MotorProfExit, &MotorProfISR0, &EmptyFcn};
        motor[motor_ins].sm_ptr->states[Prof_Finished] = (STATE_t){&MotorProfEntry, &MotorProfExit, &MotorProfISR0, &EmptyFcn};
#endif
#if defined(CTRL_METHOD_SFO)
        motor[motor_ins].sm_ptr->states[Torque_CL] = (STATE_t){&TorqueCLEntry, &EmptyFcn, &TorqueCLISR0, &TorqueCLISR1};
#elif defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
        motor[motor_ins].sm_ptr->states[Current_CL] = (STATE_t){&CurrentCLEntry, &EmptyFcn, &CurrentCLISR0, &CurrentCLISR1};
#endif
#if defined(CTRL_METHOD_RFO)
        motor[motor_ins].sm_ptr->states[Current_OL] = (STATE_t){&CurrentOLEntry, &CurrentOLExit, &CurrentOLISR0, &CurrentOLISR1};
        motor[motor_ins].sm_ptr->states[Position_CL] = (STATE_t){&PositionCLEntry, &EmptyFcn, &PositionCLISR0, &PositionCLISR1};
        motor[motor_ins].sm_ptr->states[Catch_Spin] = (STATE_t){&CatchSpinEntry, &CatchSpinExit, &CatchSpinISR0, &CatchSpinISR1};
#endif

        // Set initial state and trigger its Entry to load parameters and init hardware
        motor[motor_ins].sm_ptr->current = Init;
        motor[motor_ins].sm_ptr->vars.init.param_init_done = false;
        motor[motor_ins].sm_ptr->states[motor[motor_ins].sm_ptr->current].Entry(&motor[motor_ins]);

        motor[motor_ins].sm_ptr->vars.fault.clr_try_cnt = 0U;

        // Initialise the function-execution handler separately; it must NOT be
        // inside ResetAllModules because the GUI can call that at runtime
        FCN_EXE_HANDLER_Init();
        FCN_EXE_HANDLER_Reset();

#if defined(PC_TEST)
        for (uint32_t index = 0; index < sizeof(motor[motor_ins].sm_ptr->vars.capture_vals) / sizeof(float); ++index)
        {
            motor[motor_ins].sm_ptr->vars.capture_vals[index] = 0.0f;
            motor[motor_ins].sm_ptr->vars.capture_channels[index] = &motor[motor_ins].sm_ptr->vars.capture_vals[index];
        }
#endif
    }
    for (uint32_t motor_ins = 0; motor_ins < MOTOR_CTRL_NO_OF_MOTOR; motor_ins++)
    {
        hw_fcn.StartPeripherals(motor_ins); // enable all ISRs, PWMs, ADCs, etc.
    }
}

/** @brief High-frequency ISR0 dispatcher: runs common sensor/fault logic, the active
 *         state's fast control path (current loop, modulator), and any registered add-on callback.
 *  @param motor_ptr  Pointer to the motor instance */
RAMFUNC_BEGIN
void STATE_MACHINE_RunISR0(MOTOR_t *motor_ptr)
{
#if defined(PC_TEST)
    motor_ptr = &motor[0];
#endif
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    // 1. Run sensor acquisition and fault protection (common to all states)
    CommonISR0Wrap(motor_ptr);
    // 2. Run the active state's fast control path (current loop, modulator, etc.)
    sm_ptr->states[sm_ptr->current].RunISR0(motor_ptr);
    // 3. Run any optional add-on callback registered for this state (e.g. profiler)
    sm_ptr->add_callback.RunISR0(motor_ptr);
}
RAMFUNC_END

/** @brief Low-frequency ISR1 dispatcher: runs common slow-rate logic, the active state's
 *         speed/position loop, optional callback, transition condition evaluation, and
 *         executes any pending Exit/Entry state transition.
 *  @param motor_ptr  Pointer to the motor instance */
void STATE_MACHINE_RunISR1(MOTOR_t *motor_ptr)
{
#if defined(PC_TEST)
    motor_ptr = &motor[0];
#endif
    STATE_MACHINE_t *sm_ptr = motor_ptr->sm_ptr;
    // 1. Run slow-rate sensor processing and fault checks
    CommonISR1Wrap(motor_ptr);
    // 2. Run the active state's slow control path (speed loop, filters, ramps)
    sm_ptr->states[sm_ptr->current].RunISR1(motor_ptr);
    // 3. Run optional slow-rate callback
    sm_ptr->add_callback.RunISR1(motor_ptr);
    // 4. Evaluate transition conditions; sets sm_ptr->next if a change is needed
    ConditionCheck(motor_ptr);
    // 5. Execute pending state transition (if any)
    if (sm_ptr->next != sm_ptr->current)
    {
#if defined(PC_TEST)
        // Capture debug variable snapshot at the moment of the transition
        for (uint32_t index = 0; index < sizeof(sm_ptr->vars.capture_vals) / sizeof(float); ++index)
        {
            sm_ptr->vars.capture_vals[index] = *(sm_ptr->vars.capture_channels[index]);
        }
#endif
        sm_ptr->states[sm_ptr->current].Exit(motor_ptr);
        sm_ptr->states[sm_ptr->next].Entry(motor_ptr);
        // Update current LAST to preserve data integrity with ISR0 (which reads sm_ptr->current)
        sm_ptr->current = sm_ptr->next;
    }
}
