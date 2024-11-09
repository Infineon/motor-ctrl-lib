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

#include "HardwareIface.h"
#include "Controller.h"
#include "probe_scope.h"

// PSOC6.....CAT1A
// PSOC-C3...CAT1B
// XMC7200...CAT1C
// XMC4400...CAT3

#if defined(COMPONENT_CAT1A) || defined(COMPONENT_CAT1C)
extern uint8_t Em_Eeprom_Storage[srss_0_eeprom_0_PHYSICAL_SIZE];

#elif defined(COMPONENT_CAT1B)
const uint8_t *Em_Eeprom_Storage = (uint8_t *)(CY_FLASH_BASE + CY_FLASH_SIZE - srss_0_eeprom_0_PHYSICAL_SIZE);

#elif defined(COMPONENT_CAT3)
extern uint8_t Em_Eeprom_Storage[scu_0_eeprom_0_EEPROM_SIZE];

#endif
bool hall_gpio_u, hall_gpio_v, hall_gpio_w;

RAMFUNC_BEGIN
void MCU_PhaseUEnterHighZ()
{
#if defined(COMPONENT_CAT1)
    Cy_GPIO_SetHSIOM(PWMUL_PORT, PWMUL_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWMUH_PORT, PWMUH_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_Clr(PWMUL_PORT, PWMUL_NUM);
    Cy_GPIO_Clr(PWMUH_PORT, PWMUH_NUM);

#elif defined(COMPONENT_CAT3)
    XMC_GPIO_SetMode(PWMUL_PORT, PWMUL_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(PWMUH_PORT, PWMUH_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputLow(PWMUL_PORT, PWMUL_PIN);
    XMC_GPIO_SetOutputLow(PWMUH_PORT, PWMUH_PIN);

#endif
}
RAMFUNC_END

RAMFUNC_BEGIN
void MCU_PhaseUExitHighZ()
{
#if defined(COMPONENT_CAT1)
    Cy_GPIO_SetHSIOM(PWMUL_PORT, PWMUL_NUM, PWMUL_HSIOM);
    Cy_GPIO_SetHSIOM(PWMUH_PORT, PWMUH_NUM, PWMUH_HSIOM);

#elif defined(COMPONENT_CAT3)
    XMC_GPIO_SetMode(PWMUL_PORT, PWMUL_PIN, PWMUL_MODE);
    XMC_GPIO_SetMode(PWMUH_PORT, PWMUH_PIN, PWMUH_MODE);

#endif
}
RAMFUNC_END

RAMFUNC_BEGIN
void MCU_PhaseVEnterHighZ()
{
#if defined(COMPONENT_CAT1)
    Cy_GPIO_SetHSIOM(PWMVL_PORT, PWMVL_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWMVH_PORT, PWMVH_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_Clr(PWMVL_PORT, PWMVL_NUM);
    Cy_GPIO_Clr(PWMVH_PORT, PWMVH_NUM);

#elif defined(COMPONENT_CAT3)
    XMC_GPIO_SetMode(PWMVL_PORT, PWMVL_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(PWMVH_PORT, PWMVH_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputLow(PWMVL_PORT, PWMVL_PIN);
    XMC_GPIO_SetOutputLow(PWMVH_PORT, PWMVH_PIN);

#endif
}
RAMFUNC_END

RAMFUNC_BEGIN
void MCU_PhaseVExitHighZ()
{
#if defined(COMPONENT_CAT1)
    Cy_GPIO_SetHSIOM(PWMVL_PORT, PWMVL_NUM, PWMVL_HSIOM);
    Cy_GPIO_SetHSIOM(PWMVH_PORT, PWMVH_NUM, PWMVH_HSIOM);

#elif defined(COMPONENT_CAT3)
    XMC_GPIO_SetMode(PWMVL_PORT, PWMVL_PIN, PWMVL_MODE);
    XMC_GPIO_SetMode(PWMVH_PORT, PWMVH_PIN, PWMVH_MODE);

#endif
}
RAMFUNC_END

RAMFUNC_BEGIN
void MCU_PhaseWEnterHighZ()
{
#if defined(COMPONENT_CAT1)
    Cy_GPIO_SetHSIOM(PWMWL_PORT, PWMWL_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWMWH_PORT, PWMWH_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_Clr(PWMWL_PORT, PWMWL_NUM);
    Cy_GPIO_Clr(PWMWH_PORT, PWMWH_NUM);

#elif defined(COMPONENT_CAT3)
    XMC_GPIO_SetMode(PWMWL_PORT, PWMWL_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(PWMWH_PORT, PWMWH_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputLow(PWMWL_PORT, PWMWL_PIN);
    XMC_GPIO_SetOutputLow(PWMWH_PORT, PWMWH_PIN);

#endif
}
RAMFUNC_END

RAMFUNC_BEGIN
void MCU_PhaseWExitHighZ()
{
#if defined(COMPONENT_CAT1)
    Cy_GPIO_SetHSIOM(PWMWL_PORT, PWMWL_NUM, PWMWL_HSIOM);
    Cy_GPIO_SetHSIOM(PWMWH_PORT, PWMWH_NUM, PWMWH_HSIOM);

#elif defined(COMPONENT_CAT3)
    XMC_GPIO_SetMode(PWMWL_PORT, PWMWL_PIN, PWMWL_MODE);
    XMC_GPIO_SetMode(PWMWH_PORT, PWMWH_PIN, PWMWH_MODE);

#endif
}
RAMFUNC_END

#if defined(ANALOG_ROUTING_MUX_RUNTIME)
void (*MCU_RoutingConfigMUX0Wrap)() = &EmptyFcn;   // Either MUXA0 or MUXB0
void (*MCU_RoutingConfigMUX1Wrap)() = &EmptyFcn;   // Either MUXA1 or MUXB1
#else
void (*MCU_RoutingConfigMUXWrap)() = &EmptyFcn;    // Either MUXA or MUXB
#endif

#if defined(COMPONENT_CAT3)
RAMFUNC_BEGIN
static inline uint32_t XMC_CCU4_SLICE_GetTimerValueLong(XMC_CCU4_SLICE_t* slice_high, XMC_CCU4_SLICE_t* slice_low)
{
    uint16_t timer_h0, timer_h1, timer_l;
    uint32_t result;
    do
    {
        timer_h0 = XMC_CCU4_SLICE_GetTimerValue(slice_high);
        timer_l = XMC_CCU4_SLICE_GetTimerValue(slice_low);
        timer_h1 = XMC_CCU4_SLICE_GetTimerValue(slice_high);
        result = (uint32_t)(timer_h1 << 16) | (uint32_t)timer_l;
    }
    while (timer_h0 != timer_h1); // maximum one iteration
    return result;
}
RAMFUNC_END
#endif

RAMFUNC_BEGIN
void MCU_StartTimeCap(MCU_TIME_CAP_t* time_cap)
{
#if defined(COMPONENT_CAT1)
    time_cap->start = (int32_t)(Cy_TCPWM_Counter_GetCounter(EXE_TIMER_HW, EXE_TIMER_NUM));
#elif defined(COMPONENT_CAT3)
    time_cap->start = (int32_t)(XMC_CCU4_SLICE_GetTimerValueLong(EXE_TIMER_H_HW, EXE_TIMER_L_HW));
#endif
}
RAMFUNC_END

RAMFUNC_BEGIN
void MCU_StopTimeCap(MCU_TIME_CAP_t* time_cap)
{
    // Over flow and roll-over is OK as long as int32_t is used for 32bit timer/counters
#if defined(COMPONENT_CAT1)
    time_cap->stop = (int32_t)(Cy_TCPWM_Counter_GetCounter(EXE_TIMER_HW, EXE_TIMER_NUM));
#elif defined(COMPONENT_CAT3)
    time_cap->stop = (int32_t)(XMC_CCU4_SLICE_GetTimerValueLong(EXE_TIMER_H_HW, EXE_TIMER_L_HW));
#endif
    time_cap->duration_ticks = time_cap->stop - time_cap->start;
}
RAMFUNC_END

void MCU_ProcessTimeCapISR1(MCU_TIME_CAP_t* time_cap)
{
    time_cap->duration_sec = ((float)(time_cap->duration_ticks)) * (time_cap->sec_per_tick);
    time_cap->util = time_cap->duration_sec * time_cap->inv_max_time;
}

RAMFUNC_BEGIN
float MCU_TempSensorCalc()
{
    float result;
#if (ACTIVE_TEMP_SENSOR) // Active IC
    result = (hw.mcu.adc_scale.temp_ps * (uint16_t)hw.mcu.dma_results[ADC_TEMP]) - (TEMP_SENSOR_OFFSET / TEMP_SENSOR_SCALE);
#else // Passive NTC
    float lut_input = hw.mcu.adc_scale.temp_ps * (uint16_t)hw.mcu.dma_results[ADC_TEMP];
    uint32_t index = SAT(1U, TEMP_SENS_LUT_WIDTH - 1U, (uint32_t)(lut_input * Temp_Sens_LUT.step_inv));
    float input_index = Temp_Sens_LUT.step * index;
    result = Temp_Sens_LUT.val[index-1U] + (lut_input - input_index) * Temp_Sens_LUT.step_inv * (Temp_Sens_LUT.val[index] - Temp_Sens_LUT.val[index-1U]);
#endif
    return result;
}
RAMFUNC_END

RAMFUNC_BEGIN
void MCU_RunISR0()
{
#if defined(DMA_ADC_2_HW)
    if((++hw.mcu.isr0.count) % 3U != 0U) { return; }
#else
    if((++hw.mcu.isr0.count) % 2U != 0U) { return; }
#endif

    MCU_StartTimeCap(&hw.mcu.isr0_exe);

#if defined(COMPONENT_CAT3)
	DMA_ADC_1_Des_Addr[0U] = DMA_ADC_1_Src_Addr[0U];
	DMA_ADC_1_Des_Addr[1U] = DMA_ADC_1_Src_Addr[1U];
	DMA_ADC_1_Des_Addr[2U] = DMA_ADC_1_Src_Addr[2U];
	DMA_ADC_1_Des_Addr[3U] = DMA_ADC_1_Src_Addr[3U];
#endif

#if defined(CTRL_METHOD_TBC)
    if(ctrl.block_comm.enter_high_z_flag.u) { MCU_PhaseUEnterHighZ(); }
    if(ctrl.block_comm.enter_high_z_flag.v) { MCU_PhaseVEnterHighZ(); }
    if(ctrl.block_comm.enter_high_z_flag.w) { MCU_PhaseWEnterHighZ(); }

    if(ctrl.block_comm.exit_high_z_flag.u) { MCU_PhaseUExitHighZ(); }
    if(ctrl.block_comm.exit_high_z_flag.v) { MCU_PhaseVExitHighZ(); }
    if(ctrl.block_comm.exit_high_z_flag.w) { MCU_PhaseWExitHighZ(); }
#endif

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
       
#if defined(POSIF_ENC_HW) 
#if defined(COMPONENT_CAT1)
    if(params.sys.fb.mode == AqB_Enc)
    { 
        inc_encoder.pos_cap = Cy_TCPWM_Counter_GetCounter(ENC_POS_CNTR_HW, ENC_POS_CNTR_NUM);
        inc_encoder.per_cap = Cy_TCPWM_Counter_GetCapture(ENC_TIME_BTW_TICKS_HW, ENC_TIME_BTW_TICKS_NUM);
        inc_encoder.dir_cap = Cy_TCPWM_MOTIF_Get_Quaddec_Rotation_Direction(POSIF_ENC_HW) ? +1.0f : -1.0f;
        //faults.flags.sw.encoder = 0b0;
    }    
#elif defined(COMPONENT_CAT3)
    if(params.sys.fb.mode == AqB_Enc)
    { 
        inc_encoder.pos_cap = XMC_CCU4_SLICE_GetTimerValue(ENC_POS_CNTR_HW);
        inc_encoder.per_cap = XMC_CCU4_SLICE_GetCaptureRegisterValue(ENC_TIME_BTW_TICKS_HW, 1U);
        inc_encoder.dir_cap = ( XMC_POSIF_QD_GetDirection(POSIF_ENC_HW) == XMC_POSIF_QD_DIR_CLOCKWISE ) ? +1.0f : -1.0f;
        //faults.flags.sw.encoder = XMC_POSIF_GetEventStatus(POSIF_ENC_HW, XMC_POSIF_IRQ_EVENT_ERR);
    }
#endif    
#endif
    
#if defined(COMPONENT_CAT1)
    hall_gpio_u = !Cy_GPIO_Read(HALL_0_PORT, HALL_0_NUM);
    hall_gpio_v = !Cy_GPIO_Read(HALL_1_PORT, HALL_1_NUM);
    hall_gpio_w = !Cy_GPIO_Read(HALL_2_PORT, HALL_2_NUM);
#elif defined(COMPONENT_CAT3)
    hall_gpio_u = !XMC_GPIO_GetInput(HALL_0_PORT, HALL_0_PIN);
    hall_gpio_v = !XMC_GPIO_GetInput(HALL_1_PORT, HALL_1_PIN);
    hall_gpio_w = !XMC_GPIO_GetInput(HALL_2_PORT, HALL_2_PIN);
#endif
    hall.signal.u = hall_gpio_u;
    hall.signal.v = hall_gpio_v;
    hall.signal.w = hall_gpio_w;
    
    // SW capture (w/o POSIF)
    static bool hall_cap_sig, hall_cap_sig_prev = false;  
    static uint32_t hall_cap_val, hall_cap_val_prev = 0U;
    hall_cap_sig = hall.signal.u ^ hall.signal.v ^ hall.signal.w; // 6 steps per revolution
    if(TRANS_EDGE(hall_cap_sig_prev, hall_cap_sig))
    {
#if defined(COMPONENT_CAT1)
    	hall_cap_val = Cy_TCPWM_Counter_GetCounter(HALL_TIMER_HW, HALL_TIMER_NUM);
#elif defined(COMPONENT_CAT3)
    	hall_cap_val = XMC_CCU4_SLICE_GetTimerValueLong(HALL_TIMER_H_HW, HALL_TIMER_L_HW);
#endif
        hall.per_cap = hall_cap_val - hall_cap_val_prev;
        hall_cap_val_prev = hall_cap_val;
    }
    hall_cap_sig_prev = hall_cap_sig;
    
#endif
    
    const int32_t Curr_ADC_Half_Point_Ticks = (0x1<<11);
    sensor_iface.i_samp_0.raw = hw.mcu.adc_scale.i_uvw * (Curr_ADC_Half_Point_Ticks - (uint16_t)hw.mcu.dma_results[hw.mcu.adc_mux.idx_isamp[0]]);
    sensor_iface.i_samp_1.raw = hw.mcu.adc_scale.i_uvw * (Curr_ADC_Half_Point_Ticks - (uint16_t)hw.mcu.dma_results[hw.mcu.adc_mux.idx_isamp[1]]);
    sensor_iface.i_samp_2.raw = hw.mcu.adc_scale.i_uvw * (Curr_ADC_Half_Point_Ticks - (uint16_t)hw.mcu.dma_results[hw.mcu.adc_mux.idx_isamp[2]]);

#if defined(ANALOG_ROUTING_MUX_RUNTIME)
    if(hw.mcu.adc_mux.en)
    {
        hw.mcu.adc_mux.seq = (hw.mcu.isr0.count >> 1) & 0x1;
        switch(hw.mcu.adc_mux.seq)
        {
        case Analog_Routing_MUX_0:
        default:
            sensor_iface.v_dc.raw = hw.mcu.adc_scale.v_dc * (uint16_t)hw.mcu.dma_results[ADC_VBUS];
            sensor_iface.pot.raw = hw.mcu.adc_scale.v_pot * (uint16_t)hw.mcu.dma_results[ADC_VPOT];
            sensor_iface.temp_ps.raw = MCU_TempSensorCalc();
            MCU_RoutingConfigMUX1Wrap();
            break;
        case Analog_Routing_MUX_1:
            sensor_iface.v_uz.raw = hw.mcu.adc_scale.v_uvw * (uint16_t)hw.mcu.dma_results[ADC_VU];
            sensor_iface.v_vz.raw = hw.mcu.adc_scale.v_uvw * (uint16_t)hw.mcu.dma_results[ADC_VV];
            sensor_iface.v_wz.raw = hw.mcu.adc_scale.v_uvw * (uint16_t)hw.mcu.dma_results[ADC_VW];
            MCU_RoutingConfigMUX0Wrap();
            break;
        }
    }
    else
    {
        sensor_iface.v_dc.raw = hw.mcu.adc_scale.v_dc * (uint16_t)hw.mcu.dma_results[ADC_VBUS];
        sensor_iface.pot.raw = hw.mcu.adc_scale.v_pot * (uint16_t)hw.mcu.dma_results[ADC_VPOT];
        sensor_iface.temp_ps.raw = MCU_TempSensorCalc();
    } 
#else
	sensor_iface.v_uz.raw = hw.mcu.adc_scale.v_uvw * (uint16_t)hw.mcu.dma_results[ADC_VU];
	sensor_iface.v_vz.raw = hw.mcu.adc_scale.v_uvw * (uint16_t)hw.mcu.dma_results[ADC_VV];
	sensor_iface.v_wz.raw = hw.mcu.adc_scale.v_uvw * (uint16_t)hw.mcu.dma_results[ADC_VW];
    sensor_iface.v_dc.raw = hw.mcu.adc_scale.v_dc * (uint16_t)hw.mcu.dma_results[ADC_VBUS];
	sensor_iface.pot.raw = hw.mcu.adc_scale.v_pot * (uint16_t)hw.mcu.dma_results[ADC_VPOT];
	sensor_iface.temp_ps.raw = MCU_TempSensorCalc();
#endif

    STATE_MACHINE_RunISR0();

    UVW_t d_uvw_cmd_adj = PWM_INVERSION ? (UVW_t){.w=(1.0f - vars.d_uvw_cmd.w), .v=(1.0f - vars.d_uvw_cmd.v), .u=(1.0f - vars.d_uvw_cmd.u)} :
                                          (UVW_t){.w=vars.d_uvw_cmd.w, .v=vars.d_uvw_cmd.v, .u=vars.d_uvw_cmd.u};
    uint32_t pwm_u_cc = (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * d_uvw_cmd_adj.u);
    uint32_t pwm_v_cc = (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * d_uvw_cmd_adj.v);
    uint32_t pwm_w_cc = (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * d_uvw_cmd_adj.w);

#if defined(COMPONENT_CAT1)
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_U_HW, PWM_U_NUM, pwm_u_cc);
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_U_HW, PWM_U_NUM, pwm_u_cc);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_V_HW, PWM_V_NUM, pwm_v_cc);
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_V_HW, PWM_V_NUM, pwm_v_cc);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_W_HW, PWM_W_NUM, pwm_w_cc);
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_W_HW, PWM_W_NUM, pwm_w_cc);

#elif defined(COMPONENT_CAT3)
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_U_HW, pwm_u_cc);
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_U_HW, pwm_u_cc);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_V_HW, pwm_v_cc);
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_V_HW, pwm_v_cc);
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_W_HW, pwm_w_cc);
	XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_W_HW, pwm_w_cc);

	XMC_CCU8_EnableShadowTransfer(PWM_UVW_ADCn_ISRn_HW, XMC_CCU8_SHADOW_TRANSFER_PWM_U_SLICE | XMC_CCU8_SHADOW_TRANSFER_PWM_V_SLICE | XMC_CCU8_SHADOW_TRANSFER_PWM_W_SLICE);
#endif

    uint32_t adc_isr0_cc_samp0, adc_isr0_cc_samp1;
    if(params.sys.analog.shunt.type == Single_Shunt)
    {
        adc_isr0_cc_samp0 = PWM_INVERSION ? (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * (1.0f + vars.d_samp[0])) : (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * vars.d_samp[0]);
        adc_isr0_cc_samp1 = PWM_INVERSION ? (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * (1.0f + vars.d_samp[1])) : (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * vars.d_samp[1]);
#if defined(COMPONENT_CAT1)
        Cy_TCPWM_PWM_SetCompare0BufVal(ADC0_ISR0_HW, ADC0_ISR0_NUM, adc_isr0_cc_samp0);
        Cy_TCPWM_PWM_SetCompare1BufVal(ADC1_ISR0_HW, ADC1_ISR0_NUM, adc_isr0_cc_samp1);
#elif defined(COMPONENT_CAT3)
    	XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(ADC0_ISR0_HW, adc_isr0_cc_samp0);
    	XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(ADC1_ISR0_HW, adc_isr0_cc_samp1);
    	XMC_CCU8_EnableShadowTransfer(PWM_UVW_ADCn_ISRn_HW, XMC_CCU8_SHADOW_TRANSFER_ADC0_ISR0_SLICE | XMC_CCU8_SHADOW_TRANSFER_ADC1_ISR0_SLICE);
#endif
    }
    
    ProbeScope_Sampling();

    MCU_StopTimeCap(&hw.mcu.isr0_exe);
}
RAMFUNC_END

#if defined(COMPONENT_CAT1)
RAMFUNC_BEGIN
static void DMA_ADC_0_RunISR() {
	Cy_DMA_Channel_ClearInterrupt(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL);
    NVIC_ClearPendingIRQ(hw.mcu.interrupt.nvic_dma_adc_0);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL, &DMA_ADC_0_Descriptor_0);
    MCU_RunISR0();
}
RAMFUNC_END

RAMFUNC_BEGIN
static void DMA_ADC_1_RunISR() {
    Cy_DMA_Channel_ClearInterrupt(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL);
    NVIC_ClearPendingIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL, &DMA_ADC_1_Descriptor_0);
    MCU_RunISR0();
}
RAMFUNC_END

#if defined(DMA_ADC_2_HW)
RAMFUNC_BEGIN
static void DMA_ADC_2_RunISR() {
    Cy_DMA_Channel_ClearInterrupt(DMA_ADC_2_HW, DMA_ADC_2_CHANNEL);
    NVIC_ClearPendingIRQ(hw.mcu.interrupt.nvic_dma_adc_2);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_2_HW, DMA_ADC_2_CHANNEL, &DMA_ADC_2_Descriptor_0);
    MCU_RunISR0();
}
RAMFUNC_END
#endif

#elif defined(COMPONENT_CAT3)
RAMFUNC_BEGIN
void DMA_ADC_0_DMA_ADC_2_RunISR(void)
{	// Note: only GPDMA channels 0 and 1 can do burst-mode transfers!
	XMC_DMA_CH_EVENT_t event = XMC_DMA_CH_GetEventStatus(DMA_ADC_0_HW, DMA_ADC_0_NUM);
	if(event == XMC_DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE)   // DMA_ADC_0
	{
		XMC_DMA_CH_ClearEventStatus(DMA_ADC_0_HW, DMA_ADC_0_NUM, XMC_DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE);
		MCU_RunISR0();
	}
	else    // DMA_ADC_2
	{
		XMC_DMA_CH_ClearEventStatus(DMA_ADC_2_HW, DMA_ADC_2_NUM, XMC_DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE);
		MCU_RunISR0();
	}
	NVIC_ClearPendingIRQ(hw.mcu.interrupt.nvic_dma_adc_0);
}
RAMFUNC_END

RAMFUNC_BEGIN
void DMA_ADC_1_RunISR()
{
	MCU_RunISR0();
	NVIC_ClearPendingIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
}
RAMFUNC_END
#endif

void MCU_RunISR1()
{
#if defined(COMPONENT_CAT1)
    Cy_TCPWM_ClearInterrupt(SYNC_ISR1_HW, SYNC_ISR1_NUM, SYNC_ISR1_config.interruptSources);
#elif defined(COMPONENT_CAT3)
	XMC_CCU8_SLICE_ClearEvent((XMC_CCU8_SLICE_t*) SYNC_ISR1_HW, XMC_CCU8_SLICE_IRQ_ID_COMPARE_MATCH_UP_CH_1);
#endif
    
	NVIC_ClearPendingIRQ(hw.mcu.interrupt.nvic_sync_isr1);
    MCU_StartTimeCap(&hw.mcu.isr1_exe);
    if(hw.mcu.isr1.count++ == 1U) {MCU_DisableTimerReload();}

    // Smart gate driver
#if defined(USING_SGD)
    SGD_RunISR1();
    sensor_iface.digital.fault = !Cy_GPIO_Read(N_FAULT_SGD_PORT, N_FAULT_SGD_NUM);
#else // using simple gate drivers
#if defined(COMPONENT_CAT1)
    sensor_iface.digital.fault = !Cy_GPIO_Read(N_FAULT_HW_PORT, N_FAULT_HW_NUM);
#elif defined(COMPONENT_CAT3)
    sensor_iface.digital.fault = !XMC_GPIO_GetInput(N_FAULT_HW_PORT, N_FAULT_HW_PIN);
#endif
    faults.flags.hw.cs_ocp = sensor_iface.digital.fault ? 0b111 : 0b000; // hw faults only cover over-current without SGD
#endif
    
    // Direction switch
#if defined(DIR_SWITCH_PORT) // switch
    sensor_iface.digital.dir = Cy_GPIO_Read(DIR_SWITCH_PORT, DIR_SWITCH_NUM);
#elif defined(N_DIR_PUSHBTN_PORT) // push button
    static bool user_btn_prev, user_btn = true;
    user_btn_prev = user_btn;
    user_btn = Cy_GPIO_Read(N_DIR_PUSHBTN_PORT, N_DIR_PUSHBTN_NUM);
    sensor_iface.digital.dir = FALL_EDGE(user_btn_prev, user_btn) ? ~sensor_iface.digital.dir : sensor_iface.digital.dir; // toggle switch    
#endif
    
    // Direction LED
#if defined(DIR_LED_PORT)
#if defined(COMPONENT_CAT1)
    Cy_GPIO_Write(DIR_LED_PORT, DIR_LED_NUM, (vars.dir == +1.0f));
#elif defined(COMPONENT_CAT3)
    XMC_GPIO_SetOutputLevel(DIR_LED_PORT, DIR_LED_PIN, (vars.dir == +1.0f) ? XMC_GPIO_OUTPUT_LEVEL_HIGH : XMC_GPIO_OUTPUT_LEVEL_LOW);
#endif
#endif
    
    // Brake switch
#if defined(N_BRK_SWITCH_PORT)
    sensor_iface.digital.brk = !Cy_GPIO_Read(N_BRK_SWITCH_PORT, N_BRK_SWITCH_NUM);
#else
    sensor_iface.digital.brk = 0x0; // no brake switch   
#endif    
    
    // Control ISR1
    STATE_MACHINE_RunISR1();
    
    // SW fault LED
#if defined(N_FAULT_LED_SW_PORT) // seperate leds for hw and sw faults
    Cy_GPIO_Write(N_FAULT_LED_SW_PORT, N_FAULT_LED_SW_NUM, (bool)(!faults.flags_latched.sw.reg));
#elif defined(FAULT_LED_ALL_PORT) // one led for all faults
#if defined(COMPONENT_CAT1)
    Cy_GPIO_Write(FAULT_LED_ALL_PORT, FAULT_LED_ALL_NUM, (bool)(faults.flags_latched.all));
#elif defined(COMPONENT_CAT3)
    XMC_GPIO_SetOutputLevel(FAULT_LED_ALL_PORT, FAULT_LED_ALL_PIN, (bool)(faults.flags_latched.all) ? XMC_GPIO_OUTPUT_LEVEL_HIGH : XMC_GPIO_OUTPUT_LEVEL_LOW);
#endif
#endif
   
    MCU_StopTimeCap(&hw.mcu.isr1_exe);
    MCU_ProcessTimeCapISR1(&hw.mcu.isr0_exe);
    MCU_ProcessTimeCapISR1(&hw.mcu.isr1_exe);
    
    // Watchdog kick
#if defined(CY_USING_HAL)
    cyhal_wdt_kick(&hw.mcu.wdt_obj);
#endif
}

void MCU_EnterCriticalSection()
{
#if defined(COMPONENT_CAT1)
    hw.mcu.interrupt.state = Cy_SysLib_EnterCriticalSection();
#elif defined(COMPONENT_CAT3)
    hw.mcu.interrupt.state = XMC_EnterCriticalSection();
#endif
}

void MCU_ExitCriticalSection()
{
#if defined(COMPONENT_CAT1)
    Cy_SysLib_ExitCriticalSection(hw.mcu.interrupt.state);
#elif defined(COMPONENT_CAT3)
    XMC_ExitCriticalSection(hw.mcu.interrupt.state);
#endif
}

void MCU_GateDriverEnterHighZ()
{
#if defined(ANALOG_ROUTING_MUX_RUNTIME)
    hw.mcu.adc_mux.en = true;
#endif
 
#if defined(USING_SGD) // smart gate driver
    Cy_GPIO_Clr(N_BRK_SGD_PORT, N_BRK_SGD_PIN);
#endif
    
    MCU_PhaseUEnterHighZ();
    MCU_PhaseVEnterHighZ();
    MCU_PhaseWEnterHighZ();
}

void MCU_GateDriverExitHighZ()
{
#if defined(ANALOG_ROUTING_MUX_RUNTIME)
    hw.mcu.adc_mux.en = false;
    MCU_RoutingConfigMUX0Wrap();
#endif
    
#if defined(USING_SGD) // smart gate driver
    Cy_GPIO_Set(N_BRK_SGD_PORT, N_BRK_SGD_PIN);
#endif
    
    MCU_PhaseUExitHighZ();
    MCU_PhaseVExitHighZ();
    MCU_PhaseWExitHighZ();
}

void MCU_Init()
{
    MCU_InitChipInfo();
    MCU_InitInterrupts();
    MCU_InitADCs();
    MCU_InitAnalogRouting();
    MCU_InitDMAs();
    MCU_InitTimers();
    MCU_InitPosInterface();
    MCU_InitWatchdog();
    ProbeScope_Init((uint32_t)params.sys.samp.fs0);
    sensor_iface.digital.dir = true; // initial direction is positive
}

void MCU_InitChipInfo()
{
#if defined(COMPONENT_CAT1)
    mc_info.chip_id = Cy_SysLib_GetDevice();
    mc_info.chip_id <<= 16;
    mc_info.chip_id |= Cy_SysLib_GetDeviceRevision();
#elif defined(COMPONENT_CAT3)
    mc_info.chip_id = SCU_GENERAL->IDCHIP;
#endif
}

void MCU_InitInterrupts()
{
    // Interrupt callbacks and priorities (higher value = lower urgency) .......
#if defined(COMPONENT_CAT1)
    // DMA_ADC_0:
    cy_stc_sysint_t DMA_ADC_0_cfg = { .intrSrc = DMA_ADC_0_IRQ, .intrPriority = 0 };
    Cy_SysInt_Init(&DMA_ADC_0_cfg, DMA_ADC_0_RunISR);
    // DMA_ADC_1:
    cy_stc_sysint_t DMA_ADC_1_cfg = { .intrSrc = DMA_ADC_1_IRQ, .intrPriority = 0 };
    Cy_SysInt_Init(&DMA_ADC_1_cfg, DMA_ADC_1_RunISR);
#if defined(DMA_ADC_2_IRQ)
    // DMA_ADC_2:
    cy_stc_sysint_t DMA_ADC_2_cfg = { .intrSrc = DMA_ADC_2_IRQ, .intrPriority = 0 };
    Cy_SysInt_Init(&DMA_ADC_2_cfg, DMA_ADC_2_RunISR);
#endif    
    // ISR1:
    cy_stc_sysint_t ISR1_cfg = { .intrSrc = SYNC_ISR1_IRQ, .intrPriority = 1 };
    Cy_SysInt_Init(&ISR1_cfg, MCU_RunISR1);
    
#elif defined(COMPONENT_CAT3)   
    NVIC_SetPriority(DMA_ADC_0_IRQ, 0); // DMA_ADC_0
	NVIC_SetPriority(DMA_ADC_1_IRQ, 0); // DMA_ADC_1
    NVIC_SetPriority(SYNC_ISR1_IRQ, 1); // ISR1

#endif

    // NVIC connections ........................................................
#if defined(COMPONENT_CAT1A) || defined(COMPONENT_CAT1B) || defined(COMPONENT_CAT3)
    hw.mcu.interrupt.nvic_dma_adc_0 = DMA_ADC_0_IRQ;
    hw.mcu.interrupt.nvic_dma_adc_1 = DMA_ADC_1_IRQ;
    hw.mcu.interrupt.nvic_sync_isr1 = SYNC_ISR1_IRQ;
#elif defined(COMPONENT_CAT1C)
    hw.mcu.interrupt.nvic_dma_adc_0 = Cy_SysInt_GetNvicConnection(DMA_ADC_0_IRQ);
    hw.mcu.interrupt.nvic_dma_adc_1 = Cy_SysInt_GetNvicConnection(DMA_ADC_1_IRQ);
    hw.mcu.interrupt.nvic_dma_adc_2 = Cy_SysInt_GetNvicConnection(DMA_ADC_2_IRQ);
    hw.mcu.interrupt.nvic_sync_isr1 = Cy_SysInt_GetNvicConnection(SYNC_ISR1_IRQ);
#endif
}

void MCU_InitADCs()
{
    // ADC conversion coefficients .............................................
#if defined(USING_SGD) // smart gate driver with adjustable current sense gains
    static const float Cs_Gain_LUT[8] = {4.0f, 8.0f, 12.0f, 16.0f, 20.0f, 24.0f, 32.0f, 64.0f};
    float cs_gain = Cs_Gain_LUT[params.sys.analog.shunt.opamp_gain & 0x7];
#else
    float cs_gain = params.sys.analog.shunt.opamp_gain;
#endif
    hw.mcu.adc_scale.i_uvw = (ADC_VREF_GAIN * CY_CFG_PWR_VDDA_MV * 1.0E-3f) / ((1<<12U) * params.sys.analog.shunt.res * cs_gain); // [A/ticks]
    hw.mcu.adc_scale.v_uvw = (ADC_VREF_GAIN * CY_CFG_PWR_VDDA_MV * 1.0E-3f) / ((1<<12U) * ADC_SCALE_VUVW); // [V/ticks]
    hw.mcu.adc_scale.v_dc = (ADC_VREF_GAIN * CY_CFG_PWR_VDDA_MV * 1.0E-3f) / ((1<<12U) * ADC_SCALE_VDC); // [V/ticks]
    hw.mcu.adc_scale.v_pot = 1.0f / (1<<12U); // [%/ticks]
#if (ACTIVE_TEMP_SENSOR)
    hw.mcu.adc_scale.temp_ps = (ADC_VREF_GAIN * CY_CFG_PWR_VDDA_MV * 1.0E-3f) / ((1<<12U) * TEMP_SENSOR_SCALE); // [Celsius/ticks]
#else // passive NTC
    hw.mcu.adc_scale.temp_ps = 1.0f / (1<<12U); // [1/ticks], normalized voltage wrt Vcc
#endif  
    
    // Configure ADC modules ...................................................
#if defined(COMPONENT_CAT1A)
    Cy_SysAnalog_Init(&ADC_AREF_config);
    Cy_SAR_CommonInit(PASS, &ADC_COMMON_config);
    Cy_SAR_Init(ADC_0_HW, &ADC_0_config);
    Cy_SAR_Init(ADC_1_HW, &ADC_1_config);

#elif defined(COMPONENT_CAT1B)
    Cy_HPPASS_Init(&pass_0_config);

#elif defined(COMPONENT_CAT1C)
    Cy_SAR2_Init(ADC_0_HW, &ADC_0_config);
    Cy_SAR2_Init(ADC_1_HW, &ADC_1_config);
    Cy_SAR2_Init(ADC_2_HW, &ADC_2_config);

#elif defined(COMPONENT_CAT3)
	XMC_VADC_QUEUE_CONFIG_t adc_queue_config; // VADC queue trigger configurations

	adc_queue_config = ADC_0_queue_config;
	adc_queue_config.trigger_signal = (uint32_t) ADC_0_TRIG;
	adc_queue_config.external_trigger = (uint32_t) true;
    XMC_VADC_GROUP_QueueInit(ADC_0_HW, &adc_queue_config);

    adc_queue_config = ADC_1_queue_config;
	adc_queue_config.trigger_signal = (uint32_t) ADC_1_TRIG;
	adc_queue_config.external_trigger = (uint32_t) true;
    XMC_VADC_GROUP_QueueInit(ADC_1_HW, &adc_queue_config);

    adc_queue_config = ADC_2_queue_config;
	adc_queue_config.trigger_signal = (uint32_t) ADC_2_TRIG;
	adc_queue_config.external_trigger = (uint32_t) true;
    XMC_VADC_GROUP_QueueInit(ADC_2_HW, &adc_queue_config);

#endif
}

void MCU_InitAnalogRouting()
{
    // Default indices (may be changed by routing configuration mux functions) ......
    hw.mcu.adc_mux.idx_isamp[0] = ADC_ISAMPA; 
    hw.mcu.adc_mux.idx_isamp[1] = ADC_ISAMPB;
    hw.mcu.adc_mux.idx_isamp[2] = ADC_ISAMPC;

    // Routing configuration mux functions ..........................................
#if defined(ANALOG_ROUTING_MUX_RUNTIME)
    MCU_RoutingConfigMUX0Wrap = (params.sys.analog.shunt.type == Single_Shunt) ? MCU_RoutingConfigMUXB0 : MCU_RoutingConfigMUXA0;
    MCU_RoutingConfigMUX1Wrap = (params.sys.analog.shunt.type == Single_Shunt) ? MCU_RoutingConfigMUXB1 : MCU_RoutingConfigMUXA1;
    MCU_RoutingConfigMUX0Wrap();
#else
    MCU_RoutingConfigMUXWrap = (params.sys.analog.shunt.type == Single_Shunt) ? MCU_RoutingConfigMUXB : MCU_RoutingConfigMUXA;
    MCU_RoutingConfigMUXWrap();
    hw.mcu.adc_mux.en = true;
#endif
}

void MCU_InitDMAs()
{
#if defined(COMPONENT_CAT1)
    // Configure DMA descriptors ...............................................
    for (uint8_t seq_idx=0U; seq_idx<ADC_SEQ_MAX; ++seq_idx)
    {
        for (uint8_t samp_idx=0U; samp_idx<ADC_SAMP_PER_SEQ_MAX; ++samp_idx)
        {
        	Cy_DMA_Descriptor_Init(DMA_Descriptors[seq_idx][samp_idx], DMA_Descriptor_Configs[seq_idx][samp_idx]);
        	Cy_DMA_Descriptor_SetSrcAddress(DMA_Descriptors[seq_idx][samp_idx], ADC_Result_Regs[seq_idx][samp_idx]);
        	Cy_DMA_Descriptor_SetDstAddress(DMA_Descriptors[seq_idx][samp_idx], &hw.mcu.dma_results[DMA_Result_Indices[seq_idx][samp_idx]]);
        }
    }
    // Configure DMA channels ..................................................
    Cy_DMA_Channel_Init(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL, &DMA_ADC_0_channelConfig);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL, &DMA_ADC_0_Descriptor_0);
    Cy_DMA_Channel_SetInterruptMask(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL, CY_DMA_INTR_MASK);
    Cy_DMA_Channel_Enable(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL);

    Cy_DMA_Channel_Init(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL, &DMA_ADC_1_channelConfig);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL, &DMA_ADC_1_Descriptor_0);
    Cy_DMA_Channel_SetInterruptMask(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL, CY_DMA_INTR_MASK);
    Cy_DMA_Channel_Enable(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL);

#if defined(DMA_ADC_2_HW)
    Cy_DMA_Channel_Init(DMA_ADC_2_HW, DMA_ADC_2_CHANNEL, &DMA_ADC_2_channelConfig);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_2_HW, DMA_ADC_2_CHANNEL, &DMA_ADC_2_Descriptor_0);
    Cy_DMA_Channel_SetInterruptMask(DMA_ADC_2_HW, DMA_ADC_2_CHANNEL, CY_DMA_INTR_MASK);
    Cy_DMA_Channel_Enable(DMA_ADC_2_HW, DMA_ADC_2_CHANNEL);
#endif

#elif (COMPONENT_CAT3)
    // Configure DMA source / destinations......................................
    XMC_DMA_CH_EnableSourceAddressReload(DMA_ADC_0_HW, DMA_ADC_0_NUM);
    XMC_DMA_CH_EnableDestinationAddressReload(DMA_ADC_0_HW, DMA_ADC_0_NUM);
    XMC_DMA_CH_Enable(DMA_ADC_0_HW, DMA_ADC_0_NUM);

    // No DMA_ADC_1 because only first two dma channels can do burst-mode transfers!

    XMC_DMA_CH_EnableSourceAddressReload(DMA_ADC_2_HW, DMA_ADC_2_NUM);
    XMC_DMA_CH_EnableDestinationAddressReload(DMA_ADC_2_HW, DMA_ADC_2_NUM);
    XMC_DMA_CH_Enable(DMA_ADC_2_HW, DMA_ADC_2_NUM);

#endif
}

void MCU_InitTimers()
{
    // Clock frequencies .......................................................
#if defined(COMPONENT_CAT1A)
    hw.mcu.clk.tcpwm = Cy_SysClk_PeriphGetFrequency(CLK_TCPWM_HW, CLK_TCPWM_NUM); // [Hz]
    hw.mcu.clk.hall = Cy_SysClk_PeriphGetFrequency(CLK_TCPWM_HW, CLK_HALL_NUM); // [Hz]  
#elif defined(COMPONENT_CAT1B) || defined(COMPONENT_CAT1C)
    hw.mcu.clk.tcpwm = Cy_SysClk_PeriPclkGetFrequency((en_clk_dst_t)CLK_TCPWM_GRP_NUM, CY_SYSCLK_DIV_8_BIT, CLK_TCPWM_NUM); // [Hz]
    hw.mcu.clk.hall = Cy_SysClk_PeriPclkGetFrequency((en_clk_dst_t)CLK_HALL_GRP_NUM, CY_SYSCLK_DIV_8_BIT, CLK_HALL_NUM); // [Hz]
    hw.mcu.clk.encoder = Cy_SysClk_PeriPclkGetFrequency((en_clk_dst_t)CLK_ENC_GRP_NUM, CY_SYSCLK_DIV_8_BIT, CLK_ENC_NUM); // [Hz]
#elif defined(COMPONENT_CAT3)
    hw.mcu.clk.tcpwm = XMC_SCU_CLOCK_GetCcuClockFrequency() / (1U << PWM_U_compare_config.prescaler_initval);
    hw.mcu.clk.hall = XMC_SCU_CLOCK_GetCcuClockFrequency() / (1U << HALL_TIMER_L_compare_config.prescaler_initval);
    hw.mcu.clk.encoder = XMC_SCU_CLOCK_GetCcuClockFrequency() / (1U << ENC_TIME_BTW_TICKS_capture_config.prescaler_initval);
#endif
    
    // Timer calculations ......................................................
    hall.per_cap_freq = hw.mcu.clk.hall; // [Hz]
    inc_encoder.per_cap_freq = hw.mcu.clk.encoder; // [Hz]
    hw.mcu.pwm.count = 0U;
    hw.mcu.pwm.period = ((uint32_t)(hw.mcu.clk.tcpwm * params.sys.samp.tpwm))&(~((uint32_t)(0x1))); // must be even
    hw.mcu.pwm.duty_cycle_coeff = (float)(hw.mcu.pwm.period >> 1);
    hw.mcu.isr0.count = 0U;
    hw.mcu.isr0.period = hw.mcu.pwm.period * params.sys.samp.fpwm_fs0_ratio;
    hw.mcu.isr0.duty_cycle_coeff = (float)(hw.mcu.isr0.period);
    hw.mcu.isr1.count = 0U;
    hw.mcu.isr1.period = hw.mcu.isr0.period * params.sys.samp.fs0_fs1_ratio;
    hw.mcu.isr1.duty_cycle_coeff = (float)(hw.mcu.isr1.period);
    hw.mcu.isr0_exe.sec_per_tick = (1.0f/hw.mcu.clk.tcpwm); // [sec/ticks]
    hw.mcu.isr0_exe.inv_max_time = params.sys.samp.fs0; // [1/sec]
    hw.mcu.isr1_exe.sec_per_tick = (1.0f/hw.mcu.clk.tcpwm); // [sec/ticks]
    hw.mcu.isr1_exe.inv_max_time = params.sys.samp.fs1; // [1/sec]  
  
    // Configure timers (TCPWMs/CCU8/CCU4) .....................................
#if defined(COMPONENT_CAT1)   
    uint32_t cc0 = PWM_INVERSION ? (hw.mcu.pwm.period - PWM_TRIG_ADVANCE) : (hw.mcu.pwm.period >> 1);
    
    Cy_TCPWM_PWM_Init(ADC0_ISR0_HW, ADC0_ISR0_NUM, &ADC0_ISR0_config);
    Cy_TCPWM_PWM_Init(ADC1_ISR0_HW, ADC1_ISR0_NUM, &ADC1_ISR0_config);
    Cy_TCPWM_PWM_SetPeriod0(ADC0_ISR0_HW, ADC0_ISR0_NUM, hw.mcu.isr0.period - 1U); // Sawtooth carrier
    Cy_TCPWM_PWM_SetPeriod0(ADC1_ISR0_HW, ADC1_ISR0_NUM, hw.mcu.isr0.period - 1U); // Sawtooth carrier
    Cy_TCPWM_PWM_SetCompare0Val(ADC0_ISR0_HW, ADC0_ISR0_NUM, cc0); // Read ADCs at the middle of lower switches' on-times
    Cy_TCPWM_PWM_SetCompare1Val(ADC1_ISR0_HW, ADC1_ISR0_NUM, cc0); // Read ADCs at the middle of lower switches' on-times
    Cy_TCPWM_PWM_SetCompare0BufVal(ADC0_ISR0_HW, ADC0_ISR0_NUM, cc0); // Read ADCs at the middle of lower switches' on-times   
    Cy_TCPWM_PWM_SetCompare1BufVal(ADC1_ISR0_HW, ADC1_ISR0_NUM, cc0); // Read ADCs at the middle of lower switches' on-times
    
    Cy_TCPWM_PWM_Init(PWM_SYNC_HW, PWM_SYNC_NUM, &PWM_SYNC_config);
    Cy_TCPWM_PWM_SetPeriod0(PWM_SYNC_HW, PWM_SYNC_NUM, hw.mcu.isr0.period - 1U); // Sawtooth carrier
    Cy_TCPWM_PWM_SetCompare0Val(PWM_SYNC_HW, PWM_SYNC_NUM, cc0); // Swap PWMs CC0/CC1 at the middle of lower switches' on-times

    Cy_TCPWM_PWM_Init(PWM_U_HW, PWM_U_NUM, &PWM_U_config);
    Cy_TCPWM_PWM_SetPeriod0(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 1); // Triangle carrier
    Cy_TCPWM_PWM_SetCompare0Val(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1Val(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%

    Cy_TCPWM_PWM_Init(PWM_V_HW, PWM_V_NUM, &PWM_V_config);
    Cy_TCPWM_PWM_SetPeriod0(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 1); // Triangle carrier
    Cy_TCPWM_PWM_SetCompare0Val(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1Val(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%

    Cy_TCPWM_PWM_Init(PWM_W_HW, PWM_W_NUM, &PWM_W_config);
    Cy_TCPWM_PWM_SetPeriod0(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 1); // Triangle carrier
    Cy_TCPWM_PWM_SetCompare0Val(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1Val(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%

    cc0 = PWM_INVERSION ? (hw.mcu.isr1.period - (hw.mcu.pwm.period >> 1)) : hw.mcu.isr1.period - 1U;
    Cy_TCPWM_PWM_Init(SYNC_ISR1_HW, SYNC_ISR1_NUM, &SYNC_ISR1_config);
    Cy_TCPWM_PWM_SetPeriod0(SYNC_ISR1_HW, SYNC_ISR1_NUM, hw.mcu.isr1.period - 1U); // Sawtooth carrier
    Cy_TCPWM_PWM_SetCompare0Val(SYNC_ISR1_HW, SYNC_ISR1_NUM, cc0);

#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    Cy_TCPWM_Counter_Init(HALL_TIMER_HW, HALL_TIMER_NUM, &HALL_TIMER_config);   // Hall sensor speed capture
#endif
    Cy_TCPWM_Counter_Init(EXE_TIMER_HW, EXE_TIMER_NUM, &EXE_TIMER_config);      // Execution timer

#if defined(POSIF_ENC_HW)
    Cy_TCPWM_Counter_Init(ENC_POS_CNTR_HW, ENC_POS_CNTR_NUM, &ENC_POS_CNTR_config);
    Cy_TCPWM_Counter_Init(ENC_TIME_BTW_TICKS_HW, ENC_TIME_BTW_TICKS_NUM, &ENC_TIME_BTW_TICKS_config);
#endif

    sensor_iface.uvw_idx = PWM_INVERSION ? &ctrl.volt_mod.uvw_idx_prev : &ctrl.volt_mod.uvw_idx;
    
#elif defined(COMPONENT_CAT3)

#if (PWM_INVERSION == true)
#error "PWM inversion is not supported for CAT3 device."
#endif    
    XMC_CCU8_SLICE_SetShadowTransferMode(ADC0_ISR0_HW, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(ADC0_ISR0_HW, hw.mcu.isr0.period - 1U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(ADC0_ISR0_HW, hw.mcu.pwm.period >> 1);

    XMC_CCU8_SLICE_SetShadowTransferMode(ADC1_ISR0_HW, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(ADC1_ISR0_HW, (hw.mcu.isr0.period) - 1U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(ADC1_ISR0_HW, hw.mcu.pwm.period >> 1);

    XMC_CCU8_SLICE_SetShadowTransferMode(PWM_U_HW, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_U_HW, (hw.mcu.pwm.period >> 1) - 1U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_U_HW, hw.mcu.pwm.period >> 2);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_U_HW, hw.mcu.pwm.period >> 2);

	XMC_CCU8_SLICE_SetShadowTransferMode(PWM_V_HW, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_V_HW, (hw.mcu.pwm.period >> 1) - 1U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_V_HW, hw.mcu.pwm.period >> 2);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_V_HW, hw.mcu.pwm.period >> 2);

    XMC_CCU8_SLICE_SetShadowTransferMode(PWM_W_HW, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(PWM_W_HW, (hw.mcu.pwm.period >> 1) - 1U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(PWM_W_HW, hw.mcu.pwm.period >> 2);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(PWM_W_HW, hw.mcu.pwm.period >> 2);

    XMC_CCU8_SLICE_SetShadowTransferMode(SYNC_ISR1_HW, XMC_CCU8_SLICE_SHADOW_TRANSFER_MODE_ONLY_IN_PERIOD_MATCH);
    XMC_CCU8_SLICE_SetTimerPeriodMatch(SYNC_ISR1_HW, hw.mcu.isr1.period - 1U);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(SYNC_ISR1_HW, hw.mcu.isr1.period - 1U);

    XMC_CCU8_EnableShadowTransfer(PWM_UVW_ADCn_ISRn_HW, XMC_CCU8_SHADOW_TRANSFER_PWM_U_SLICE | XMC_CCU8_SHADOW_TRANSFER_PWM_V_SLICE | XMC_CCU8_SHADOW_TRANSFER_PWM_W_SLICE
                                  | XMC_CCU8_SHADOW_TRANSFER_ADC0_ISR0_SLICE | XMC_CCU8_SHADOW_TRANSFER_ADC1_ISR0_SLICE);
    XMC_CCU8_EnableShadowTransfer(SYNC_ISRn_HW, XMC_CCU8_SHADOW_TRANSFER_SYNC_ISR1_SLICE);

    sensor_iface.uvw_idx = &ctrl.volt_mod.uvw_idx_prev;

#endif    
}

void MCU_InitPosInterface()
{
#if defined(POSIF_ENC_HW)
#if defined(COMPONENT_CAT1)
    Cy_TCPWM_MOTIF_Enable(POSIF_ENC_HW);
    Cy_TCPWM_MOTIF_Quaddec_Init(POSIF_ENC_HW, &POSIF_ENC_quaddec_config);
    Cy_TCPWM_MOTIF_Quaddec_Output_Config(POSIF_ENC_HW, &POSIF_ENC_quaddec_output_config);
#elif defined(COMPONENT_CAT3)
    // Nothing to do
#endif
#endif
}

void MCU_InitWatchdog()
{
#if defined(CY_USING_HAL)
    // Start the watchdog now that we are running
    cyhal_wdt_init(&hw.mcu.wdt_obj, params.sys.faults.watchdog_time);
    cyhal_wdt_start(&hw.mcu.wdt_obj);

#endif
}

void MCU_StartPeripherals()
{
    MCU_EnterCriticalSection(); // No ISRs beyond this point
  
#if defined(COMPONENT_CAT1)
    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_dma_adc_0);
    Cy_DMA_Enable(DMA_ADC_0_HW);
    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
    Cy_DMA_Enable(DMA_ADC_1_HW);
#if defined(DMA_ADC_2_HW)
    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_dma_adc_2);
    Cy_DMA_Enable(DMA_ADC_2_HW);    
#endif    
    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_sync_isr1);

#elif defined(COMPONENT_CAT3)
    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_dma_adc_0);
    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
	NVIC_EnableIRQ(hw.mcu.interrupt.nvic_sync_isr1);

#endif

#if defined(COMPONENT_CAT1A)
    Cy_SysAnalog_Enable();
    Cy_SAR_Enable(ADC_0_HW);
    Cy_SAR_Enable(ADC_1_HW);
#elif defined(COMPONENT_CAT1C)
    Cy_SAR2_Enable(ADC_0_HW);
    Cy_SAR2_Enable(ADC_1_HW);
    Cy_SAR2_Enable(ADC_2_HW);    
#endif

#if defined(POSIF_ENC_HW)
#if defined(COMPONENT_CAT1)
    Cy_TCPWM_Counter_Enable(ENC_POS_CNTR_HW, ENC_POS_CNTR_NUM);
    Cy_TCPWM_TriggerStart_Single(ENC_POS_CNTR_HW, ENC_POS_CNTR_NUM);
    Cy_TCPWM_Counter_Enable(ENC_TIME_BTW_TICKS_HW, ENC_TIME_BTW_TICKS_NUM);
    Cy_TCPWM_MOTIF_Start(POSIF_ENC_HW); 
#elif defined(COMPONENT_CAT3)
    XMC_CCU4_SLICE_StartTimer(ENC_POS_CNTR_HW);
    XMC_CCU4_SLICE_StartTimer(ENC_TIME_BTW_TICKS_HW);
    XMC_POSIF_Start(POSIF_ENC_HW);    
#endif
#endif
    
    hw.mcu.isr1.count = 0U;
    hw.mcu.isr0.count = 0U;
#if defined(COMPONENT_CAT1)
    Cy_TCPWM_PWM_Enable(ADC0_ISR0_HW, ADC0_ISR0_NUM);
    Cy_TCPWM_PWM_Enable(ADC1_ISR0_HW, ADC1_ISR0_NUM);    
    Cy_TCPWM_PWM_Enable(PWM_SYNC_HW, PWM_SYNC_NUM);
    Cy_TCPWM_PWM_Enable(PWM_U_HW, PWM_U_NUM);
    Cy_TCPWM_PWM_Enable(PWM_V_HW, PWM_V_NUM);
    Cy_TCPWM_PWM_Enable(PWM_W_HW, PWM_W_NUM);   
    Cy_TCPWM_PWM_Enable(SYNC_ISR1_HW, SYNC_ISR1_NUM);
    MCU_EnableTimerReload();
    Cy_TCPWM_TriggerStart_Single(SYNC_ISR1_HW, SYNC_ISR1_NUM); // Start ISR1 which will also start U,V,W
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    Cy_TCPWM_Counter_Enable(HALL_TIMER_HW, HALL_TIMER_NUM);
    Cy_TCPWM_TriggerStart_Single(HALL_TIMER_HW, HALL_TIMER_NUM); 
#endif
    Cy_TCPWM_Counter_Enable(EXE_TIMER_HW, EXE_TIMER_NUM);
    Cy_TCPWM_TriggerStart_Single(EXE_TIMER_HW, EXE_TIMER_NUM);   

#elif defined(COMPONENT_CAT3)
	XMC_CCU8_SLICE_StartTimer(ADC0_ISR0_HW);
	XMC_CCU8_SLICE_StartTimer(ADC1_ISR0_HW);
	XMC_CCU8_SLICE_StartTimer(PWM_U_HW);
	XMC_CCU8_SLICE_StartTimer(PWM_V_HW);
	XMC_CCU8_SLICE_StartTimer(PWM_W_HW);
    XMC_CCU8_SLICE_StartTimer(SYNC_ISR1_HW);
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    XMC_CCU4_SLICE_StartTimer(HALL_TIMER_H_HW);
    XMC_CCU4_SLICE_StartTimer(HALL_TIMER_L_HW);
#endif
    XMC_CCU4_SLICE_StartTimer(EXE_TIMER_H_HW);
    XMC_CCU4_SLICE_StartTimer(EXE_TIMER_L_HW);

#endif
  
#if defined(N_HALL_EN_PORT) && (defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC))
    if (params.sys.fb.mode == Hall)
    { 
#if defined(COMPONENT_CAT1)
        Cy_GPIO_Clr(N_HALL_EN_PORT, N_HALL_EN_PIN);
#elif defined(COMPONENT_CAT3)
        XMC_GPIO_SetOutputLow(N_HALL_EN_PORT, N_HALL_EN_PIN);
#endif
    }
#endif

#if defined(ENC_EN_PORT) && (defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC))
    if (params.sys.fb.mode == AqB_Enc)
    { 
#if defined(COMPONENT_CAT1)
        Cy_GPIO_Set(ENC_EN_PORT, ENC_EN_PIN);
#elif defined(COMPONENT_CAT3)
        XMC_GPIO_SetOutputHigh(ENC_EN_PORT, ENC_EN_PIN);
#endif
    }
#endif
    
    MCU_ExitCriticalSection();
}

void MCU_StopPeripherals()
{
    MCU_EnterCriticalSection(); // No ISRs beyond this point

#if defined(ENC_EN_PORT) && (defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC))
    if (params.sys.fb.mode == AqB_Enc)
    { 
#if defined(COMPONENT_CAT1)
        Cy_GPIO_Clr(ENC_EN_PORT, ENC_EN_PIN);
#elif defined(COMPONENT_CAT3)
        XMC_GPIO_SetOutputLow(ENC_EN_PORT, ENC_EN_PIN);
#endif
    }
#endif
    
#if defined(N_HALL_EN_PORT) && (defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC))
    if (params.sys.fb.mode == Hall)
    { 
#if defined(COMPONENT_CAT1)
        Cy_GPIO_Set(N_HALL_EN_PORT, N_HALL_EN_PIN);
#elif defined(COMPONENT_CAT3)
        XMC_GPIO_SetOutputHigh(N_HALL_EN_PORT, N_HALL_EN_PIN);
#endif
    }
#endif

#if defined(COMPONENT_CAT1)
    Cy_TCPWM_Counter_Disable(EXE_TIMER_HW, EXE_TIMER_NUM);
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
    Cy_TCPWM_Counter_Disable(HALL_TIMER_HW, HALL_TIMER_NUM);
#endif
    Cy_TCPWM_PWM_Disable(SYNC_ISR1_HW, SYNC_ISR1_NUM);
    Cy_TCPWM_PWM_Disable(PWM_W_HW, PWM_W_NUM);
    Cy_TCPWM_PWM_Disable(PWM_V_HW, PWM_V_NUM);
    Cy_TCPWM_PWM_Disable(PWM_U_HW, PWM_U_NUM);
    Cy_TCPWM_PWM_Disable(PWM_SYNC_HW, PWM_SYNC_NUM);
    Cy_TCPWM_PWM_Disable(ADC1_ISR0_HW, ADC1_ISR0_NUM); 
    Cy_TCPWM_PWM_Disable(ADC0_ISR0_HW, ADC0_ISR0_NUM);

#elif defined(COMPONENT_CAT3)
	XMC_CCU4_SLICE_StopClearTimer(EXE_TIMER_L_HW);
	XMC_CCU4_SLICE_StopClearTimer(EXE_TIMER_H_HW);
#if defined(CTRL_METHOD_RFO) || defined(CTRL_METHOD_TBC)
	XMC_CCU4_SLICE_StopClearTimer(HALL_TIMER_L_HW);
	XMC_CCU4_SLICE_StopClearTimer(HALL_TIMER_H_HW);
#endif
	XMC_CCU8_SLICE_StopClearTimer(SYNC_ISR1_HW);
	XMC_CCU8_SLICE_StopClearTimer(PWM_W_HW);
	XMC_CCU8_SLICE_StopClearTimer(PWM_V_HW);
	XMC_CCU8_SLICE_StopClearTimer(PWM_U_HW);
	XMC_CCU8_SLICE_StopClearTimer(ADC1_ISR0_HW);
	XMC_CCU8_SLICE_StopClearTimer(ADC0_ISR0_HW);

#endif

#if defined(POSIF_ENC_HW)
#if defined(COMPONENT_CAT1)
    Cy_TCPWM_MOTIF_Stop(POSIF_ENC_HW); 
    Cy_TCPWM_Counter_Disable(ENC_TIME_BTW_TICKS_HW, ENC_TIME_BTW_TICKS_NUM);
    Cy_TCPWM_Counter_Disable(ENC_POS_CNTR_HW, ENC_POS_CNTR_NUM);
#elif defined(COMPONENT_CAT3)
    XMC_POSIF_Stop(POSIF_ENC_HW);    
    XMC_CCU4_SLICE_StopClearTimer(ENC_TIME_BTW_TICKS_HW);
    XMC_CCU4_SLICE_StopClearTimer(ENC_POS_CNTR_HW);
#endif
#endif
    
#if defined(COMPONENT_CAT1A)
    Cy_SAR_Disable(ADC_1_HW);
    Cy_SAR_Disable(ADC_0_HW);
    Cy_SysAnalog_Disable();
#elif defined(COMPONENT_CAT1C)
	Cy_SAR2_Disable(ADC_2_HW);
    Cy_SAR2_Disable(ADC_1_HW);
    Cy_SAR2_Disable(ADC_0_HW);
#endif

#if defined(COMPONENT_CAT1)
    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_sync_isr1);
#if defined(DMA_ADC_2_HW)
    Cy_DMA_Disable(DMA_ADC_2_HW);
    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_dma_adc_2);    
#endif 
    Cy_DMA_Disable(DMA_ADC_1_HW);
    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
    Cy_DMA_Disable(DMA_ADC_0_HW);
    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_dma_adc_0);

#elif defined(COMPONENT_CAT3)
	NVIC_DisableIRQ(hw.mcu.interrupt.nvic_sync_isr1);
    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_dma_adc_0);

#endif
    
    MCU_ExitCriticalSection();
}

void MCU_FlashInit()
{
#if defined(COMPONENT_CAT1)
	// EEPROM Emulator
    hw.mcu.eeprom.config.eepromSize = srss_0_eeprom_0_SIZE,
    hw.mcu.eeprom.config.simpleMode = srss_0_eeprom_0_SIMPLEMODE,
    hw.mcu.eeprom.config.wearLevelingFactor = srss_0_eeprom_0_WEARLEVELING_FACTOR,
    hw.mcu.eeprom.config.redundantCopy = srss_0_eeprom_0_REDUNDANT_COPY,
    hw.mcu.eeprom.config.blockingWrite = srss_0_eeprom_0_BLOCKINGMODE,
    hw.mcu.eeprom.config.userFlashStartAddr = (uint32_t)&(Em_Eeprom_Storage[0U]),

    hw.mcu.eeprom.status = Cy_Em_EEPROM_Init(&hw.mcu.eeprom.config, &hw.mcu.eeprom.context);

#elif defined(COMPONENT_CAT3)
   	hw.mcu.eeprom.status = E_EEPROM_XMC4_Init(&hw.mcu.eeprom.handle, scu_0_eeprom_0_EEPROM_SIZE);

#endif

   	hw.mcu.eeprom.init_done = true;
}

bool MCU_FlashWriteParams(PARAMS_t* ram_data)
{

	if(!hw.mcu.eeprom.init_done)
	{
		MCU_FlashInit();
	}
#if defined(COMPONENT_CAT1)
	if (CY_EM_EEPROM_SUCCESS != hw.mcu.eeprom.status)
	{
		return false;
	}
#elif defined(COMPONENT_CAT3)
	if (E_EEPROM_XMC4_STATUS_OK != hw.mcu.eeprom.status)
	{
		return false;
	}
#endif

#if defined(COMPONENT_CAT1)
	hw.mcu.eeprom.status = Cy_Em_EEPROM_Write(0U, ram_data, sizeof(PARAMS_t), &hw.mcu.eeprom.context);
	if (CY_EM_EEPROM_SUCCESS != hw.mcu.eeprom.status)
	{
	    return false;
	}

#elif defined(COMPONENT_CAT3)
    (void)E_EEPROM_XMC4_WriteArray(0U, (uint8_t*)ram_data, sizeof(PARAMS_t));
    E_EEPROM_XMC4_UpdateFlashContents();

#endif
    return true;
}

bool MCU_FlashReadParams(PARAMS_ID_t id, PARAMS_t* ram_data)
{
	if(!hw.mcu.eeprom.init_done)
	{
		MCU_FlashInit();
	}
#if defined(COMPONENT_CAT1)
	if (CY_EM_EEPROM_SUCCESS != hw.mcu.eeprom.status)
	{
		return false;
	}
#elif defined(COMPONENT_CAT3)
	if (E_EEPROM_XMC4_STATUS_OK != hw.mcu.eeprom.status)
	{
		return false;
	}
#endif

#if defined(COMPONENT_CAT1)
	hw.mcu.eeprom.status = Cy_Em_EEPROM_Read(0u, ram_data, sizeof(PARAMS_t), &hw.mcu.eeprom.context);
	if (CY_EM_EEPROM_SUCCESS != hw.mcu.eeprom.status)
	{
	    return false;
	}

#elif defined(COMPONENT_CAT3)
    E_EEPROM_XMC4_ReadArray(0, (uint8_t*)ram_data, sizeof(PARAMS_t));

#endif

    if (ram_data->id.code != id.code || ram_data->id.build_config != id.build_config || ram_data->id.ver != id.ver)
    {
        return false;
    }
    return true;
}

RAMFUNC_BEGIN
bool MCU_ArePhaseVoltagesMeasured()
{
  return hw.mcu.adc_mux.en;
}
RAMFUNC_END

