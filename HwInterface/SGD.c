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


#if defined(USING_SGD) // smart gate driver

#include "HardwareIface.h"
#include "Controller.h"

#define SPI_WRITE       0x80
#define SPI_READ        0x00

#define READ_REG(poll, index, reg)          poll[(index)*3] = (reg) | SPI_READ; \
                                            poll[(index)*3+1] = 0x00; \
                                            poll[(index)*3+2] = 0x00

#define WRITE_REG(poll, index, reg, val)    poll[(index)*3] = (reg) | SPI_WRITE; \
                                            poll[(index)*3+1] = ((val)>>8); \
                                            poll[(index)*3+2] = (val)&0xFF

void SGD_WriteReg(uint8_t reg, uint16_t val) {
    uint8_t buff[3];

    buff[0] = reg | SPI_WRITE;
    buff[1] = val >> 8;
    buff[2] = val & 0xff;

    // Transmit and wait for for completion
    Cy_SCB_SPI_WriteArrayBlocking(SPI_SGD_HW, buff, 3);
    while (Cy_SCB_GetNumInRxFifo(SPI_SGD_HW) != 3);
    Cy_SCB_ReadArrayNoCheck(SPI_SGD_HW, buff, 3);

    // Read the value back into the gui register table
    hw.sgd.register_table[reg] = SGD_ReadReg(reg);
}

uint16_t SGD_ReadReg(uint8_t reg) {
    uint8_t buff[3];

    buff[0] = reg | SPI_READ;
    buff[1] = 0;
    buff[2] = 0;

    // Transmit and wait for for completion
    Cy_SCB_SPI_WriteArrayBlocking(SPI_SGD_HW, buff, 3);
    while (Cy_SCB_GetNumInRxFifo(SPI_SGD_HW) != 3);
    Cy_SCB_ReadArrayNoCheck(SPI_SGD_HW, buff, 3);

    // Swap endianess and return
    buff[0] = buff[2];
    return *((uint16_t*) buff);
}

void SGD_Init(void) {
    // Initialize poll command array
    // Reads
    READ_REG(hw.sgd.poll_cmd, 0, ADDR_FAULTS_ST);
    READ_REG(hw.sgd.poll_cmd, 1, ADDR_TEMP_ST);
    READ_REG(hw.sgd.poll_cmd, 2, ADDR_SUPPLY_ST);
    READ_REG(hw.sgd.poll_cmd, 3, ADDR_FUNCT_ST);
    READ_REG(hw.sgd.poll_cmd, 4, ADDR_OTP_ST);
    READ_REG(hw.sgd.poll_cmd, 5, ADDR_ADC_ST);
    READ_REG(hw.sgd.poll_cmd, 6, ADDR_CP_ST);

    // Writes
    WRITE_REG(hw.sgd.poll_cmd, 7, ADDR_FAULTS_CLR, 0);

    // Place holder for gui
    hw.sgd.write_register = SKIP_GUI_POLL;
    READ_REG(hw.sgd.poll_cmd, 8, ADDR_FAULTS_ST);
    READ_REG(hw.sgd.poll_cmd, 9, ADDR_FAULTS_ST);

    hw.sgd.poll_size = 10;
    CY_ASSERT(hw.sgd.poll_size < SGD_MAX_POLL_SIZE);

    cy_stc_scb_spi_context_t context;

    // Initialize the SPI
    Cy_SCB_SPI_Init(SPI_SGD_HW, &SPI_SGD_config, &context);
    Cy_SCB_SPI_SetActiveSlaveSelect(SPI_SGD_HW, CY_SCB_SPI_SLAVE_SELECT0);
    Cy_SCB_SPI_Enable(SPI_SGD_HW);

    // Check the chip version
    CY_ASSERT(SGD_ReadReg(ADDR_DEVICE_ID) == 0x0006);

    // Write default configuration
    // Dvdd oc threshold = 450mA
    SGD_WriteReg(ADDR_SUPPLY_CFG, PVCC_SETPT_12V | CS_REF_CFG_1_2 | DVDD_OCP_THR_450 | DVDD_SFTSTRT_100us
                    | DVDD_SETPT_3_3 | BK_FREQ_1M | DVDD_TON_DELAY_200us | CP_PRECHARGE_DISABLE);

    SGD_WriteReg(ADDR_ADC_CFG, ADC_OD_REQ_NO_ACTION | ADC_OD_IN_SEL_IDIGITAL | ADC_EN_FILT_ENABLE
                    | ADC_FILT_CFG_8_SAMPLES | ADC_FILT_CFG_PVDD_32_SAMPLES);
    // HW brake mode set to high z
    SGD_WriteReg(ADDR_PWM_CFG, PWM_MODE_6 | PWM_FREEW_CFG_ACTIVE | BRAKE_CFG_HIGH_Z | PWM_RECIRC_DISABLE);
    // OT shut down enabled
    const uint16_t Shunt_Amp_Timing_Mode = (params.sys.analog.shunt.type == Three_Shunt) ? CS_TMODE_GLxHIGH : CS_TMODE_ALWAYS;
    SGD_WriteReg(ADDR_SENSOR_CFG, HALL_DEGLITCH_640ns | OTS_DIS_PROTECTION_ENABLE | Shunt_Amp_Timing_Mode);
    // WD fault disabled
    SGD_WriteReg(ADDR_WD_CFG, WD_EN_DISABLE | WD_INSEL_EN_DRV | WD_FLTCFG_REGISTER | WD_TIMER_T_100us);
    // WD brake disabled, locked rotor fault disabled
    SGD_WriteReg(ADDR_WD_CFG2, WD_BRAKE_NORMAL | WD_EN_LATCH_DISABLED | WD_DVDD_RSTR_ATT_0
                    | WD_DVDD_RSTRT_DLY_0ms5 | WD_RLOCK_EN_DISABLED | WD_RLOCK_T_1s | WD_BK_DIS_DISABLED);

    SGD_WriteReg(ADDR_IDRIVE_CFG, IHS_SRC_500mA | IHS_SINK_500mA | ILS_SRC_500mA | ILS_SINK_500mA);
    SGD_WriteReg(ADDR_IDRIVE_PRE_CFG, I_PRE_SRC_500mA | I_PRE_SINK_500mA | I_PRE_EN_ENABLED);
    SGD_WriteReg(ADDR_TDRIVE_SRC_CFG, TDRIVE1_80ns | TDRIVE2_100ns);
    SGD_WriteReg(ADDR_TDRIVE_SINK_CFG, TDRIVE3_200ns | TDRIVE4_100ns);

    SGD_WriteReg(ADDR_DT_CFG, DT_RISE_120ns | DT_FALL_120ns);

    SGD_WriteReg(ADDR_CP_CFG, CP_CLK_CFG_781_25kHz | CP_CLK_SS_DIS_ENABLED);
    // CS_OCP fault enabled, 8us deglitch time:
    const uint16_t CS_Gain = (uint16_t)(params.sys.analog.shunt.opamp_gain) << 0U;
    SGD_WriteReg(ADDR_CSAMP_CFG, CS_Gain | CS_EN_PHASE_A | CS_EN_PHASE_B | CS_EN_PHASE_C
                    | CS_BLANK_500ns | CS_EN_DCCAL_DISABLED | CS_OCP_DEGLITCH_8us | CS_OCPFLT_CFG_ALL); // set
    // CS_OCP fault braking disabled, threshold = +/-300A, latched
    SGD_WriteReg(ADDR_CSAMP_CFG2, CS_OCP_PTHR_300mV | CS_OCP_NTHR_300mV | CS_OCP_LATCH_ENABLE
                    | CS_MODE_SHUNT | CS_OCP_BRAKE_DISABLE | CS_TRUNC_DIS_DISABLE | VREF_INSEL_INTERNAL
                    | CS_NEG_OCP_DIS_ENABLE | CS_AZ_CFG_INTERNAL); // set, +/-300A fault threshold
    SGD_WriteReg(ADDR_OTP_PROG, OTP_PROG_DISABLE);

    // Disable brake and enable the now configured gate driver
    Cy_GPIO_Set(N_BRK_SGD_PORT, N_BRK_SGD_NUM);
    Cy_GPIO_Set(EN_DRV_SGD_PORT, EN_DRV_SGD_NUM);

    // Configure SPI DMA RX
    Cy_DMA_Descriptor_Init(&DMA_SGD_RX_Descriptor_0, &DMA_SGD_RX_Descriptor_0_config);
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_SGD_RX_Descriptor_0, (void*)&SPI_SGD_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&DMA_SGD_RX_Descriptor_0, hw.sgd.poll_res);
    Cy_DMA_Descriptor_SetYloopDataCount(&DMA_SGD_RX_Descriptor_0, hw.sgd.poll_size);
    Cy_DMA_Descriptor_SetNextDescriptor(&DMA_SGD_RX_Descriptor_0, &DMA_SGD_RX_Descriptor_0);
    Cy_DMA_Channel_Init(DMA_SGD_RX_HW, DMA_SGD_RX_CHANNEL, &DMA_SGD_RX_channelConfig);
    Cy_DMA_Channel_SetDescriptor(DMA_SGD_RX_HW, DMA_SGD_RX_CHANNEL, &DMA_SGD_RX_Descriptor_0);
    Cy_DMA_Channel_Enable(DMA_SGD_RX_HW, DMA_SGD_RX_CHANNEL);
    Cy_DMA_Enable(DMA_SGD_RX_HW);

    // Configure SPI DMA TX - Enabling will initiate the first poll
    Cy_DMA_Descriptor_Init(&DMA_SGD_TX_Descriptor_0, &DMA_SGD_TX_Descriptor_0_config);
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_SGD_TX_Descriptor_0, hw.sgd.poll_cmd);
    Cy_DMA_Descriptor_SetDstAddress(&DMA_SGD_TX_Descriptor_0, (void*)&SPI_SGD_HW->TX_FIFO_WR);
    Cy_DMA_Descriptor_SetYloopDataCount(&DMA_SGD_TX_Descriptor_0, hw.sgd.poll_size);
    Cy_DMA_Descriptor_SetNextDescriptor(&DMA_SGD_TX_Descriptor_0, &DMA_SGD_TX_Descriptor_0);
    Cy_DMA_Channel_Init(DMA_SGD_TX_HW, DMA_SGD_TX_CHANNEL, &DMA_SGD_TX_channelConfig);
    Cy_DMA_Channel_SetDescriptor(DMA_SGD_TX_HW, DMA_SGD_TX_CHANNEL, &DMA_SGD_TX_Descriptor_0);
    Cy_DMA_Channel_Enable(DMA_SGD_TX_HW, DMA_SGD_TX_CHANNEL);
    Cy_DMA_Enable(DMA_SGD_TX_HW);

    // Configure the SPI transaction timer
    Cy_TCPWM_PWM_Init(SPI_TRANSACTIONS_HW, SPI_TRANSACTIONS_NUM, &SPI_TRANSACTIONS_config);
    Cy_TCPWM_PWM_Enable(SPI_TRANSACTIONS_HW, SPI_TRANSACTIONS_NUM);
    Cy_TCPWM_TriggerStart_Single(SPI_TRANSACTIONS_HW, SPI_TRANSACTIONS_NUM);
}


void SGD_RunISR1()
{
    int32_t index;

    // Fill the table with the status registers
    for (index=0; index < 7 ;index++)
        hw.sgd.register_table[index] = GET_POLL_RES(hw.sgd.poll_res, index);

    // Handle fault reading
    faults.flags.hw.reg = hw.sgd.register_table[0]; // ADDR_FAULTS_ST
    if(sm.vars.fault.clr_request)
    {
        WRITE_REG(hw.sgd.poll_cmd, 7, ADDR_FAULTS_CLR, CLR_FLTS | CLR_LATCH);
    }
    else
    {
        WRITE_REG(hw.sgd.poll_cmd, 7, ADDR_FAULTS_CLR, 0);
    }

    // Handle the GUI requests
    if (hw.sgd.write_register != SKIP_GUI_POLL)
    {
        if (hw.sgd.poll_cmd[8*3] == (ADDR_FAULTS_ST | SPI_READ))
        {
            // Add outgoing write and read
            WRITE_REG(hw.sgd.poll_cmd, 8, hw.sgd.write_register, hw.sgd.write_data);
            READ_REG(hw.sgd.poll_cmd, 9, hw.sgd.write_register);
            Cy_GPIO_Clr(EN_DRV_SGD_PORT, EN_DRV_SGD_NUM); //TBD: FW must be in Init state for GUI to be able to write
        } else {
            // Handle response incoming
            hw.sgd.register_table[hw.sgd.write_register] = GET_POLL_RES(hw.sgd.poll_res, 9);
            hw.sgd.write_register = SKIP_GUI_POLL;
            READ_REG(hw.sgd.poll_cmd, 8, ADDR_FAULTS_ST);
            Cy_GPIO_Set(EN_DRV_SGD_PORT, EN_DRV_SGD_NUM); //TBD: FW must be in Init state for GUI to be able to write
        }
    }

    // Then kick off next poll
    Cy_DMA_Channel_Enable(DMA_SGD_TX_HW, DMA_SGD_TX_CHANNEL);
}

#endif

