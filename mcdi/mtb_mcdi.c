/*******************************************************************************
* \file mtb_mcdi.c
* \version 1.0
*
* \brief
* Provides API implementation for the MCDI library.
*
********************************************************************************
* \copyright
* (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation. All rights reserved.
********************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation or one of its
* affiliates ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "mtb_mcdi.h"

#ifdef CY_IP_MXS40MCPASS

cy_rslt_t mtb_mcdi_init(mtb_stc_mcdi_cfg_t const * cfg)
{
    cy_rslt_t status = (cy_rslt_t)(cy_en_mcdi_status_t)CY_MCDI_SUCCESS;

    if ((cfg->topo == MTB_MCDI_3SHUNT) || (cfg->topo == MTB_MCDI_1SHUNT))
    {
        for (uint32_t i = 0UL; i < CY_MCDI_PWM_NUM; i++)
        {
            status |=
                (cy_rslt_t)Cy_TCPWM_PWM_Init(cfg->tcpwmBase, cfg->pwm[i].idx, cfg->pwm[i].cfg);
        }

        for (uint32_t i = 0UL; i < CY_MCDI_TMR_NUM; i++)
        {
            status |=
                (cy_rslt_t)Cy_TCPWM_Counter_Init(cfg->tcpwmBase, cfg->tmr[i].idx, cfg->tmr[i].cfg);
        }
    }
    else
    {
        status = (cy_rslt_t)(cy_en_mcdi_status_t)CY_MCDI_BAD_PARAM;
    }

    return status;
}


cy_rslt_t mtb_mcdi_enable(mtb_stc_mcdi_cfg_t const * cfg)
{
    for (uint32_t i = 0UL; i < CY_MCDI_PWM_NUM; i++)
    {
        Cy_TCPWM_Enable_Single(cfg->tcpwmBase, cfg->pwm[i].idx);
    }

    for (uint32_t i = 0UL; i < CY_MCDI_TMR_NUM; i++)
    {
        Cy_TCPWM_Enable_Single(cfg->tcpwmBase, cfg->tmr[i].idx);
    }

    if (NULL != cfg->fault)
    {
        Cy_GPIO_SetInterruptMask(cfg->fault->base, cfg->fault->pinNum, 1UL);
    }

    Cy_TCPWM_SetInterruptMask(cfg->tcpwmBase, cfg->tmr[MTB_MCDI_TMR_SLOW].idx, cfg->slowIntrMsk);

    if (MTB_MCDI_3SHUNT == cfg->topo)
    {
        Cy_HPPASS_SAR_Result_SetInterruptMask(
            cfg->fastIntrMsk | Cy_HPPASS_SAR_Result_GetInterruptMask());
    }
    else /* MTB_MCDI_1SHUNT */
    {
        Cy_TCPWM_SetInterruptMask(cfg->tcpwmBase,
                                  cfg->tmr[MTB_MCDI_TMR_FAST].idx,
                                  cfg->fastIntrMsk);
    }

    return CY_RSLT_SUCCESS; /* For future capability */
}


cy_rslt_t mtb_mcdi_disable(mtb_stc_mcdi_cfg_t const * cfg)
{
    if (MTB_MCDI_3SHUNT == cfg->topo)
    {
        Cy_HPPASS_SAR_Result_SetInterruptMask(
            ~cfg->fastIntrMsk & Cy_HPPASS_SAR_Result_GetInterruptMask());
    }
    else
    {
        Cy_TCPWM_SetInterruptMask(cfg->tcpwmBase, cfg->tmr[MTB_MCDI_TMR_FAST].idx, 0UL);
    }

    Cy_TCPWM_SetInterruptMask(cfg->tcpwmBase, cfg->tmr[MTB_MCDI_TMR_SLOW].idx, 0UL);

    if (NULL != cfg->fault)
    {
        Cy_GPIO_SetInterruptMask(cfg->fault->base, cfg->fault->pinNum, 0UL);
    }

    for (uint32_t i = 0UL; i < CY_MCDI_PWM_NUM; i++)
    {
        Cy_TCPWM_Disable_Single(cfg->tcpwmBase, cfg->pwm[i].idx);
    }

    for (uint32_t i = 0UL; i < CY_MCDI_TMR_NUM; i++)
    {
        Cy_TCPWM_Disable_Single(cfg->tcpwmBase, cfg->tmr[i].idx);
    }

    return CY_RSLT_SUCCESS; /* For future capability */
}


cy_rslt_t mtb_mcdi_start(mtb_stc_mcdi_cfg_t const * cfg)
{
    cy_rslt_t status = (cy_rslt_t)Cy_TrigMux_SwTrigger(cfg->syncStartTrig, CY_TRIGGER_TWO_CYCLES);

    if (CY_RSLT_SUCCESS == status)
    {
        Cy_TCPWM_TriggerStart_Single(cfg->tcpwmBase, cfg->tmr[MTB_MCDI_TMR_SLOW].idx);
    }

    return status;
}


#endif /* CY_IP_MXS40MCPASS */

/* [] END OF FILE */
