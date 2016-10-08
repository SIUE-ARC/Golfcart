/*******************************************************************************
* File Name: POTADC_SAR_PM.c
* Version 3.0
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "POTADC_SAR.h"


/***************************************
* Local data allocation
***************************************/

static POTADC_SAR_BACKUP_STRUCT  POTADC_SAR_backup =
{
    POTADC_SAR_DISABLED
};


/*******************************************************************************
* Function Name: POTADC_SAR_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void POTADC_SAR_SaveConfig(void)
{
    /* All configuration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: POTADC_SAR_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void POTADC_SAR_RestoreConfig(void)
{
    /* All congiguration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: POTADC_SAR_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred routine to prepare the component for sleep.
*  The POTADC_SAR_Sleep() routine saves the current component state,
*  then it calls the ADC_Stop() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  POTADC_SAR_backup - The structure field 'enableState' is modified
*  depending on the enable state of the block before entering to sleep mode.
*
*******************************************************************************/
void POTADC_SAR_Sleep(void)
{
    if((POTADC_SAR_PWRMGR_SAR_REG  & POTADC_SAR_ACT_PWR_SAR_EN) != 0u)
    {
        if((POTADC_SAR_SAR_CSR0_REG & POTADC_SAR_SAR_SOF_START_CONV) != 0u)
        {
            POTADC_SAR_backup.enableState = POTADC_SAR_ENABLED | POTADC_SAR_STARTED;
        }
        else
        {
            POTADC_SAR_backup.enableState = POTADC_SAR_ENABLED;
        }
        POTADC_SAR_Stop();
    }
    else
    {
        POTADC_SAR_backup.enableState = POTADC_SAR_DISABLED;
    }
}


/*******************************************************************************
* Function Name: POTADC_SAR_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred routine to restore the component to the state when
*  POTADC_SAR_Sleep() was called. If the component was enabled before the
*  POTADC_SAR_Sleep() function was called, the
*  POTADC_SAR_Wakeup() function also re-enables the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  POTADC_SAR_backup - The structure field 'enableState' is used to
*  restore the enable state of block after wakeup from sleep mode.
*
*******************************************************************************/
void POTADC_SAR_Wakeup(void)
{
    if(POTADC_SAR_backup.enableState != POTADC_SAR_DISABLED)
    {
        POTADC_SAR_Enable();
        #if(POTADC_SAR_DEFAULT_CONV_MODE != POTADC_SAR__HARDWARE_TRIGGER)
            if((POTADC_SAR_backup.enableState & POTADC_SAR_STARTED) != 0u)
            {
                POTADC_SAR_StartConvert();
            }
        #endif /* End POTADC_SAR_DEFAULT_CONV_MODE != POTADC_SAR__HARDWARE_TRIGGER */
    }
}


/* [] END OF FILE */
