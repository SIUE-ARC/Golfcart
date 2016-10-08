/*******************************************************************************
* File Name: POTADC_PM.c
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

#include "POTADC.h"


/***************************************
* Local data allocation
***************************************/

static POTADC_BACKUP_STRUCT  POTADC_backup =
{
    POTADC_DISABLED
};


/*******************************************************************************
* Function Name: POTADC_SaveConfig
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
void POTADC_SaveConfig(void)
{
    /* All configuration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: POTADC_RestoreConfig
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
void POTADC_RestoreConfig(void)
{
    /* All congiguration registers are marked as [reset_all_retention] */
}


/*******************************************************************************
* Function Name: POTADC_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred routine to prepare the component for sleep.
*  The POTADC_Sleep() routine saves the current component state,
*  then it calls the ADC_Stop() function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  POTADC_backup - The structure field 'enableState' is modified
*  depending on the enable state of the block before entering to sleep mode.
*
*******************************************************************************/
void POTADC_Sleep(void)
{
    if((POTADC_PWRMGR_SAR_REG  & POTADC_ACT_PWR_SAR_EN) != 0u)
    {
        if((POTADC_SAR_CSR0_REG & POTADC_SAR_SOF_START_CONV) != 0u)
        {
            POTADC_backup.enableState = POTADC_ENABLED | POTADC_STARTED;
        }
        else
        {
            POTADC_backup.enableState = POTADC_ENABLED;
        }
        POTADC_Stop();
    }
    else
    {
        POTADC_backup.enableState = POTADC_DISABLED;
    }
}


/*******************************************************************************
* Function Name: POTADC_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred routine to restore the component to the state when
*  POTADC_Sleep() was called. If the component was enabled before the
*  POTADC_Sleep() function was called, the
*  POTADC_Wakeup() function also re-enables the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  POTADC_backup - The structure field 'enableState' is used to
*  restore the enable state of block after wakeup from sleep mode.
*
*******************************************************************************/
void POTADC_Wakeup(void)
{
    if(POTADC_backup.enableState != POTADC_DISABLED)
    {
        POTADC_Enable();
        #if(POTADC_DEFAULT_CONV_MODE != POTADC__HARDWARE_TRIGGER)
            if((POTADC_backup.enableState & POTADC_STARTED) != 0u)
            {
                POTADC_StartConvert();
            }
        #endif /* End POTADC_DEFAULT_CONV_MODE != POTADC__HARDWARE_TRIGGER */
    }
}


/* [] END OF FILE */
