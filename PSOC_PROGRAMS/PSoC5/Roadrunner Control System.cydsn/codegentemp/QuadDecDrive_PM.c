/*******************************************************************************
* File Name: QuadDecDrive_PM.c
* Version 3.0
*
* Description:
*  This file contains the setup, control and status commands to support 
*  component operations in low power mode.  
*
* Note:
*  None.
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "QuadDecDrive.h"

static QuadDecDrive_BACKUP_STRUCT QuadDecDrive_backup = {0u};


/*******************************************************************************
* Function Name: QuadDecDrive_SaveConfig
********************************************************************************
* Summary:
*  Saves the current user configuration of the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecDrive_SaveConfig(void) 
{
    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
        QuadDecDrive_Cnt8_SaveConfig();
    #else 
        /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) || 
         * (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
         */
        QuadDecDrive_Cnt16_SaveConfig();
    #endif /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT) */
}


/*******************************************************************************
* Function Name: QuadDecDrive_RestoreConfig
********************************************************************************
* Summary:
*  Restores the current user configuration of the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecDrive_RestoreConfig(void) 
{
    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
        QuadDecDrive_Cnt8_RestoreConfig();
    #else 
        /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) || 
         * (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT) 
         */
        QuadDecDrive_Cnt16_RestoreConfig();
    #endif /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT) */
}


/*******************************************************************************
* Function Name: QuadDecDrive_Sleep
********************************************************************************
* 
* Summary:
*  Prepare Quadrature Decoder Component goes to sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  QuadDecDrive_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void QuadDecDrive_Sleep(void) 
{
    if (0u != (QuadDecDrive_SR_AUX_CONTROL & QuadDecDrive_INTERRUPTS_ENABLE))
    {
        QuadDecDrive_backup.enableState = 1u;
    }
    else /* The Quadrature Decoder Component is disabled */
    {
        QuadDecDrive_backup.enableState = 0u;
    }

    QuadDecDrive_Stop();
    QuadDecDrive_SaveConfig();
}


/*******************************************************************************
* Function Name: QuadDecDrive_Wakeup
********************************************************************************
*
* Summary:
*  Prepare Quadrature Decoder Component to wake up.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  QuadDecDrive_backup - used when non-retention registers are restored.
*
*******************************************************************************/
void QuadDecDrive_Wakeup(void) 
{
    QuadDecDrive_RestoreConfig();

    if (QuadDecDrive_backup.enableState != 0u)
    {
        #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
            QuadDecDrive_Cnt8_Enable();
        #else 
            /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) || 
            *  (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT) 
            */
            QuadDecDrive_Cnt16_Enable();
        #endif /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT) */

        /* Enable component's operation */
        QuadDecDrive_Enable();
    } /* Do nothing if component's block was disabled before */
}


/* [] END OF FILE */

