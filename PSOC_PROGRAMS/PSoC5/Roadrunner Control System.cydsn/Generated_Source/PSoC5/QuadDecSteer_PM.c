/*******************************************************************************
* File Name: QuadDecSteer_PM.c
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

#include "QuadDecSteer.h"

static QuadDecSteer_BACKUP_STRUCT QuadDecSteer_backup = {0u};


/*******************************************************************************
* Function Name: QuadDecSteer_SaveConfig
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
void QuadDecSteer_SaveConfig(void) 
{
    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
        QuadDecSteer_Cnt8_SaveConfig();
    #else 
        /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) || 
         * (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
         */
        QuadDecSteer_Cnt16_SaveConfig();
    #endif /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT) */
}


/*******************************************************************************
* Function Name: QuadDecSteer_RestoreConfig
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
void QuadDecSteer_RestoreConfig(void) 
{
    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
        QuadDecSteer_Cnt8_RestoreConfig();
    #else 
        /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) || 
         * (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT) 
         */
        QuadDecSteer_Cnt16_RestoreConfig();
    #endif /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT) */
}


/*******************************************************************************
* Function Name: QuadDecSteer_Sleep
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
*  QuadDecSteer_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void QuadDecSteer_Sleep(void) 
{
    if (0u != (QuadDecSteer_SR_AUX_CONTROL & QuadDecSteer_INTERRUPTS_ENABLE))
    {
        QuadDecSteer_backup.enableState = 1u;
    }
    else /* The Quadrature Decoder Component is disabled */
    {
        QuadDecSteer_backup.enableState = 0u;
    }

    QuadDecSteer_Stop();
    QuadDecSteer_SaveConfig();
}


/*******************************************************************************
* Function Name: QuadDecSteer_Wakeup
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
*  QuadDecSteer_backup - used when non-retention registers are restored.
*
*******************************************************************************/
void QuadDecSteer_Wakeup(void) 
{
    QuadDecSteer_RestoreConfig();

    if (QuadDecSteer_backup.enableState != 0u)
    {
        #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
            QuadDecSteer_Cnt8_Enable();
        #else 
            /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) || 
            *  (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT) 
            */
            QuadDecSteer_Cnt16_Enable();
        #endif /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT) */

        /* Enable component's operation */
        QuadDecSteer_Enable();
    } /* Do nothing if component's block was disabled before */
}


/* [] END OF FILE */

