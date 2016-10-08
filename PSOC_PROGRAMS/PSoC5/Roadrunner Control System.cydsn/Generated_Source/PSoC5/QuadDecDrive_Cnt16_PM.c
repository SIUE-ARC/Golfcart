/*******************************************************************************
* File Name: QuadDecDrive_Cnt16_PM.c  
* Version 3.0
*
*  Description:
*    This file provides the power management source code to API for the
*    Counter.  
*
*   Note:
*     None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "QuadDecDrive_Cnt16.h"

static QuadDecDrive_Cnt16_backupStruct QuadDecDrive_Cnt16_backup;


/*******************************************************************************
* Function Name: QuadDecDrive_Cnt16_SaveConfig
********************************************************************************
* Summary:
*     Save the current user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  QuadDecDrive_Cnt16_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
*******************************************************************************/
void QuadDecDrive_Cnt16_SaveConfig(void) 
{
    #if (!QuadDecDrive_Cnt16_UsingFixedFunction)

        QuadDecDrive_Cnt16_backup.CounterUdb = QuadDecDrive_Cnt16_ReadCounter();

        #if(!QuadDecDrive_Cnt16_ControlRegRemoved)
            QuadDecDrive_Cnt16_backup.CounterControlRegister = QuadDecDrive_Cnt16_ReadControlRegister();
        #endif /* (!QuadDecDrive_Cnt16_ControlRegRemoved) */

    #endif /* (!QuadDecDrive_Cnt16_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: QuadDecDrive_Cnt16_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  QuadDecDrive_Cnt16_backup:  Variables of this global structure are used to 
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void QuadDecDrive_Cnt16_RestoreConfig(void) 
{      
    #if (!QuadDecDrive_Cnt16_UsingFixedFunction)

       QuadDecDrive_Cnt16_WriteCounter(QuadDecDrive_Cnt16_backup.CounterUdb);

        #if(!QuadDecDrive_Cnt16_ControlRegRemoved)
            QuadDecDrive_Cnt16_WriteControlRegister(QuadDecDrive_Cnt16_backup.CounterControlRegister);
        #endif /* (!QuadDecDrive_Cnt16_ControlRegRemoved) */

    #endif /* (!QuadDecDrive_Cnt16_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: QuadDecDrive_Cnt16_Sleep
********************************************************************************
* Summary:
*     Stop and Save the user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  QuadDecDrive_Cnt16_backup.enableState:  Is modified depending on the enable 
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void QuadDecDrive_Cnt16_Sleep(void) 
{
    #if(!QuadDecDrive_Cnt16_ControlRegRemoved)
        /* Save Counter's enable state */
        if(QuadDecDrive_Cnt16_CTRL_ENABLE == (QuadDecDrive_Cnt16_CONTROL & QuadDecDrive_Cnt16_CTRL_ENABLE))
        {
            /* Counter is enabled */
            QuadDecDrive_Cnt16_backup.CounterEnableState = 1u;
        }
        else
        {
            /* Counter is disabled */
            QuadDecDrive_Cnt16_backup.CounterEnableState = 0u;
        }
    #else
        QuadDecDrive_Cnt16_backup.CounterEnableState = 1u;
        if(QuadDecDrive_Cnt16_backup.CounterEnableState != 0u)
        {
            QuadDecDrive_Cnt16_backup.CounterEnableState = 0u;
        }
    #endif /* (!QuadDecDrive_Cnt16_ControlRegRemoved) */
    
    QuadDecDrive_Cnt16_Stop();
    QuadDecDrive_Cnt16_SaveConfig();
}


/*******************************************************************************
* Function Name: QuadDecDrive_Cnt16_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*  
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  QuadDecDrive_Cnt16_backup.enableState:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void QuadDecDrive_Cnt16_Wakeup(void) 
{
    QuadDecDrive_Cnt16_RestoreConfig();
    #if(!QuadDecDrive_Cnt16_ControlRegRemoved)
        if(QuadDecDrive_Cnt16_backup.CounterEnableState == 1u)
        {
            /* Enable Counter's operation */
            QuadDecDrive_Cnt16_Enable();
        } /* Do nothing if Counter was disabled before */    
    #endif /* (!QuadDecDrive_Cnt16_ControlRegRemoved) */
    
}


/* [] END OF FILE */
