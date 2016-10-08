/*******************************************************************************
* File Name: QuadDecDrive.c  
* Version 3.0
*
* Description:
*  This file provides the source code to the API for the Quadrature Decoder
*  component.
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

#if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
    #include "QuadDecDrive_PVT.h"
#endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */

uint8 QuadDecDrive_initVar = 0u;


/*******************************************************************************
* Function Name: QuadDecDrive_Init
********************************************************************************
*
* Summary:
*  Inits/Restores default QuadDec configuration provided with customizer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecDrive_Init(void) 
{
    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
        /* Disable Interrupt. */
        CyIntDisable(QuadDecDrive_ISR_NUMBER);
        /* Set the ISR to point to the QuadDecDrive_isr Interrupt. */
        (void) CyIntSetVector(QuadDecDrive_ISR_NUMBER, & QuadDecDrive_ISR);
        /* Set the priority. */
        CyIntSetPriority(QuadDecDrive_ISR_NUMBER, QuadDecDrive_ISR_PRIORITY);
    #endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */
}


/*******************************************************************************
* Function Name: QuadDecDrive_Enable
********************************************************************************
*
* Summary:
*  This function enable interrupts from Component and also enable Component's
*  ISR in case of 32-bit counter.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecDrive_Enable(void) 
{
    uint8 enableInterrupts;

    QuadDecDrive_SetInterruptMask(QuadDecDrive_INIT_INT_MASK);

    /* Clear pending interrupts. */
    (void) QuadDecDrive_GetEvents();
    
    enableInterrupts = CyEnterCriticalSection();

    /* Enable interrupts from Statusi register */
    QuadDecDrive_SR_AUX_CONTROL |= QuadDecDrive_INTERRUPTS_ENABLE;

    CyExitCriticalSection(enableInterrupts);        

    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
        /* Enable Component interrupts */
        CyIntEnable(QuadDecDrive_ISR_NUMBER);
    #endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */
}


/*******************************************************************************
* Function Name: QuadDecDrive_Start
********************************************************************************
*
* Summary:
*  Initializes UDBs and other relevant hardware.
*  Resets counter, enables or disables all relevant interrupts.
*  Starts monitoring the inputs and counting.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  QuadDecDrive_initVar - used to check initial configuration, modified on
*  first function call.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void QuadDecDrive_Start(void) 
{
    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
        QuadDecDrive_Cnt8_Start();
        QuadDecDrive_Cnt8_WriteCounter(QuadDecDrive_COUNTER_INIT_VALUE);
    #else
        /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) || 
        *  (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT) 
        */
        QuadDecDrive_Cnt16_Start();
        QuadDecDrive_Cnt16_WriteCounter(QuadDecDrive_COUNTER_INIT_VALUE);
    #endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT */
    
    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)        
       QuadDecDrive_count32SoftPart = 0;
    #endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */

    if (QuadDecDrive_initVar == 0u)
    {
        QuadDecDrive_Init();
        QuadDecDrive_initVar = 1u;
    }

    QuadDecDrive_Enable();
}


/*******************************************************************************
* Function Name: QuadDecDrive_Stop
********************************************************************************
*
* Summary:
*  Turns off UDBs and other relevant hardware.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecDrive_Stop(void) 
{
    uint8 enableInterrupts;

    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
        QuadDecDrive_Cnt8_Stop();
    #else 
        /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) ||
        *  (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
        */
        QuadDecDrive_Cnt16_Stop();    /* counter disable */
    #endif /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT) */
 
    enableInterrupts = CyEnterCriticalSection();

    /* Disable interrupts interrupts from Statusi register */
    QuadDecDrive_SR_AUX_CONTROL &= (uint8) (~QuadDecDrive_INTERRUPTS_ENABLE);

    CyExitCriticalSection(enableInterrupts);

    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
        CyIntDisable(QuadDecDrive_ISR_NUMBER);    /* interrupt disable */
    #endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */
}


/*******************************************************************************
* Function Name: QuadDecDrive_GetCounter
********************************************************************************
*
* Summary:
*  Reports the current value of the counter.
*
* Parameters:
*  None.
*
* Return:
*  The counter value. Return type is signed and per the counter size setting.
*  A positive value indicates clockwise movement (B before A).
*
* Global variables:
*  QuadDecDrive_count32SoftPart - used to get hi 16 bit for current value
*  of the 32-bit counter, when Counter size equal 32-bit.
*
*******************************************************************************/
int16 QuadDecDrive_GetCounter(void) 
{
    int16 count;
    uint16 tmpCnt;

    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
        int16 hwCount;

        CyIntDisable(QuadDecDrive_ISR_NUMBER);

        tmpCnt = QuadDecDrive_Cnt16_ReadCounter();
        hwCount = (int16) ((int32) tmpCnt - (int32) QuadDecDrive_COUNTER_INIT_VALUE);
        count = QuadDecDrive_count32SoftPart + hwCount;

        CyIntEnable(QuadDecDrive_ISR_NUMBER);
    #else 
        /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT) || 
        *  (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT)
        */
        #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
            tmpCnt = QuadDecDrive_Cnt8_ReadCounter();
        #else /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) */
            tmpCnt = QuadDecDrive_Cnt16_ReadCounter();
        #endif  /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT */

        count = (int16) ((int32) tmpCnt -
                (int32) QuadDecDrive_COUNTER_INIT_VALUE);

    #endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */ 

    return (count);
}


/*******************************************************************************
* Function Name: QuadDecDrive_SetCounter
********************************************************************************
*
* Summary:
*  Sets the current value of the counter.
*
* Parameters:
*  value:  The new value. Parameter type is signed and per the counter size
*  setting.
*
* Return:
*  None.
*
* Global variables:
*  QuadDecDrive_count32SoftPart - modified to set hi 16 bit for current
*  value of the 32-bit counter, when Counter size equal 32-bit.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void QuadDecDrive_SetCounter(int16 value) 
{
    #if ((QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT) || \
         (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT))
        uint16 count;
        
        if (value >= 0)
        {
            count = (uint16) value + QuadDecDrive_COUNTER_INIT_VALUE;
        }
        else
        {
            count = QuadDecDrive_COUNTER_INIT_VALUE - (uint16)(-value);
        }
        #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
            QuadDecDrive_Cnt8_WriteCounter(count);
        #else /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) */
            QuadDecDrive_Cnt16_WriteCounter(count);
        #endif  /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT */
    #else /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT) */
        CyIntDisable(QuadDecDrive_ISR_NUMBER);

        QuadDecDrive_Cnt16_WriteCounter(QuadDecDrive_COUNTER_INIT_VALUE);
        QuadDecDrive_count32SoftPart = value;

        CyIntEnable(QuadDecDrive_ISR_NUMBER);
    #endif  /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT) ||
             * (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT)
             */
}


/*******************************************************************************
* Function Name: QuadDecDrive_GetEvents
********************************************************************************
* 
* Summary:
*   Reports the current status of events. This function clears the bits of the 
*   status register.
*
* Parameters:
*  None.
*
* Return:
*  The events, as bits in an unsigned 8-bit value:
*    Bit      Description
*     0        Counter overflow.
*     1        Counter underflow.
*     2        Counter reset due to index, if index input is used.
*     3        Invalid A, B inputs state transition.
*
*******************************************************************************/
uint8 QuadDecDrive_GetEvents(void) 
{
    return (QuadDecDrive_STATUS_REG & QuadDecDrive_INIT_INT_MASK);
}


/*******************************************************************************
* Function Name: QuadDecDrive_SetInterruptMask
********************************************************************************
*
* Summary:
*  Enables / disables interrupts due to the events.
*  For the 32-bit counter, the overflow, underflow and reset interrupts cannot
*  be disabled, these bits are ignored.
*
* Parameters:
*  mask: Enable / disable bits in an 8-bit value, where 1 enables the interrupt.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecDrive_SetInterruptMask(uint8 mask) 
{
    #if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
        /* Underflow, Overflow and Reset interrupts for 32-bit Counter are always enable */
        mask |= (QuadDecDrive_COUNTER_OVERFLOW | QuadDecDrive_COUNTER_UNDERFLOW |
                 QuadDecDrive_COUNTER_RESET);
    #endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */

    QuadDecDrive_STATUS_MASK = mask;
}


/*******************************************************************************
* Function Name: QuadDecDrive_GetInterruptMask
********************************************************************************
*
* Summary:
*  Reports the current interrupt mask settings.
*
* Parameters:
*  None.
*
* Return:
*  Enable / disable bits in an 8-bit value, where 1 enables the interrupt.
*  For the 32-bit counter, the overflow, underflow and reset enable bits are
*  always set.
*
*******************************************************************************/
uint8 QuadDecDrive_GetInterruptMask(void) 
{
    return (QuadDecDrive_STATUS_MASK & QuadDecDrive_INIT_INT_MASK);
}


/* [] END OF FILE */
