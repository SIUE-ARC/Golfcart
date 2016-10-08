/***************************************************************************
* File Name: RAMBUF_dma.c  
* Version 1.70
*
*  Description:
*   Provides an API for the DMAC component. The API includes functions
*   for the DMA controller, DMA channels and Transfer Descriptors.
*
*
*   Note:
*     This module requires the developer to finish or fill in the auto
*     generated funcions and setup the dma channel and TD's.
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/
#include <CYLIB.H>
#include <CYDMAC.H>
#include <RAMBUF_dma.H>



/****************************************************************************
* 
* The following defines are available in Cyfitter.h
* 
* 
* 
* RAMBUF__DRQ_CTL_REG
* 
* 
* RAMBUF__DRQ_NUMBER
* 
* Number of TD's used by this channel.
* RAMBUF__NUMBEROF_TDS
* 
* Priority of this channel.
* RAMBUF__PRIORITY
* 
* True if RAMBUF_TERMIN_SEL is used.
* RAMBUF__TERMIN_EN
* 
* TERMIN interrupt line to signal terminate.
* RAMBUF__TERMIN_SEL
* 
* 
* True if RAMBUF_TERMOUT0_SEL is used.
* RAMBUF__TERMOUT0_EN
* 
* 
* TERMOUT0 interrupt line to signal completion.
* RAMBUF__TERMOUT0_SEL
* 
* 
* True if RAMBUF_TERMOUT1_SEL is used.
* RAMBUF__TERMOUT1_EN
* 
* 
* TERMOUT1 interrupt line to signal completion.
* RAMBUF__TERMOUT1_SEL
* 
****************************************************************************/


/* Zero based index of RAMBUF dma channel */
uint8 RAMBUF_DmaHandle = DMA_INVALID_CHANNEL;

/*********************************************************************
* Function Name: uint8 RAMBUF_DmaInitalize
**********************************************************************
* Summary:
*   Allocates and initialises a channel of the DMAC to be used by the
*   caller.
*
* Parameters:
*   BurstCount.
*       
*       
*   ReqestPerBurst.
*       
*       
*   UpperSrcAddress.
*       
*       
*   UpperDestAddress.
*       
*
* Return:
*   The channel that can be used by the caller for DMA activity.
*   DMA_INVALID_CHANNEL (0xFF) if there are no channels left. 
*
*
*******************************************************************/
uint8 RAMBUF_DmaInitialize(uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) 
{

    /* Allocate a DMA channel. */
    RAMBUF_DmaHandle = (uint8)RAMBUF__DRQ_NUMBER;

    /* Configure the channel. */
    (void)CyDmaChSetConfiguration(RAMBUF_DmaHandle,
                                  BurstCount,
                                  ReqestPerBurst,
                                  (uint8)RAMBUF__TERMOUT0_SEL,
                                  (uint8)RAMBUF__TERMOUT1_SEL,
                                  (uint8)RAMBUF__TERMIN_SEL);

    /* Set the extended address for the transfers */
    (void)CyDmaChSetExtendedAddress(RAMBUF_DmaHandle, UpperSrcAddress, UpperDestAddress);

    /* Set the priority for this channel */
    (void)CyDmaChPriority(RAMBUF_DmaHandle, (uint8)RAMBUF__PRIORITY);
    
    return RAMBUF_DmaHandle;
}

/*********************************************************************
* Function Name: void RAMBUF_DmaRelease
**********************************************************************
* Summary:
*   Frees the channel associated with RAMBUF.
*
*
* Parameters:
*   void.
*
*
*
* Return:
*   void.
*
*******************************************************************/
void RAMBUF_DmaRelease(void) 
{
    /* Disable the channel */
    (void)CyDmaChDisable(RAMBUF_DmaHandle);
}

