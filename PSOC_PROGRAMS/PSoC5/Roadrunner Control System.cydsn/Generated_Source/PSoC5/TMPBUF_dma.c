/***************************************************************************
* File Name: TMPBUF_dma.c  
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
#include <TMPBUF_dma.H>



/****************************************************************************
* 
* The following defines are available in Cyfitter.h
* 
* 
* 
* TMPBUF__DRQ_CTL_REG
* 
* 
* TMPBUF__DRQ_NUMBER
* 
* Number of TD's used by this channel.
* TMPBUF__NUMBEROF_TDS
* 
* Priority of this channel.
* TMPBUF__PRIORITY
* 
* True if TMPBUF_TERMIN_SEL is used.
* TMPBUF__TERMIN_EN
* 
* TERMIN interrupt line to signal terminate.
* TMPBUF__TERMIN_SEL
* 
* 
* True if TMPBUF_TERMOUT0_SEL is used.
* TMPBUF__TERMOUT0_EN
* 
* 
* TERMOUT0 interrupt line to signal completion.
* TMPBUF__TERMOUT0_SEL
* 
* 
* True if TMPBUF_TERMOUT1_SEL is used.
* TMPBUF__TERMOUT1_EN
* 
* 
* TERMOUT1 interrupt line to signal completion.
* TMPBUF__TERMOUT1_SEL
* 
****************************************************************************/


/* Zero based index of TMPBUF dma channel */
uint8 TMPBUF_DmaHandle = DMA_INVALID_CHANNEL;

/*********************************************************************
* Function Name: uint8 TMPBUF_DmaInitalize
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
uint8 TMPBUF_DmaInitialize(uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) 
{

    /* Allocate a DMA channel. */
    TMPBUF_DmaHandle = (uint8)TMPBUF__DRQ_NUMBER;

    /* Configure the channel. */
    (void)CyDmaChSetConfiguration(TMPBUF_DmaHandle,
                                  BurstCount,
                                  ReqestPerBurst,
                                  (uint8)TMPBUF__TERMOUT0_SEL,
                                  (uint8)TMPBUF__TERMOUT1_SEL,
                                  (uint8)TMPBUF__TERMIN_SEL);

    /* Set the extended address for the transfers */
    (void)CyDmaChSetExtendedAddress(TMPBUF_DmaHandle, UpperSrcAddress, UpperDestAddress);

    /* Set the priority for this channel */
    (void)CyDmaChPriority(TMPBUF_DmaHandle, (uint8)TMPBUF__PRIORITY);
    
    return TMPBUF_DmaHandle;
}

/*********************************************************************
* Function Name: void TMPBUF_DmaRelease
**********************************************************************
* Summary:
*   Frees the channel associated with TMPBUF.
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
void TMPBUF_DmaRelease(void) 
{
    /* Disable the channel */
    (void)CyDmaChDisable(TMPBUF_DmaHandle);
}

