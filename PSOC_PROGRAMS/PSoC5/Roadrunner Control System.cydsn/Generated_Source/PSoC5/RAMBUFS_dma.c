/***************************************************************************
* File Name: RAMBUFS_dma.c  
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
#include <RAMBUFS_dma.H>



/****************************************************************************
* 
* The following defines are available in Cyfitter.h
* 
* 
* 
* RAMBUFS__DRQ_CTL_REG
* 
* 
* RAMBUFS__DRQ_NUMBER
* 
* Number of TD's used by this channel.
* RAMBUFS__NUMBEROF_TDS
* 
* Priority of this channel.
* RAMBUFS__PRIORITY
* 
* True if RAMBUFS_TERMIN_SEL is used.
* RAMBUFS__TERMIN_EN
* 
* TERMIN interrupt line to signal terminate.
* RAMBUFS__TERMIN_SEL
* 
* 
* True if RAMBUFS_TERMOUT0_SEL is used.
* RAMBUFS__TERMOUT0_EN
* 
* 
* TERMOUT0 interrupt line to signal completion.
* RAMBUFS__TERMOUT0_SEL
* 
* 
* True if RAMBUFS_TERMOUT1_SEL is used.
* RAMBUFS__TERMOUT1_EN
* 
* 
* TERMOUT1 interrupt line to signal completion.
* RAMBUFS__TERMOUT1_SEL
* 
****************************************************************************/


/* Zero based index of RAMBUFS dma channel */
uint8 RAMBUFS_DmaHandle = DMA_INVALID_CHANNEL;

/*********************************************************************
* Function Name: uint8 RAMBUFS_DmaInitalize
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
uint8 RAMBUFS_DmaInitialize(uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) 
{

    /* Allocate a DMA channel. */
    RAMBUFS_DmaHandle = (uint8)RAMBUFS__DRQ_NUMBER;

    /* Configure the channel. */
    (void)CyDmaChSetConfiguration(RAMBUFS_DmaHandle,
                                  BurstCount,
                                  ReqestPerBurst,
                                  (uint8)RAMBUFS__TERMOUT0_SEL,
                                  (uint8)RAMBUFS__TERMOUT1_SEL,
                                  (uint8)RAMBUFS__TERMIN_SEL);

    /* Set the extended address for the transfers */
    (void)CyDmaChSetExtendedAddress(RAMBUFS_DmaHandle, UpperSrcAddress, UpperDestAddress);

    /* Set the priority for this channel */
    (void)CyDmaChPriority(RAMBUFS_DmaHandle, (uint8)RAMBUFS__PRIORITY);
    
    return RAMBUFS_DmaHandle;
}

/*********************************************************************
* Function Name: void RAMBUFS_DmaRelease
**********************************************************************
* Summary:
*   Frees the channel associated with RAMBUFS.
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
void RAMBUFS_DmaRelease(void) 
{
    /* Disable the channel */
    (void)CyDmaChDisable(RAMBUFS_DmaHandle);
}

