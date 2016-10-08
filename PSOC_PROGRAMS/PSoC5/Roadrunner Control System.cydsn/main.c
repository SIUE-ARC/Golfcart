/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>

/* Defines for RAMBUF */
#define RAMBUF_BYTES_PER_BURST 2
#define RAMBUF_REQUEST_PER_BURST 1
#define RAMBUF_SRC_BASE (CYDEV_PERIPH_BASE)
#define RAMBUF_DST_BASE (CYDEV_SRAM_BASE)

unsigned int adc_samp[2]   =    {0, 0};
void init();

int main()
{
    init();
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        /* Place your application code here. */
    }
}

void init()
{
    /* Variable declarations for RAMBUF */
    /* Move these variable declarations to the top of the function */
    uint8 RAMBUF_Chan;
    uint8 RAMBUF_TD[1];

    /* DMA Configuration for RAMBUF */
    RAMBUF_Chan = RAMBUF_DmaInitialize(RAMBUF_BYTES_PER_BURST, RAMBUF_REQUEST_PER_BURST, 
        HI16(RAMBUF_SRC_BASE), HI16(RAMBUF_DST_BASE));
    RAMBUF_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(RAMBUF_TD[0], 2, RAMBUF_TD[0], TD_INC_DST_ADR);
    CyDmaTdSetAddress(RAMBUF_TD[0], LO16((uint32)POTADC_SAR_WRK0_PTR), LO16((uint32)adc_samp[0]));
    CyDmaChSetInitialTd(RAMBUF_Chan, RAMBUF_TD[0]);
    CyDmaChEnable(RAMBUF_Chan, 1);

}
/* [] END OF FILE */
