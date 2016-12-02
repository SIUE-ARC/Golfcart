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
#include "roadrunner.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

int main()
{
    /* Variable declarations for USBFS */
    hword length;
    byte i;
    byte estop = FALSE;
    
    init();
    USBFS_Start(USBFS_DEVICE, USBFS_5V_OPERATION);
    while(0u == USBFS_GetConfiguration()){} //wait until we are enumerated by host
    USBFS_EnableOutEP(OUT_EP_NUM);
    
    for(;;)
    {
        argc = 0;
        cacheValid = FALSE;
        
        if(!(ESTOP_DR & ESTOP_MASK))
        {
            stop();
            #ifdef VERBOSE
                strcpy(ibuffer, "ESTOP engaged!\0");
            #endif
            
            while (!(ESTOP_DR & ESTOP_MASK));
            
            #ifdef VERBOSE
                strcpy(ibuffer, "Resume\0");
            #endif
            estop = TRUE;
        }
        else
            estop = FALSE;
        
        // Run the PID calculations for both motors
        updateBrakeCtl();
        updateTurnCtl();
    
        /* Check if configuration is changed. */
        if (0u != USBFS_IsConfigurationChanged())
        {
            /* Re-enable endpoint when device is configured. */
            if (0u != USBFS_GetConfiguration())
            {
                /* Enable OUT endpoint to receive data from host. */
                USBFS_EnableOutEP(OUT_EP_NUM);
            }
        }

        /* Check if data was received. */
        if (USBFS_OUT_BUFFER_FULL == USBFS_GetEPState(OUT_EP_NUM))
        {
            /* Read number of received data bytes. */
            length = USBFS_GetEPCount(OUT_EP_NUM);

            /* Trigger DMA to copy data from OUT endpoint buffer. */
            USBFS_ReadOutEP(OUT_EP_NUM, obuffer, length);

            /* Wait until DMA completes copying data from OUT endpoint buffer. */
            while (USBFS_OUT_BUFFER_FULL == USBFS_GetEPState(OUT_EP_NUM))
            {
            }
            
            for(i = 0; i < length; i++)
            {
                if(obuffer[i] == ' ')
                {
                    argc++;
                    obuffer[i] = '\0';
                    argv[argc] = &obuffer[i+1];
                }
            }
            
            if(!estop) command_lookup(argc, argv);
            
            /* Enable OUT endpoint to receive data from host. */
            USBFS_EnableOutEP(OUT_EP_NUM);

            /* Wait until IN buffer becomes empty (host has read data). */
            while (USBFS_IN_BUFFER_EMPTY != USBFS_GetEPState(IN_EP_NUM))
            {
            }
            
            USBFS_LoadInEP(IN_EP_NUM, ibuffer, BUFFER_SIZE);
        }
    }
}
/* [] END OF FILE */
