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

byte obuffer[BUFFER_SIZE];
byte ibuffer[BUFFER_SIZE];

hword ssample[100];
hword bsample[100];

int main()
{
    /* Variable declarations for USBFS */
    hword length;
    byte i;
    //byte j = 0;
    byte argc;
    byte** argv;
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    init();
    
    USBFS_EnableOutEP(OUT_EP_NUM);
    for(;;)
    {
        argc = 0;
        
        //hword sample = POTADC_GetResult16();
        //j = (j == 99) ?  0:j;
        
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
            
            
            command_lookup(argc, argv);
            
            /* Enable OUT endpoint to receive data from host. */
            USBFS_EnableOutEP(OUT_EP_NUM);

            /* Wait until IN buffer becomes empty (host has read data). */
            while (USBFS_IN_BUFFER_EMPTY != USBFS_GetEPState(IN_EP_NUM))
            {
            }

            //BRAKEADC_StopConvert();
            //STEERADC_StopConvert();
            //i = (i >= BUFFER_SIZE) ? 0:i;
            
            //ibuffer[i] = LO8(bsample[j]);
            //ibuffer[i+1] = HI8(bsample[j]);
            //ibuffer[i+2] = LO8(bsample[j]);
            //ibuffer[i+3] = HI8(bsample[j]);
            
            //i += 2;
            //length = i;
            
        /* Trigger DMA to copy data into IN endpoint buffer.
        * After data has been copied, IN endpoint is ready to be read by the
        * host.
        */
            USBFS_LoadInEP(IN_EP_NUM, ibuffer, length);
        
            BRAKEADC_StartConvert();
            STEERADC_StartConvert();
        }
        //j++;
    }
}


void command_lookup(byte argc, byte** argv)
{
    byte command = obuffer[0];
    
    switch(command)
    {
        case 'F':
        case 'f':
            if(argc > 0)
            {
                hword dval = atoi(argv[1]);
                RLY_REVERSE_DR &= ~RLY_REVERSE_MASK;
                DVDAC_SetValue(dval);
            }
            else
            {
                stpcpy(ibuffer, "No DAC val given!\r\n");
            }
            break;
        case 'B':
        case 'b':
            if(argc > 0)
            {
                hword dval = atoi(argv[1]);
                RLY_REVERSE_DR |= RLY_REVERSE_MASK;
                DVDAC_SetValue(dval);
            }
            else
            {
                stpcpy(ibuffer, "No DAC val given!\r\n");
            }
            break;
        case 'D':
        case 'd':
            {
                byte* buf0 = "Drive encoder: ";
                byte* buf1;
                hword count = QuadDecDrive_GetCounter();
                itoa(count, buf1, 10); 
                #ifdef VERBOSE 
                    strcpy(buf0, buf1);
                    strcpy(buf0, "\r\n");
                    strcpy(ibuffer, buf0);
                #else
                    strcpy(ibuffer, buf1);
                #endif
            }
            break;
        case 'I':
        case 'i':
            UART_WriteTxData((char)BAUD_BYTE);
            //baudSent = TRUE;
            break;
        case 'T':
        case 't':
            if(argc > 0)
                turn(atoi(argv[1]));
                
            #ifdef VERBOSE
                else strcpy(ibuffer, "No value given!\r\n");
            #endif
            break;
        case 'S':
        case 's':
            {
                byte* buf0 = "Steer encoder: ";
                byte* buf1;
                hword count = QuadDecSteer_GetCounter();
                itoa(count, buf1, 10); 
                #ifdef VERBOSE 
                    strcpy(buf0, buf1);
                    strcpy(buf0, "\r\n");
                    strcpy(ibuffer, buf0);
                #else
                    strcpy(ibuffer, buf1);
                #endif
            }
            break;
        case 'R':
        case 'r':
            calibrateSteering();
            strcpy(ibuffer, "Shaft reset\r\n");
            break;
        
    #ifdef EXTENDED_COMMANDS
        case 'H':
        case 'h':
            break;
        case 'E':
        case 'e':
            break;
        case 'Q':
        case 'q':
            break;
        case 'L':
        case 'l':
            break;
        case 'J':
        case 'j':
            break;
        case 'P':
        case 'p':
            break;
    #endif
    }
}
/* [] END OF FILE */
