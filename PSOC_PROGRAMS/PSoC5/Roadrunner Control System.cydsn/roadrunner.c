/* ========================================
 * This file contains all neccessary definitions
 * for the functions described in roadrunner.h
 * ========================================
*/
#include "roadrunner.h"
#include <stdlib.h>

byte obuffer[BUFFER_SIZE];
byte ibuffer[BUFFER_SIZE];

hword ssample[100];
hword bsample[100];


// Are currently cached analog reads valid, or should they be reaquired
byte cacheValid = FALSE;

// Have the motor controllers been initilized
byte baudSent = FALSE;

// The current count of the quadrature encoder connected to the steering column
int steerCount = 0;

// The current target positions for both motor controllers
int brakeSetpoint = 0;
int steerSetpoint = 0;

byte argc = 0;
byte** argv;

void init()
{
    /* Variable declarations for RAMBUF1 */
    /* Move these variable declarations to the top of the function */
    uint8 RAMBUF1_Chan;
    uint8 RAMBUF1_TD[1];
    /* Variable declarations for RAMBUF2 */
    /* Move these variable declarations to the top of the function */
    uint8 RAMBUF2_Chan;
    uint8 RAMBUF2_TD[1];
    
    argv = (byte**)malloc(sizeof(byte*)*BUFFER_SIZE);

    /* DMA Configuration for RAMBUF1 */
    RAMBUF1_Chan = RAMBUF1_DmaInitialize(RAMBUF_BYTES_PER_BURST, RAMBUF_REQUEST_PER_BURST, 
                                         HI16(RAMBUF_SRC_BASE), HI16(RAMBUF_DST_BASE));
    RAMBUF1_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(RAMBUF1_TD[0], 200, RAMBUF1_TD[0], TD_INC_DST_ADR | TD_AUTO_EXEC_NEXT | RAMBUF1__TD_TERMOUT_EN);
    CyDmaTdSetAddress(RAMBUF1_TD[0], LO16((uint32)STEERADC_SAR_WRK0_PTR), LO16((uint32)ssample));
    CyDmaChSetInitialTd(RAMBUF1_Chan, RAMBUF1_TD[0]);
    CyDmaChEnable(RAMBUF1_Chan, 1);
    
    /* DMA Configuration for RAMBUF2 */
    RAMBUF2_Chan = RAMBUF2_DmaInitialize(RAMBUF_BYTES_PER_BURST, RAMBUF_REQUEST_PER_BURST, 
                                         HI16(RAMBUF_SRC_BASE), HI16(RAMBUF_DST_BASE));
    RAMBUF2_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(RAMBUF2_TD[0], 200, RAMBUF2_TD[0], TD_INC_DST_ADR | TD_AUTO_EXEC_NEXT | RAMBUF2__TD_TERMOUT_EN);
    CyDmaTdSetAddress(RAMBUF2_TD[0], LO16((uint32)BRAKEADC_SAR_WRK0_PTR), LO16((uint32)bsample));
    CyDmaChSetInitialTd(RAMBUF2_Chan, RAMBUF2_TD[0]);
    CyDmaChEnable(RAMBUF2_Chan, 1);
    
    USBFS_Start(USBFS_DEVICE, USBFS_5V_OPERATION);
    while(0u == USBFS_GetConfiguration()){} //wait until we are enumerated by host
    
    DVDAC_Start();
    DVDAC_SetValue(0);
    UART_Start();
    
    STEERADC_Start();
    STEERADC_StartConvert();
    STEERADC_IRQ_Disable();
    BRAKEADC_Start();
    BRAKEADC_StartConvert();
    BRAKEADC_IRQ_Disable();
    
    RAMBUF1_DONE_Start();
    RAMBUF2_DONE_Start();
    
    USBFS_EnableOutEP(OUT_EP_NUM);
    CyGlobalIntEnable; /* Enable global interrupts. */
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
                stpcpy(ibuffer, "No DAC val given!\r\n\0");
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
                stpcpy(ibuffer, "No DAC val given!\r\n\0");
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
                    strcpy(buf0, "\r\n\0");
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

// Tells the specified motor controller to run at the specfied speed. The
// controller will continue to run until it is explicity stopped. setControllerSpeed
// will cache the last parameters sent to each motor and only update the motor
// controller if the values have changed.
// Speed must be in the range [0, 127]
// dir is either a 0 or 1
void setControllerSpeed(byte addr, byte speed, byte dir)
{
    static byte lastValue[2];
    
    // Only send the serial packet if it changes the state of the motor controller of interest
    // speed is only a 7 bit value
    if (baudSent && (lastValue[addr == BRAKE_CTL ? 1 : 0] != (dir << 7 | speed))) 
    {
        // Packet format, 4 bytes
        // Address [128, 255], direction [0, 1], speed [0, 127], checksum
		byte TX[4];

		// This line has to be after line 320
		if((lastValue[addr == BRAKE_CTL ? 1 : 0] >> 7) != dir)
			speed = STOP;
		
        TX[0] = addr;
        TX[1] = dir;
        TX[2] = speed;
        TX[3] = (addr + dir + speed) & 0x7F;
        UART_PutArray(TX, 4);
        
        lastValue[addr == BRAKE_CTL ? 1 : 0] = (dir << 7 | speed);

        #ifdef VERBOSE
            UART_CPutString(addr == BRAKE_CTL ? "Brake" : "Turn");
            if(speed != 0) {
                UART_CPutString(" controller turning ");
                UART_CPutString(dir == LEFT ? "left" : "right");
            } else {
                UART_CPutString(" controller stopping");
            }
            UART_PutCRLF();
        #endif
    }
}

void turn(word count)
{
    count = max(ENCODER_LEFT_BOUND, count);
    count = min(ENCODER_RIGHT_BOUND, count);
    
    steerSetpoint = count;
    
    #ifdef VERBOSE
        UART_CPutString("Steer setpoint: ");
        UART_PutSHexInt(count);
        UART_PutCRLF();
    #endif
}

void brake(int pVal)
{
    pVal = max(BRAKE_MIN_POS, pVal);
    pVal = min(BRAKE_MAX_POS, pVal);
    
    brakeSetpoint = pVal;
    
    #ifdef VERBOSE
        UART_CPutString("Brake setpoint: ");
        UART_PutSHexInt(pVal);
        UART_PutCRLF();
    #endif 
}

void stop(void) 
{
    setControllerSpeed(STEER_CTL, STOP, STOP);
    setControllerSpeed(BRAKE_CTL, STOP, STOP);
    
    #ifdef VERBOSE
    #endif
}

hword getBrakePosition()
{
    int i;
    hword sum = 0;
    for(i = 0; i < 100; i++)
    {
        sum += bsample[i];
    }
    
    return sum/100;   
}

hword getSteerPosition()
{
    int i;
    hword sum = 0;
    for(i = 0; i < 100; i++)
    {
        sum += ssample[i];
    }
    
    return sum/100;   
}

void calibrateSteering()
{
    // Send the control off in the right direction
    byte dir = getSteerPosition() < STEER_POT_CENTER ? RIGHT : LEFT;
    setControllerSpeed(STEER_CTL, STEER_SPEED, dir);
    
    // Spin until the pot lines up
    while (abs(getSteerPosition() - STEER_POT_CENTER) > 30)
    {
        #ifdef VERBOSE
		    UART_PutSHexInt(getSteerPotPosition() - STEER_POT_CENTER);
        #endif
    }
    // We are spinning here, so we need to invalidate the cache ourself
    cacheValid = FALSE;
    
    // Reset everything at center
    setControllerSpeed(STEER_CTL, STOP, STOP);
    steerCount = 0;
    steerSetpoint = 0;
}

void updateBrakeCtl()
{
    int error = brakeSetpoint - getBrakePosition();
    
    if (error < -20)
        setControllerSpeed(BRAKE_CTL, BRAKE_SPEED, APPLY);
    else if (error > 20)
        setControllerSpeed(BRAKE_CTL, BRAKE_SPEED, RELEASE);
    else
        setControllerSpeed(BRAKE_CTL, STOP, STOP);
}

void updateTurnCtl()
{
    int error = steerSetpoint - steerCount;
    
    if (error < -50)
        setControllerSpeed(STEER_CTL, STEER_SPEED, RIGHT);
    else if (error > 50)
        setControllerSpeed(STEER_CTL, STEER_SPEED, LEFT);
    else
        setControllerSpeed(STEER_CTL, STOP, STOP);
}

void handleUSB(byte estop)
{
    /* Variable declarations for USBFS */
    hword length;
    byte i;
    
     for(;;)
    {
        argc = 0;
        
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

int min(int a, int b) { return a < b ? a : b; }
int max(int a, int b) { return a > b ? a : b; }
/* [] END OF FILE */
