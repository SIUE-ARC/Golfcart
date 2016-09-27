//----------------------------------------------------------------------------
/* Roadrunner AI Golfcart Drive Control Program using PSoC 1
	This program is the main control for the drive system on the golfcart.
	It waits for a command to be sent and responds accordingly depending on
	the command.
	
	Modules
	-------
	UART(Universal Asynchronous Receiver Transmitter) for serial communication
	DAC(Digital to Analog Converter) for sending analog voltages to the golfcart's
		"Black Box"
	
	Pinout
	------
	0[3] - DAC(Digital to Analog Converter)Ouput
	1[2] - Forward or Reverse Pin goes to 2N2222 Base to flip relay on drive motor
	1[3] - Autonomous Toggle Pin goes to 2N2222 Base to flip indicator light motor
	1[4] - Optical Encoder A Pin Reads Channel A on the encoder
	1[5] - Optical Encoder B Pin Reads Channel B on the encoder
	1[6] - Receive line for UART
	2[7] - Transmit line for UART
	
	Command Structure
	-----------------
	'A' - Autonomous Mode Toggle turns on the motor for the indicator light
	'F xx' - Forward at speed "xx". Forward takes values from 0-254. Ex: F 127
		Also makes sure 1[2] is output low to not flip the relay
	'B xx' - Backward at speed "xx". Backward takes values from 0-254. Ex: B 84
		Also sets the bit at 1[2] to high which flips the drive relay
	'D' - Requests the current drive encoder count
	'I' - Initialize: Requests a ping from the PSoC. PSoC responds with a 'D' 
		for Drive
	
	Authors: Cameron Costanzo, Jared Charter, Bryan Orabutt
	Date: October 2, 2015
*/
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include <stdlib.h>	
#include "PSoCGPIOINT.h"
#include <string.h>

#pragma interrupt_handler PSoC_GPIO_ISR_C

// Globals
int val; // DAC value 
long glblCount = 0;	// Global encoder count
BYTE prevPrt; // Previous state of port 1 for encoder count
BYTE curPrt; // Current state of port 1 for encoder count

// Functions
int convert(char* c);
void command_lookup(BYTE cmd);

void main(void)
{
	char* data;
	val = 0;
	
	// Initialize the UART for communication
	UART_1_CmdReset();
	UART_1_IntCntl(UART_1_ENABLE_RX_INT);
	UART_1_Start(UART_1_PARITY_NONE);
	
	M8C_EnableGInt ;
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO); 	// Enable GPIO Interrupts
	
	// Initialize DAC
	DAC8_1_Start(DAC8_1_HIGHPOWER);
	DAC8_1_WriteBlind(val);
		
	UART_1_CPutString("Program Started\r\n");
	
	while (TRUE)
	{
		// Setting the previous value of the port for encoder count updates
		prevPrt = (PRT1DR & (OpEncA_MASK | OpEncB_MASK)); 
		
		// Check for command in the command buffer
		if(UART_1_bCmdCheck()) 
		{                    
			if(data = UART_1_szGetParam()) 
			{
				command_lookup(*data);
			}   
			UART_1_CmdReset();  // Reset command buffer     
		}  

	}
}

// Converts the ascii characters from the command to an int
int convert(char* c)
{
	int result = 0;
	int i = 0;
	char check;
	while(check = c[i])
	{
		// Takes the ascii character and converts it to decimal
		if(i == 0)
			result += (((int)c[i]) - 48)*1000;
		else if(i == 1)
			result += (((int)c[i]) - 48)*100;
		else if(i == 2)
			result += (((int)c[i]) - 48)*10;
		else
			result += (((int)c[i]) - 48);
			
		i++;
	}
	if (i == 1)
		result = result/1000;
	else if (i == 2)
		result = result/100;
	else if (i == 3)
		result = result/10;
	return result;
}

// Command lookup structure
void command_lookup(BYTE cmd)
{
	BYTE* data;
	BYTE* TX;
	BYTE addr = 128;
	BYTE dir = 0;
	BYTE val = 0;
	BYTE checksum = 0;
	switch (cmd)
	{
		// Autonomous Mode Toggle
		case 'A':
		case 'a':
			PRT1DR ^= 0x08;
			UART_1_CPutString("Toggling Autonomy\r\n"); 
			break;
		// Forward at speed 0-254
		case 'F':
		case 'f':
			if (data = UART_1_szGetParam())
			{
				UART_1_CPutString("Moving forward >"); 
				val = convert(data); 
				UART_1_PutString(data);             // Print out command
				UART_1_CPutString("<\r\n"); 
				PRT1DR &= ~0x04;
				DAC8_1_WriteBlind(val);
			}
			else 
				UART_1_CPutString("No value given!\r\n");
			break;
		// Backward at speed 0-254
		case 'B':
		case 'b':
			if (data = UART_1_szGetParam())
			{
				UART_1_CPutString("Moving backwards >"); 
				val = convert(data); 
				UART_1_PutString(data);             // Print out command
				UART_1_CPutString("<\r\n");
				PRT1DR |= 0x04;
				DAC8_1_WriteBlind(val);
			}
			else 
				UART_1_CPutString("No value given!\r\n");
			break;
		// Request drive encoder count
		case 'D':
		case 'd':
			UART_1_PutString(ltoa(TX,glblCount,10));
			UART_1_PutCRLF();
			break;
		// Ping the Drive PSoC returns 'D'
		case 'I':
		case 'i':
			UART_1_PutChar('D');
			break;
		// Invalid command given
		default:
			UART_1_CPutString("Invalid Command: >");
			UART_1_PutChar(cmd);
			UART_1_CPutString("<\n\r");
			break;
	}
}

// Encoder count updates on GPIO interrupts
void PSoC_GPIO_ISR_C(void)
{
	curPrt = (PRT1DR & (OpEncA_MASK | OpEncB_MASK));		// Setting prevPort to only bits 1[4] and 1[5]
															// of PRT1DR
		
	if ((prevPrt == 0x00) && (curPrt == 0x10))	// If prevPort is 0x00 and then after the interrupt curPrt is
												// 0x10 then A is high and B is low which means you wanted to
												// increment by turning clockwise and hitting a rising edge on A
	{
		// Increasing the count when clockwise turn interrupt occurred
		glblCount++;
	}
	else if ((prevPrt == 0x00) && (curPrt == 0x20))	// If prevPort is 0x00 and then after the interrupt curPrt is
													// 0x20 then B is high and A is low which means you wanted to
													// decrement by turning counterclockwise and hitting 
													// a rising edge on B
	{
		// Decreasing the count when the counterclockwise interrupt occurred
		glblCount--;
	}
}