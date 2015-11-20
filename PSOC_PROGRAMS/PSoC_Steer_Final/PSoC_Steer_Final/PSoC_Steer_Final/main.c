//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "PSoCGPIOINT.h"
#include <string.h>
#include <stdlib.h>	

//#define ON "on"
//#define OFF "off"

#pragma interrupt_handler PSoC_GPIO_ISR_C
#define STEER_POT_CENTER 0x1E1
#define ENCODER_LEFT_BOUND -2300
#define ENCODER_RIGHT_BOUND 2300
#define POT_LEFT_BOUND 0x00FF
#define POT_RIGHT_BOUND 0x02EF

int val;
BOOL encoderFlag = FALSE;
int glblCount = 0;
unsigned int steerPotvalue = 0;
BYTE prevPrt;
BYTE curPrt;
BYTE turning = 0;
BYTE manTurn = 0;
BYTE useBrake = 0;
BYTE* cancelComm;
char on[] = "on";
char off[] = "off";
//char* data;


int convert(char* c);
int command_lookup(BYTE cmd);
void countEncoder(void );
int turnToCount(int count);
void sendSTOP(void );
void applyBrake(void );
void releaseBrake(void );
void turn(BYTE direction);
unsigned int getActuatorPosition(void );
unsigned int getSteerPotPosition(void );
void resetPotShaft(void );


void main(void)
{
	char* data;
	int reqCount = 0;
	char baudChar = 0xAA;
	//unsigned int potValue = 0;
	val = 0;
	
	
	M8C_EnableGInt ; 
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO); 	// Enable GPIO Interrupts (see m8c.h)
	
	UART_CmdReset();
	UART_IntCntl(UART_ENABLE_RX_INT);
	UART_Start(UART_PARITY_NONE);
	
	TX8_Start(TX8_PARITY_NONE);
	TX8_EnableInt();
	Actuator_Pot_Start(Actuator_Pot_HIGHPOWER);
	Steer_Pot_Start(Steer_Pot_HIGHPOWER);
	DUALADC_Start(DUALADC_HIGHPOWER);
	
	LCD_Start();
	LCD_Position(0,0);
	LCD_PrCString("Steering PSoC");
	
	
	
	UART_CPutString("Steer Program Start\r\n");
	//TX8_PutChar(baudChar);
	//DAC8_1_Start(DAC8_1_HIGHPOWER);
	//DAC8_1_WriteBlind(val);
		
	while (TRUE)
	{
		prevPrt = (PRT1DR & (OpEncA_MASK | OpEncB_MASK)); 
		if (!(PRT1DR & ESTOP_MASK)){
			UART_CPutString("ESTOP");
			UART_PutCRLF();
			if (useBrake)applyBrake();
			sendSTOP();
			while (!(PRT1DR & ESTOP_MASK));
			if(useBrake)releaseBrake();
			UART_CPutString("RESUME");
			UART_PutCRLF();
		}
		if ((reqCount == glblCount) && turning) 
		{
			sendSTOP();
		}
		//In the future try to get protection for every method of turning with pot values
		/*if (turning || manTurn)
		{
			potValue = getSteerPotPosition();
			if ((potValue >= POT_RIGHT_BOUND) || (potValue <= POT_LEFT_BOUND)) sendSTOP();
		}*/
		if(UART_bCmdCheck()) 
		{                    // Wait for command    
			if(data = UART_szGetParam()) 
			{
				//UART_PutString(data);
				reqCount = command_lookup(*data);
			}   
		UART_CmdReset();  // Reset command buffer     
		}
	}
}

int convert(char* c)
{
	int result = 0;
	int i = 0;
	char check;
	while(check = c[i])
	{
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

//Parses the command buffer when new command received
int command_lookup(BYTE cmd)
{
	BYTE* data;
	BYTE* TX;
	BYTE addr = 128;
	BYTE dir = 0;
	BYTE val = 0;
	BYTE checksum = 0;
	BYTE baud = 0xAA;
	int count = 0;
	cancelComm = 0;
	switch (cmd)
	{
		//turn to a specific count
		case 'T':
		case 't':
			if (data = UART_szGetParam())
			{
				count = atoi(data);
				if ((count < ENCODER_LEFT_BOUND) || (count > ENCODER_RIGHT_BOUND))
				{
					UART_CPutString("Count outside of bounds.\r\n");
					break;
				}
				else
				{
					UART_CPutString("Turning to >");
					UART_PutString(data);
					UART_CPutString(" degrees<\r\n");
					return turnToCount(count);
				}
			}
			else
			{
				UART_CPutString("No value given!\r\n");
			}
			break;
		//send steer count
		case 'S':
		case 's':
			//UART_CPutString("Sending steer encoder count\r\n");
			UART_PutString(itoa(TX,glblCount,10));
			UART_PutCRLF();
			UART_CmdReset();
			break;
		//manual turning right
		case 'E':
		case 'e':
			UART_CPutString("Turning Right\r\n");
			turn(1);
			UART_CmdReset();
			break;
		//manual turning left
		case 'Q':
		case 'q':
			UART_CPutString("Turning Left\r\n");
			turn(0);
			UART_CmdReset();
			break;
		//manual stopping
		case 'L':
		case 'l':
			UART_CPutString("Sending a manual STOP\r\n");
			sendSTOP();
			UART_CmdReset();
			break;
		//toggles the useBrake flag for estop purposes
		case 'U':
		case 'u':
			if (useBrake) useBrake = 0;
			else useBrake = 1;
			break;
		//sending the baud character
		case 'I':
		case 'i':
			UART_PutChar('S');
			TX8_PutChar(baud);		
			//resetPotShaft();
			//UART_CPutString("Shaft Reset\r\n");
			UART_CmdReset();
			break;
		//reset the position of the wheels to '0'
		case 'R':
		case 'r':
			resetPotShaft();
			UART_CPutString("Shaft Reset\r\n");
			UART_CmdReset();
			break;
		//Putting the brake on or off
		case 'h':
		case 'H':
			if (data = UART_szGetParam())
			{
				//brake on
				if (strcmp(data,on)==0){
					UART_CPutString("Brake ON\r\n");
					sendSTOP();
					applyBrake();
					UART_CmdReset();
				//brake off
				} else if (strcmp(data,off)==0){
					UART_CPutString("Brake OFF\r\n");
					sendSTOP();
					releaseBrake();
					UART_CmdReset();
				}else UART_CPutString("Invalid halt command!!!\r\n");
			}
			break;
		//Invalid command
		default:
			UART_CPutString("Invalid Command: >");
			UART_PutChar(cmd);
			UART_CPutString("<\n\r");
			UART_CmdReset();
			break;
	}
	return 0;
}

void PSoC_GPIO_ISR_C(void)
{
	//encoderFlag = TRUE;
	//countEncoder();
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


//A ___|-----|_____|-----|____
//B   ____|-----|_____|-----|____
void countEncoder(void )
{
	//char * TX;
	//encoderFlag = FALSE;
	curPrt = (PRT1DR &= (OpEncA_MASK | OpEncB_MASK));		// Setting prevPort to only bits 1[4] and 1[5]
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
	//LCD_Position(1,7);
	//LCD_PrHexInt(glblCount);
	//UART_PutString(itoa(TX,glblCount,10));
	//UART_PutCRLF();
	//PRT1DR ^= 0x01;
}

int turnToCount(int count)
{
	BYTE* TX;
	BYTE addr = 128;
	BYTE dir = 0;
	BYTE val = 88;
	BYTE checksum = 0;
	int i = 0;
	
	UART_CmdReset();
	if(turning){
		//for (i = 0; i < 56000; i++);
		sendSTOP();
		for (i = 0; i < 10000; i++);
	}
	//UART_CPutString("turning left to>");
	//UART_PutSHexInt(count);
	//UART_CPutString("<\r\n");
	//LCD_Control(LCD_DISP_CLEAR_HOME);
	
	//LCD_Position(0,0);
	//LCD_PrCString("Going Left!!");
	//dir = 0;
	turning = 1;
	if (count > glblCount)dir = 1;
	
	//else if (((count == 0)&&(glblCount > 0))||(count < glblCount))dir = 0;
	else if (count < glblCount)dir = 0;
	
	checksum = addr + dir + val;
	checksum = checksum & 0x7F;
	
	TX[0] = addr;
	TX[1] = dir;
	TX[2] = val;
	TX[3] = checksum;
	TX8_Write(TX,4);

	//Increasing count for testing without encoder
	/*while (glblCount != count)
	{
		prevPrt = (PRT1DR &= (OpEncA_MASK | OpEncB_MASK));  
		//if(encoderFlag)
		//{
			//countEncoder();	
		//}
	
	
		if(UART_bCmdCheck()) // command in receive buffer
		{ 
			//if(data = UART_szGetParam())
			//{
	            // Wait for command    		
				//sendSTOP();
			UART_CPutString("Got breakout command!!!\r\n");
			if (cancelComm = UART_szGetParam())
			{
				UART_CPutString("Received a command breaking out!!!\r\n");
				break;
			}
			//}
		}
	}*/
	//sendSTOP();
	return count;
}

void sendSTOP(void ){
	BYTE* TX;
	BYTE addr = 128;
	BYTE dir = 0;
	BYTE val = 0;
	BYTE checksum = 0;
	
	UART_CmdReset();
	//val = *data;
	//checksum = addr + dir + val;
	//checksum = checksum & 0x7F;
	//checksum &= (addr | dir | val);
	manTurn = 0;
	turning = 0;
	TX[0] = addr;
	TX[1] = dir;
	TX[2] = val;
	TX[3] = checksum;
	TX8_Write(TX,4);
	UART_CPutString("Stopping!!\r\n");
}

void applyBrake(void )
{
	BYTE* TX;
	BYTE addr = 130;
	BYTE dir = 1;
	BYTE val = 120;
	BYTE checksum = 0;
	
	UART_CmdReset();
	checksum = addr + dir + val;
	checksum = checksum & 0x7F;
	TX[0] = addr;
	TX[1] = dir;
	TX[2] = val;
	TX[3] = checksum;
	TX8_Write(TX,4);
	
	UART_CPutString("Braking!!\r\n");
	while (getActuatorPosition() < 900);
	TX[2] = 0;
	checksum = addr + dir;
	checksum = checksum & 0x7F;
	TX[3] = checksum;
	TX8_Write(TX,4);
	 // 96
}

void releaseBrake(void )
{
	BYTE* TX;
	BYTE addr = 130;
	BYTE dir = 0;
	BYTE val = 120;
	BYTE checksum = 0;
	
	UART_CmdReset();
	checksum = addr + dir + val;
	checksum = checksum & 0x7F;
	TX[0] = addr;
	TX[1] = dir;
	TX[2] = val;
	TX[3] = checksum;
	TX8_Write(TX,4);
	
	UART_CPutString("Unbraking!!\r\n");
	while (getActuatorPosition() > 100);
	TX[2] = 0;
	checksum = addr + dir;
	checksum = checksum & 0x7F;
	TX[3] = checksum;
	TX8_Write(TX,4);
	
	UART_CPutString("FREEEEEDOM!!\r\n");
}

unsigned int getActuatorPosition(void )
{
	unsigned int result = 0;
	DUALADC_GetSamples(2);
	// Wait for data to be ready
	while(DUALADC_fIsDataAvailable ()==0);
		
	// Get Data and clear flag
	result=DUALADC_iGetData2ClearFlag();
	return result;
}

unsigned int getSteerPotPosition(void )
{
	unsigned int result = 0;
	DUALADC_GetSamples(2);
	// Wait for data to be ready
	while(DUALADC_fIsDataAvailable ()==0);
		
	// Get Data and clear flag
	result=DUALADC_iGetData1ClearFlag();
	return result;
}

void turn(BYTE direction)
{
	BYTE* TX;
	BYTE addr = 128;
	BYTE dir;
	BYTE val = 80;
	BYTE checksum = 0;
	int i  = 0;
	
	UART_CmdReset();
	//val = *data;
	if(manTurn){
		//for (i = 0; i < 56000; i++);
		sendSTOP();
		for (i = 0; i < 10000; i++);
	}
	manTurn = 1;
	dir = direction;
	checksum = addr + dir + val;
	checksum = checksum & 0x7F;
	//checksum &= (addr | dir | val);
	TX[0] = addr;
	TX[1] = dir;
	TX[2] = val;
	TX[3] = checksum;
	TX8_Write(TX,4);
}

void resetPotShaft(void)
{
	BYTE* TX;
	BYTE addr = 128;
	BYTE dir = 0;
	BYTE val = 80;
	BYTE checksum = 0;
	int i = 0;
	
	UART_CmdReset();
	//UART_CPutString("turning left to>");
	//UART_PutSHexInt(count);
	//UART_CPutString("<\r\n");
	//LCD_Control(LCD_DISP_CLEAR_HOME);
	
	//LCD_Position(0,0);
	//LCD_PrCString("Going Left!!");
	//dir = 0;
	steerPotvalue = getSteerPotPosition();
	if (steerPotvalue < STEER_POT_CENTER)dir = 1;
	
	//else if (((count == 0)&&(glblCount > 0))||(count < glblCount))dir = 0;
	else if (steerPotvalue > STEER_POT_CENTER)dir = 0;
	
	checksum = addr + dir + val;
	checksum = checksum & 0x7F;
	
	TX[0] = addr;
	TX[1] = dir;
	TX[2] = val;
	TX[3] = checksum;
	TX8_Write(TX,4);
	
	while (steerPotvalue != STEER_POT_CENTER)
	{
		steerPotvalue = getSteerPotPosition();
	}
	sendSTOP();
	glblCount = 0;
}
	