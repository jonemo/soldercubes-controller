/*
 * PotentiometerAlignment.c
 *
 * Created: 8/5/2012 7:08:50 PM
 *  Author: Jonas
 */ 


#define ENABLE_BIT_DEFINITIONS
#define F_CPU 16000000UL
#define RS232_BAUD_RATE	34   //57600bps at 16MHz

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdio.h>
#include <util/delay.h>
#include <util/delay_basic.h>

// the following comes from Arduino's Wire library twi.h
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

typedef unsigned char byte;
typedef unsigned int word;

#define WAIT_PER_CYCLE 50	// milliseconds of doing nothing per cycle
#define SERIAL_OUTPUT_EVERY_X_CYCLES 20

volatile uint16_t myAddress; // the address of this cube (used in RS232 communication, not in RS485 communication)
volatile byte heaterState[6];
volatile byte dynamixelAddress = 1;


typedef struct  
{
	uint16_t sender;
	uint16_t recipient;
	byte command;
	byte databyte[4];
} message;

#include "IO.c"
#include "SerialCom.c"
#include "Dynamixel.c"


volatile uint16_t motorPos;
volatile uint16_t targetPos;
volatile int counter = 0;

int main(void)
{	
	// On the debug system we needed some extra time for the crystal to swing in
	_delay_ms(1000);
	
	//Port In/Out Direction Definition
	InitializeIOPins(); 
	
	// set RS485 and inter-cube communication to Listen State.
	RS485_RXD; 
	DATA_RXD;
	
	// set up baud rates and similar stuff
	InitializeRS485Serial( 0x01 ); // baud rate = 1MHz, Rx interrupt driven
	InitializeRS232Serial( RS232_BAUD_RATE ); // not interrupt driven
	
	//Enable Interrupt -- Compiler Function
	sei();
	
	_delay_ms(200);
	
	SetStatusLEDs(0,1);
	
	// initialize Dynamixels
	initializeDynamixels();
		
	_delay_ms(100);
	
	motorPos = DynamixelGetMotorPosition(dynamixelAddress);
	targetPos = motorPos;
	
	while(1)
    {
		motorPos = DynamixelGetMotorPosition(dynamixelAddress);
		
		if (counter % SERIAL_OUTPUT_EVERY_X_CYCLES == 0) 
		{
			TxRS232_String("MotorPos: "); TxRS232_Dec(motorPos); TxRS232_String(" = 0x"); TxRS232_Hex(motorPos); TxRS232_String("\r\n");
		}
		
		counter++;		
		
		/************************************************************************/
		/* Set motor LED according to whether this is a save interval position  */
		/************************************************************************/
		
		if ( (motorPos > 0 && motorPos < 102) ||
			(motorPos > 307 && motorPos < 409) ||
			(motorPos > 615 && motorPos < 717) ||
			(motorPos > 922 && motorPos < 1024) )	
		{
			SetStatusLEDs(0,1); // green
		}				
		else
		{
			SetStatusLEDs(1,0); // red
		}
		
		
		/************************************************************************/
		/* +/- on the serial port turn this thing                               */
		/************************************************************************/
		
		if (RS232RxBufferWritePointer != RS232RxBufferReadPointer) 
		{
			RS232RxBufferReadPointer++;
			
			if (RS232RxInterruptBuffer[RS232RxBufferReadPointer] == '+')
			{
				TxRS232_String("Incrementing Goal Position:"); TxRS232_Dec(targetPos); TxRS232_String(" --> ");
				targetPos++;
				if (targetPos > 1024) targetPos = 0;
				DynamixelSetGoalPosition(dynamixelAddress, targetPos);
				TxRS232_Dec(targetPos); TxRS232_String("\r\n");
			}
			else if (RS232RxInterruptBuffer[RS232RxBufferReadPointer] == '-')
			{
				TxRS232_String("Decrementing Goal Position:"); TxRS232_Dec(targetPos); TxRS232_String(" --> ");
				targetPos--;
				if (targetPos < 0) targetPos = 1024;
				DynamixelSetGoalPosition(dynamixelAddress, targetPos);
				TxRS232_Dec(targetPos); TxRS232_String("\r\n");
				
			}
			else if (RS232RxInterruptBuffer[RS232RxBufferReadPointer] == '.')
			{
				TxRS232_String("Storing Initial Encoder Count: "); TxRS232_Dec(motorPos); TxRS232_String(" = 0x"); TxRS232_Hex(motorPos); TxRS232_String("\r\n");
				setInitialEncoderCount(motorPos);
			}							
		}
		
		_delay_ms(WAIT_PER_CYCLE);
    }
}