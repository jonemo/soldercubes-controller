/*
 * RS485.c
 *
 * Created: 8/21/2012 11:11:39 PM
 *  Author: Jonas
 */ 

#include "RS485.h"
#include "RS232.h" // for writing error messages

/*
	Sets up the serial port of the Atmega and 
*/
void RS485Initialize(byte bBaudrate)
{
	UBRR0H = 0;
	UBRR0L = bBaudrate;
	UCSR0A = 0x02;
	UCSR0B = 0x18;
	sbi( UCSR0B, RXCIE0 ); // RxD interrupt enable
	UCSR0C = 0x06;
	UDR0 = 0xFF;
	sbi(UCSR0A,6);//SET_TXD0_FINISH; // Note: set 1, then 0 is read
	
	// communication direction pins
	DDRD |= _BV(BIT_RS485_DIRECTION0);
	DDRD |= _BV(BIT_RS485_DIRECTION1);
	
	// enable pullup on RXD
	PORTD |= _BV(PD0);
	
	//RS485 RxBuffer Clearing. Note that the buffers are volatile aka global variables
	RS485RxBufferReadPointer = RS485RxBufferWritePointer = 0;
}

/*
TxRS485() send data to USART 0.
*/
void TxRS485(byte bTxdData)
{
	while(!TXD0_READY);
	TXD0_DATA = bTxdData;
}

/*
Add byte to the receive buffer. Triggered by interrupt handler.
*/
void RxRS485(byte bTxdData)
{
	RS485RxInterruptBuffer[(RS485RxBufferWritePointer++)] = bTxdData;
}

/*
TxRS485_Packet() send data to RS485.
TxRS485_Packet() needs 3 parameter; ID of Dynamixel, Instruction byte, Length of parameters.
TxRS485_Packet() return length of Return packet from Dynamixel.
*/
byte TxRS485_Packet(byte bID, byte bInstruction, byte bParameterLength, byte *bParameters)
{
	byte bCount, bCheckSum, bPacketLength;
	byte RS485TxBuffer[128];

	RS485TxBuffer[0] = 0xff;
	RS485TxBuffer[1] = 0xff;
	RS485TxBuffer[2] = bID;

	RS485TxBuffer[3] = bParameterLength+2; //Length(Paramter,Instruction,Checksum)
	RS485TxBuffer[4] = bInstruction;
	for(bCount = 0; bCount < bParameterLength; bCount++)
	{
		RS485TxBuffer[bCount+5] = bParameters[bCount];
	}
	bCheckSum = 0;
	bPacketLength = bParameterLength+4+2;
	for(bCount = 2; bCount < bPacketLength-1; bCount++) //except 0xff,checksum
	{
		bCheckSum += RS485TxBuffer[bCount];
	}
	RS485TxBuffer[bCount] = ~bCheckSum; //Writing Checksum with Bit Inversion

	RS485_TXD;
	for(bCount = 0; bCount < bPacketLength; bCount++)
	{
		sbi(UCSR0A,6);//SET_TXD0_FINISH;
		TxRS485(RS485TxBuffer[bCount]);
	}
	while(!CHECK_TXD0_FINISH); //Wait until TXD Shift register empty
	RS485_RXD;
	
	return(bPacketLength);
}

/*
RxRS485_Packet() read data from buffer.
RxRS485_Packet() need a Parameter; Total length of Return Packet.
RxRS485_Packet() return Length of Return Packet.
*/

byte RxRS485_Packet(byte bRxPacketLength, byte *RS485RxBuffer)
{
	unsigned long ulCounter;
	byte bCount = 0, bLength = 0, bChecksum = 0;
	byte bTimeout = 0;
	
	for( bCount = 0; bCount < bRxPacketLength; bCount++ )
	{
		ulCounter = 0;
		while(RS485RxBufferReadPointer == RS485RxBufferWritePointer)
		{
			_delay_us(10);
			
			if(ulCounter++ > RS485_TIMEOUT)
			{
				bTimeout = 1;
				break;
			}
		}

		if( bTimeout ) break;
		RS485RxBuffer[bCount] = RS485RxInterruptBuffer[(RS485RxBufferReadPointer)++];
	}
	
	bLength = bCount;
	bChecksum = 0;
	

	if(bTimeout && bRxPacketLength != 255)
	{
		TxRS232_String("[Err: RxD Timeout]\r\n");
		// clear buffer;
		RS485RxBufferReadPointer = RS485RxBufferWritePointer;
		return 0;
	}

	if(bLength > 3) //checking is available.
	{
		if(RS485RxBuffer[0] != 0xff || RS485RxBuffer[1] != 0xff )
		{
			TxRS232_String("[Err: Wrong Header]\r\n");
			// clear buffer;
			RS485RxBufferReadPointer = RS485RxBufferWritePointer;
			return 0;
		}
			
		if(RS485RxBuffer[3] != bLength-4)
		{
			TxRS232_String("[Err: Wrong Length]\r\n");
			// clear buffer;
			RS485RxBufferReadPointer = RS485RxBufferWritePointer;
			return 0;
		}
			
		for(bCount = 2; bCount < bLength; bCount++)
		{
			bChecksum += RS485RxBuffer[bCount];
		}
			
		if(bChecksum != 0xff)
		{
				
			TxRS232_String("[Err: Wrong CheckSum]\r\n");
			// clear buffer;
			RS485RxBufferReadPointer = RS485RxBufferWritePointer;
			return 0;
		}
	}

	return bLength;
}


/*
This is the equivalent to TxRS232_String() only used by the electric imp module
*/
void TxRS485_String2(char *bData, int strlen)
{
  // switch on data transmit control line
  RS485_TXD; // not strictly needed because when dealing with the imp we are not on a bus
  
  // move string into output buffer one by one
  for (int i=0; i<strlen; i++)
  {
    //TxRS232_String(" 0x"); TxRS232_Hex(bData[i]); TxRS232_String(" ");
    TxRS485(bData[i]);
  }
  // make sure everything is sent
  while(!CHECK_TXD0_FINISH);
  // add 300us because otherwise the oscilloscope shows the in/out
  // control line going down before transmission finishes
  _delay_us(RS232_TRANSMISSION_MODE_DELAY);
  
  // switch off transmit mode, go into receive mode
  RS485_RXD; // not strictly needed because when dealing with the imp we are not on a bus
}