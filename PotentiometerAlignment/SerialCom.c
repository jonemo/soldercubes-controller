/*
 * SerialComFunctions.c
 *
 * Created: 5/2/2012 2:35:44 PM
 *  Author: Jonas
 */ 

#include "IO.h"
#include "SerialCom.h"

/*
Write data to buffer whenever it arrives from the Dynamixel communications line.
*/

ISR (USART0__RX_vect)
{
	RS485RxInterruptBuffer[(RS485RxBufferWritePointer++)] = RXD0_DATA;
}

ISR(USART1_RX_vect)
{
	RS232RxInterruptBuffer[(RS232RxBufferWritePointer++)] = RXD1_DATA;
}


/*
SerialInitialize() set Serial Port to initial state.
Vide Mega128 Data sheet about Setting bit of register.
SerialInitialize() needs port, Baud rate, Interrupt value.

*/
void InitializeRS485Serial (byte bBaudrate) 
{
	UBRR0H = 0; 
	UBRR0L = bBaudrate;
	UCSR0A = 0x02;  
	UCSR0B = 0x18;
	sbi( UCSR0B, RXCIE0 ); // RxD interrupt enable
	UCSR0C = 0x06; 
	UDR0 = 0xFF;
	sbi(UCSR0A,6);//SET_TXD0_FINISH; // Note: set 1, then 0 is read
	
	//RS485 RxBuffer Clearing. Note that the buffers are volatile aka global variables
	RS485RxBufferReadPointer = RS485RxBufferWritePointer = 0; 
}

/*
TxRS485_Packet() send data to RS485.
TxRS485_Packet() needs 3 parameter; ID of Dynamixel, Instruction byte, Length of parameters.
TxRS485_Packet() return length of Return packet from Dynamixel.
*/
byte TxRS485_Packet(byte bID, byte bInstruction, byte bParameterLength)
{	
	byte bCount, bCheckSum, bPacketLength;

	RS485TxBuffer[0] = 0xff;
	RS485TxBuffer[1] = 0xff;
	RS485TxBuffer[2] = bID;

	RS485TxBuffer[3] = bParameterLength+2; //Length(Paramter,Instruction,Checksum)
	RS485TxBuffer[4] = bInstruction;
	for(bCount = 0; bCount < bParameterLength; bCount++)
	{
		RS485TxBuffer[bCount+5] = RS485Parameter[bCount];
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
	
	CLEAR_BUFFER;
	RS485_RXD;
	
	return(bPacketLength);
}

/*
RxRS485_Packet() read data from buffer.
RxRS485_Packet() need a Parameter; Total length of Return Packet.
RxRS485_Packet() return Length of Return Packet.
*/

byte RxRS485_Packet(byte bRxPacketLength)
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
		RS485RxBuffer[bCount] = RS485RxInterruptBuffer[RS485RxBufferReadPointer++];
	}
	
	bLength = bCount;
	bChecksum = 0;
	
	if(RS485TxBuffer[2] != BROADCASTING_ID)
	{
		if(bTimeout && bRxPacketLength != 255)
		{
			TxRS232_String("[Error: RxD Timeout]\r\n");
			CLEAR_BUFFER;
			return 0;
		}

		if(bLength > 3) //checking is available.
		{
			if(RS485RxBuffer[0] != 0xff || RS485RxBuffer[1] != 0xff )
			{
				TxRS232_String("[Error: Wrong Header]\r\n");
				CLEAR_BUFFER;
				return 0;
			}
			
			if(RS485RxBuffer[2] != RS485TxBuffer[2] )
			{
				TxRS232_String("[Error: TxID != RxID]\r\n");
				CLEAR_BUFFER;
				return 0;
			}
			
			if(RS485RxBuffer[3] != bLength-4)
			{
				TxRS232_String("[Error: Wrong Length]\r\n");
				CLEAR_BUFFER;
				return 0;
			}
			
			for(bCount = 2; bCount < bLength; bCount++) 
			{
				bChecksum += RS485RxBuffer[bCount];
			}
						
			if(bChecksum != 0xff)
			{
				
				TxRS232_String("[Error: Wrong CheckSum]\r\n");
				CLEAR_BUFFER;
				return 0;
			}
		}
	}
	return bLength;
}

/*
TxRS485() send data to USART 0.
*/
void TxRS485(byte bTxdData)
{
	while(!TXD0_READY);
	TXD0_DATA = bTxdData;
}
