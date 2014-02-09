/*
 * RS232.c
 *
 * Created: 8/16/2012 2:25:09 PM
 *  Author: Jonas
 */ 

#include "RS232.h"

/*

*/
void RS232Initialize(byte bBaudrate)
{
	// set the data direction of the data direction selectors
	DDRD |= _BV(BIT_DATA_DIRECTION0);
	DDRD |= _BV(BIT_DATA_DIRECTION1);
	
	RS232_RXD;
	
	// set up the serial port with all its parameters
	UBRR1H = 0;
	UBRR1L = bBaudrate;
	UCSR1A = 0x02;
	UCSR1B = 0x18;
	UCSR1B |= _BV(7); //used to be sbi( UCSR1B, 7 ); // RxD interrupt enable
	UCSR1C = 0x06;
	UDR1 = 0xFF;
	UCSR1A |= _BV(6); //used to be sbi(UCSR1A,6); //SET_TXD1_FINISH; // Note. set 1, then 0 is read
	
	// enable pullup on RXD
	PORTD |= _BV(PD2);
	
	// RS232 RxBuffer Clearing
	RS232RxBufferReadPointer = RS232RxBufferWritePointer = 0;
	RS232lastByteArrivedAtTime = 0;
}

/*
TxRS232() send data to USART 1.
*/
void TxRS232(byte bTxdData)
{
	while(!TXD1_READY);
	TXD1_DATA = bTxdData;
}

/*
Write received byte to interrupt read buffer
*/
void RxRS232(byte bTxdData, uint32_t bTime)
{
	RS232RxInterruptBuffer[(RS232RxBufferWritePointer++)] = bTxdData;
	RS232lastByteArrivedAtTime = bTime;
}

byte RS232_GetInterrupBufferEntry(byte bPos)
{
	return RS232RxInterruptBuffer[(byte)(RS232RxBufferWritePointer-bPos)];
}

/*
TXD32Dex() change data to decimal number system
*/
void TxRS232_Dec(long int lLong)
{
	// switch on data transmit control line
	RS232_TXD;
	
	byte bPrinted = 0;
	uint16_t lTmp;
	
	if(lLong < 0)
	{
		lLong = -lLong;
		TxRS232('-');
	}
	volatile long int lDigit = 100000L;
	
	while (lDigit > 0)
	{
		lTmp = (byte)(lLong/lDigit);
		if(lTmp)
		{
			TxRS232(((byte)lTmp)+'0');
			bPrinted = 1;
		}
		else
		{
			if (bPrinted)
			{
				TxRS232( ((byte)lTmp)+'0' );
			}
		}
		
		lLong -= ((long)lTmp)*lDigit;
		lDigit = lDigit/10;
	}
	lTmp = (byte)(lLong/lDigit);
	
	//if(lTmp) TxRS232(((byte)lTmp)+'0');
	
	// make sure everything is sent
	while(!TXD1_READY);
	
	// add 300us because otherwise the oscilloscope shows the in/out
	// control line going down before transmission finishes
	_delay_us(RS232_TRANSMISSION_MODE_DELAY);
	
	// switch off transmit mode, go into receive mode
	RS232_RXD;
}

/*
TxRS232_Hex() print data seperatly.
ex> 0x1a -> '1' 'a'.
*/
void TxRS232_Hex(byte bSentData)
{
	// switch on data transmit control line
	RS232_TXD;
	
	byte bTmp;

	bTmp =((byte)(bSentData>>4)&0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	bTmp =(byte)(bSentData & 0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	
	// make sure everything is sent
	while(!TXD1_READY);
	
	// add 300us because otherwise the oscilloscope shows the in/out
	// control line going down before transmission finishes
	_delay_us(RS232_TRANSMISSION_MODE_DELAY);
	
	// switch off transmit mode, go into receive mode
	RS232_RXD;
}

/*

*/
void TxRS232_Hex2Byte(uint16_t bSentData)
{
	// switch on data transmit control line
	RS232_TXD;
	
	// split the number into two bytes
	byte lsByte = bSentData & 0xff;
	byte msByte = (bSentData >> 8);
	
	byte bTmp;
	
	// send everything out
	bTmp =((byte)(msByte>>4)&0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	bTmp =(byte)(msByte & 0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	while(!TXD1_READY);
	
	bTmp =((byte)(lsByte>>4)&0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	bTmp =(byte)(lsByte & 0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	while(!TXD1_READY);
	
	// add 300us because otherwise the oscilloscope shows the in/out
	// control line going down before transmission finishes
	_delay_us(RS232_TRANSMISSION_MODE_DELAY);
	
	// switch off transmit mode, go into receive mode
	RS232_RXD;
}

/*
TxRS232_String() prints data in ASCII code.
*/
void TxRS232_String(char *bData)
{
	// switch on data transmit control line
	RS232_TXD;
	
	// move string into output buffer one by one
	while(*bData)
	{
		TxRS232(*bData++);
	}
	
	// make sure everything is sent
	while(!TXD1_READY);
	
	// add 300us because otherwise the oscilloscope shows the in/out
	// control line going down before transmission finishes
	_delay_us(RS232_TRANSMISSION_MODE_DELAY);
	
	// switch off transmit mode, go into receive mode
	RS232_RXD;
}

/*
TxRS232_String() prints data in ASCII code.
We needed to make this a separate function because status codes can contain NULL characters and those make the while(*bData) line in the original TxRS232_String function assume the string ends there.
*/
void TxRS232_String2(char *bData, int strlen)
{
	// switch on data transmit control line
	RS232_TXD;
	
	// move string into output buffer one by one
	for (int i=0; i<strlen; i++)
	{
		//TxRS232_String(" 0x"); TxRS232_Hex(bData[i]); TxRS232_String(" ");
		TxRS232(bData[i]);
	}
	// make sure everything is sent
	while(!TXD1_READY);
	// add 300us because otherwise the oscilloscope shows the in/out
	// control line going down before transmission finishes
	_delay_us(RS232_TRANSMISSION_MODE_DELAY);
	
	// switch off transmit mode, go into receive mode
	RS232_RXD;
}