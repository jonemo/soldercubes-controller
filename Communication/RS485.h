/*
 * RS485.h
 *
 * Created: 8/21/2012 11:12:02 PM
 *  Author: Jonas
 */ 


#ifndef RS485_H_
#define RS485_H_

	#define F_CPU 16000000UL // needed for delay.h

	#include <avr/io.h>
	#include <util/delay.h>

	#define ENABLE_BIT_DEFINITIONS

	// the following comes from Arduino's Wire library twi.h to enable "old-style" bit manipulation
	#ifndef sbi
	#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
	#endif

	#include <inttypes.h>
	typedef unsigned char byte;
	typedef unsigned int word;
	
	/*
		Version 1 
		
	#define BIT_RS485_DIRECTION0  PD4
	#define BIT_RS485_DIRECTION1  PD5
	
		End of Version 1 specific code
	*/
	
	/*
		Version 2 
	*/
	#define BIT_RS485_DIRECTION0  PD5
	#define BIT_RS485_DIRECTION1  PD4
	/* 
		End of Version 2 specific code
	*/
	
	#define CHECK_TXD0_FINISH	bit_is_set(UCSR0A,6)
	#define RS485_TIMEOUT		200L //100L // multiply by 10us to get the time the read function will wait for the next byte
	
	#define RS485_TXD			PORTD &= ~_BV(BIT_RS485_DIRECTION1), PORTD |= _BV(BIT_RS485_DIRECTION0)  //PORT_485_DIRECTION = 1
	#define RS485_RXD			PORTD &= ~_BV(BIT_RS485_DIRECTION0), PORTD |= _BV(BIT_RS485_DIRECTION1)  //PORT_485_DIRECTION = 0
	
	#define TXD0_READY			bit_is_set(UCSR0A,5)
	#define TXD0_DATA			(UDR0)

	volatile byte RS485RxInterruptBuffer[256];
	volatile byte RS485RxBufferReadPointer;
	volatile byte RS485RxBufferWritePointer;

	void RS485Initialize(byte bBaudrate);
	void TxRS485(byte bTxdData);
	void RxRS485(byte bTxdData);
	
	byte RxRS485_Packet(byte bRxPacketLength, byte *RS485RxBuffer);
	byte TxRS485_Packet(byte bID, byte bInstruction, byte bParameterLength, byte *bParameters);

  void TxRS485_String2(char *bData, int strlen);

#endif /* RS485_H_ */