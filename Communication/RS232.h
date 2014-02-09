/*
 * RS232.h
 *
 * Created: 8/16/2012 2:25:32 PM
 *  Author: Jonas
 */ 


#ifndef RS232_H_
#define RS232_H_

#define F_CPU 16000000UL // needed for delay.h

#include <avr/io.h>
#include <util/delay.h>

#define ENABLE_BIT_DEFINITIONS

#include <inttypes.h>
typedef unsigned char byte;
typedef unsigned int word;

#define BIT_DATA_DIRECTION0  PD6
#define BIT_DATA_DIRECTION1  PD7

#define RS232_TXD			PORTD &= ~_BV(BIT_DATA_DIRECTION1), PORTD |= _BV(BIT_DATA_DIRECTION0)  //PORT_485_DIRECTION = 1
#define RS232_RXD			PORTD &= ~_BV(BIT_DATA_DIRECTION0), PORTD |= _BV(BIT_DATA_DIRECTION1)  //PORT_485_DIRECTION = 0

#define TXD1_READY			bit_is_set(UCSR1A,5) //(UCSR1A_Bit5)
#define TXD1_DATA			(UDR1)
#define RXD1_READY			bit_is_set(UCSR1A,7)
#define RXD1_DATA			(UDR1)

#define RS232_TRANSMISSION_MODE_DELAY	200 // how long to wait after transmission before RS232_RXD is called

#define SET_TxD1_FINISH		sbi(UCSR1A,6)
#define RESET_TXD1_FINISH	cbi(UCSR1A,6)
#define CHECK_TXD1_FINISH	bit_is_set(UCSR1A,6)

volatile byte RS232RxInterruptBuffer[256];
volatile byte RS232RxBufferReadPointer;
volatile byte RS232RxBufferWritePointer;
volatile uint32_t RS232lastByteArrivedAtTime;

void RS232Initialize(byte bBaudrate);
void TxRS232(byte bTxdData);
void RxRS232(byte bTxdData, uint32_t bTime);
byte RS232_GetInterrupBufferEntry(uint8_t bPos);

void TxRS232_Hex(byte bSentData);
void TxRS232_Hex2Byte(uint16_t bSentData);
void TxRS232_Dec(long int lLong);
void TxRS232_String(char *bData);
void TxRS232_String2(char *bData, int strlen);

#endif /* RS232_H_ */