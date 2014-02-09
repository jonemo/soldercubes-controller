/*
 * SerialComFunctions.h
 *
 * Created: 5/2/2012 2:34:20 PM
 *  Author: Jonas
 */ 


#ifndef SERIALCOM_H_
#define SERIALCOM_H_


// Hardware Constants
#define RS232_BAUD_RATE	34   //57600bps at 16MHz

#define RS485_TXD			PORTD &= ~_BV(BIT_RS485_DIRECTION1), PORTD |= _BV(BIT_RS485_DIRECTION0)  //PORT_485_DIRECTION = 1
#define RS485_RXD			PORTD &= ~_BV(BIT_RS485_DIRECTION0), PORTD |= _BV(BIT_RS485_DIRECTION1)  //PORT_485_DIRECTION = 0

#define TXD0_READY			bit_is_set(UCSR0A,5)
#define TXD0_DATA			(UDR0)
#define RXD0_READY			bit_is_set(UCSR0A,7)
#define RXD0_DATA			(UDR0)

#define RS485_TIMEOUT		50L // multiply by 10ms to get the time the read function will wait for the next byte

#define SET_TxD0_FINISH		sbi(UCSR0A,6)
#define RESET_TXD0_FINISH	cbi(UCSR0A,6)
#define CHECK_TXD0_FINISH	bit_is_set(UCSR0A,6)

#define CLEAR_BUFFER RS485RxBufferReadPointer = RS485RxBufferWritePointer
#define DEFAULT_RETURN_PACKET_SIZE 6
#define BROADCASTING_ID 0xfe

// --- Global Variables ---
volatile byte RS485RxInterruptBuffer[256];
volatile byte RS485Parameter[128];
volatile byte RS485RxBufferReadPointer;
volatile byte RS485RxBuffer[128];
volatile byte RS485TxBuffer[128];
volatile byte RS485RxBufferWritePointer;

volatile byte RS232RxInterruptBuffer[256];
volatile byte RS232RxBufferReadPointer;
volatile byte RS232RxBufferWritePointer;
volatile byte RS232TxBuffer[128];



// Functions
void InitializeRS485Serial (byte bBaudrate);

void TxRS485(byte bTxdData);
byte TxRS485_Packet(byte bID, byte bInstruction, byte bParameterLength);
byte RxRS485_Packet(byte bRxPacketLength);

void PrintBuffer(byte *bpPrintBuffer, byte bLength);
void PrintBaudrate(void);

#endif /* SERIALCOM_H_ */