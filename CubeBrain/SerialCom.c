/*
 * SerialComFunctions.c
 *
 * Created: 5/2/2012 2:35:44 PM
 *  Author: Jonas
 */ 

#include "../Communication/RS232.h"
#include "../Communication/RS485.h"
#include "IO.h"
#include "SerialCom.h"

/*
Write data to buffer whenever it arrives from the Dynamixel communications line.
*/

ISR (USART0_RX_vect)
{
	RxRS485(RXD0_DATA);
}

ISR(USART1_RX_vect)
{
	RxRS232(RXD1_DATA, time);
		
  searchMessageInRS232InterruptBuffer();
}

void searchMessageInRS232InterruptBuffer ()
{
	// if the buffer ends with the five bit terminator string,
	// begins with the five bit header string,
	// and doesn't have any of the forbidden characters in the command,
	// this was the checksum and we can ship the buffer over to parsing
	if (RS232_GetInterrupBufferEntry(6) == MSG_END0 &&
	RS232_GetInterrupBufferEntry(5) == MSG_END1 &&
	RS232_GetInterrupBufferEntry(4) == MSG_END2 &&
	RS232_GetInterrupBufferEntry(3) == MSG_END3 &&
	RS232_GetInterrupBufferEntry(2) == MSG_END4 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH) == MSG_START0 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-1) == MSG_START1 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-2) == MSG_START2 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-3) == MSG_START3 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-4) == MSG_START4 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-9) != MSG_END0 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-9) != MSG_END4 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-9) != MSG_START0 &&
	RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-9) != MSG_START4)
	{
  	// check if the checksum is correct
  	char computedChecksum = 0;
  	for (int i=0; i<MESSAGE_LENGTH-1; i++)
  	{
    	computedChecksum = computedChecksum ^ RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-i);
  	}
  	
  	// determine recipient address
  	byte msRecipient = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-5);
  	byte lsRecipient = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-6);
  	uint16_t recipient = combineTwoBytes(msRecipient, lsRecipient);

  	if (computedChecksum != RS232_GetInterrupBufferEntry(1))
  	{
    	// checksum doesn't compute
    	LEDMorse(5,0);
  	}
  	else if (recipient != myAddress && recipient != WURFELSPEAK_ADDR_BROADCAST)
  	{
    	// this cube is not recipient of this message
  	}
  	else
  	{
    	messageQueue[messageQueueEnd].recipient = recipient;
    	
    	byte msSender = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-7);
    	byte lsSender = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-8);
    	messageQueue[messageQueueEnd].sender = combineTwoBytes(msSender, lsSender);
    	
    	messageQueue[messageQueueEnd].command = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-9);
    	
    	messageQueue[messageQueueEnd].databyte[0] = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-10);
    	messageQueue[messageQueueEnd].databyte[1] = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-11);
    	messageQueue[messageQueueEnd].databyte[2] = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-12);
    	messageQueue[messageQueueEnd].databyte[3] = RS232_GetInterrupBufferEntry(MESSAGE_LENGTH-13);
    	
    	// increment messageQueueEnd
    	incrementMessageQueueEnd();
    	// if message queue is now full, drop the oldest element by incrementing the start pointer
    	if (messageQueueEnd == messageQueueStart) incrementMessageQueueStart();
  	}
	}
}


char messageQueueEmpty(void)
{
	if (messageQueueStart == messageQueueEnd) return 1;
	
	return 0;
}

byte incrementMessageQueueStart ()
{
	if (++messageQueueStart >= MESSAGE_QUEUE_SIZE) messageQueueStart = 0;
	return messageQueueStart;
}

byte incrementMessageQueueEnd ()
{
	if (++messageQueueEnd >= MESSAGE_QUEUE_SIZE) messageQueueEnd = 0;
	return messageQueueEnd;
}

void sendCubeStatusToBaseNew()
{
	sendCubeStatusToBase('*', '*', '*');
}

void sendCubeStatusToBase(char data2, char data3, char data4)
{
	char hs = 0b11000000;
	for (uint8_t i=0; i<6; i++)
	{
		if (heaterState[i] == true)
			hs |= (1 << i);
	}
	sendMessage(WURFELSPEAK_ADDR_BASE, 'p', hs, data2, data3, data4);
}

void sendMessageToBase(char command, char d1, char d2, char d3, char d4)
{
	sendMessage(WURFELSPEAK_ADDR_BASE, command, d1, d2, d3, d4);
}

void sendMessage(uint16_t recipient, char command, char d1, char d2, char d3, char d4)
{
	// allow for a 200 ms break between receiving data and sending a message
	while (time < RS232lastByteArrivedAtTime + 200);
	
	char msg[20];
	msg[0] = MSG_START0;
	msg[1] = MSG_START1;
	msg[2] = MSG_START2;
	msg[3] = MSG_START3;
	msg[4] = MSG_START4;

	msg[5] = (recipient >> 8);
	msg[6] = recipient & 0xff;
	msg[7] = (myAddress >> 8);
	msg[8] = myAddress & 0xff;;

	msg[9] = command;
	
	msg[10] = d1;
	msg[11] = d2;
	msg[12] = d3;
	msg[13] = d4;

	msg[14] = MSG_END0;
	msg[15] = MSG_END1;
	msg[16] = MSG_END2;
	msg[17] = MSG_END3;
	msg[18] = MSG_END4;

	char computedChecksum = 0;
	for (int i=0; i<MESSAGE_LENGTH-1; i++)
	{
		computedChecksum = computedChecksum ^ msg[i];
	}
	
	msg[19] = computedChecksum;

	TxRS232_String2(msg, 20);
  
  // if we are the imp module we also want to send what we are saying to the imp
  if (myOperatingMode == MODE_ELECTRICIMP_CUBE) TxRS485_String2(msg, 20);
}

void TxRS232_String_Save (char *bData)
{
	// allow for a 200 ms break between receiving data and sending a message
	while (time < RS232lastByteArrivedAtTime + 200);
	
	TxRS232_String(bData);
}

void TxRS232_Dec_Save(long int lLong)
{
	// allow for a 200 ms break between receiving data and sending a message
	while (time < RS232lastByteArrivedAtTime + 200);
	
	TxRS232_Dec(lLong);
}

void TxRS232_Hex_Save(byte bSentData)
{
	// allow for a 200 ms break between receiving data and sending a message
	while (time < RS232lastByteArrivedAtTime + 200);
	
	TxRS232_Hex(bSentData);
}