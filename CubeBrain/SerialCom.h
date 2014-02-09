/*
 * SerialComFunctions.h
 *
 * Created: 5/2/2012 2:34:20 PM
 *  Author: Jonas
 */ 


#ifndef SERIALCOM_H_
#define SERIALCOM_H_


// Hardware Constants

#define RXD0_DATA			(UDR0)
#define RXD1_DATA			(UDR1)

#define SET_TxD0_FINISH		sbi(UCSR0A,6)
#define RESET_TXD0_FINISH	cbi(UCSR0A,6)

#define DEFAULT_RETURN_PACKET_SIZE 6
#define DYNAMIXEL_BROADCASTING_ID 0xfe


// Protocol related

#define MSG_START0	0x70
#define MSG_START1	0xFF
#define MSG_START2	0xFF
#define MSG_START3	0xFF
#define MSG_START4	0x71

#define MSG_END0	0x72
#define MSG_END1	0xFF
#define MSG_END2	0xFF
#define MSG_END3	0xFF
#define MSG_END4	0x73

#define WURFELSPEAK_ADDR_BASE						0x20 // = space
#define WURFELSPEAK_ADDR_BROADCAST					0x21 // = !

#define MESSAGE_LENGTH 20 // including checksum
#define MESSAGE_QUEUE_SIZE 5 // counted in messages


// Message Struct

typedef struct
{
	uint16_t sender;
	uint16_t recipient;
	byte command;
	byte databyte[4];
} message;


// --- Global Variables ---

volatile message messageQueue[MESSAGE_QUEUE_SIZE];
volatile byte messageQueueStart = 0;
volatile byte messageQueueEnd = 0;

// Functions

void searchMessageInRS232InterruptBuffer (void);

char messageQueueEmpty(void);
byte incrementMessageQueueStart (void);
byte incrementMessageQueueEnd (void);

void sendCubeStatusToBaseNew();
void sendCubeStatusToBase(char data2, char data3, char data4);
void sendMessageToBase(char command, char d1, char d2, char d3, char d4);
void sendMessage(uint16_t recipient, char command, char d1, char d2, char d3, char d4);

void TxRS232_String_Save (char *bData);
void TxRS232_Dec_Save(long int lLong);
void TxRS232_Hex_Save(byte bSentData);

#endif /* SERIALCOM_H_ */