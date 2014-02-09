/*
 * IOFunctions.c
 *
 * Created: 5/2/2012 2:18:58 PM
 *  Author: Jonas
 */ 

#include "IO.h"

void InitializeIOPins (void)
{
	DDRA = DDRB = DDRC = DDRD = 0;  //Set all port to input direction first.
	PORTA = PORTB = PORTC = PORTD = 0x00; //PortData initialize to 0
	
	//cbi(SFIOR,2); //All Port Pull Up ready - doesn't work
	// communications pins
	DDRD |= _BV(BIT_RS485_DIRECTION0);
	DDRD |= _BV(BIT_RS485_DIRECTION1);
	DDRD |= _BV(BIT_DATA_DIRECTION0);
	DDRD |= _BV(BIT_DATA_DIRECTION1); //set output the bit RS485direction
	
	// the heater control pins
	DDRC |= _BV(BIT_HEAT_1);
	DDRC |= _BV(BIT_HEAT_2);
	DDRC |= _BV(BIT_HEAT_3);
	DDRC |= _BV(BIT_HEAT_4);
	DDRC |= _BV(BIT_HEAT_5);
	DDRC |= _BV(BIT_HEAT_6);
	
	SwitchHeatOff(1);
	SwitchHeatOff(2);
	SwitchHeatOff(3);
	SwitchHeatOff(4);
	SwitchHeatOff(5);
	SwitchHeatOff(6);
	
	
	// I2C
	DDRC |= _BV(PC0);
	DDRC |= _BV(PC1);
	PORTC |= _BV(PC0);
	PORTC |= _BV(PC1);
	
	// LEDs
	DDRB |= _BV(BIT_LED_ORANGE);
	DDRB |= _BV(BIT_LED_GREEN);
	PORTB |= _BV(BIT_LED_ORANGE); // LEDs are off when the pin is low
	PORTB |= _BV(BIT_LED_GREEN);
	
}

void SetStatusLEDs (int orange, int green) {
	
	// remember that LEDs are on when the pin is low and off when the pin is high.
	
	if (orange == 0) {
		PORTB |= _BV(BIT_LED_ORANGE);
	} else {
		PORTB &= ~_BV(BIT_LED_ORANGE);
	}
	
	if (green == 0) {
		PORTB |= _BV(BIT_LED_GREEN);
	} else {
		PORTB &= ~_BV(BIT_LED_GREEN);
	}
}

void LEDMorse(byte cntShort, byte cntLong)
{
	_delay_ms(400);
	for (byte i=0; i<cntShort; i++)
	{
		SetStatusLEDs(0,1);
		_delay_ms(150);
		SetStatusLEDs(0,0);
		_delay_ms(150);
	}
	for (byte i=0; i<cntLong; i++)
	{
		SetStatusLEDs(0,1);
		_delay_ms(400);
		SetStatusLEDs(0,0);
		_delay_ms(150);
	}
	_delay_ms(400);
}

void SwitchHeatOn (int faceNo) {
	if (faceNo == 1) {
		PORTC |= _BV(BIT_HEAT_1);
	}
	else if (faceNo == 2) {
		PORTC |= _BV(BIT_HEAT_2);
	}
	else if (faceNo == 3) {
		PORTC |= _BV(BIT_HEAT_3);
	}
	else if (faceNo == 4) {
		PORTC |= _BV(BIT_HEAT_4);
	}
	else if (faceNo == 5) {
		PORTC |= _BV(BIT_HEAT_5);
	}
	else if (faceNo == 6) {
		PORTC |= _BV(BIT_HEAT_6);
	}
}

void SwitchHeatOff (int faceNo) {
	if (faceNo == 1) {
		PORTC &= ~_BV(BIT_HEAT_1);
	}
	else if (faceNo == 2) {
		PORTC &= ~_BV(BIT_HEAT_2);
	}
	else if (faceNo == 3) {
		PORTC &= ~_BV(BIT_HEAT_3);
	}
	else if (faceNo == 4) {
		PORTC &= ~_BV(BIT_HEAT_4);
	}
	else if (faceNo == 5) {
		PORTC &= ~_BV(BIT_HEAT_5);
	}
	else if (faceNo == 6) {
		PORTC &= ~_BV(BIT_HEAT_6);
	}
}