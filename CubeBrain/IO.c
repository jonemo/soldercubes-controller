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
	
	// set heater control pins to output
	DDRC |= _BV(BIT_HEAT_1);
	DDRC |= _BV(BIT_HEAT_2);
	DDRA |= _BV(BIT_HEAT_3);
	DDRB |= _BV(BIT_HEAT_4);
	DDRC |= _BV(BIT_HEAT_5);
	DDRC |= _BV(BIT_HEAT_6);
	
  // set sensor switch (sensors on/off) to output
  // (note that individual sensor pins are inputs as per default above)
	#ifdef VERSION2
		DDRC |= _BV(BIT_SENSOR_ON);
	#endif
	
	// default is off for all heaters
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
  
  SetStatusLEDs(0,0);
}

void SetStatusLEDs (int orange, int green) {
	
	// remember that LEDs are on when the pin is low and off when the pin is high.
  // if this is an led mode cube, the green led is tied to the green channel of the big led (out of pins :( ) and the red one needs to be 
  // controlled via the ocr0a register as pulse width modulated
	
	if (orange == 0) 
  {
    PORTB |= _BV(BIT_LED_ORANGE);
	} 
  else 
  {
    PORTB &= ~_BV(BIT_LED_ORANGE); 
	}
	
	if (green == 0) {
		if (myOperatingMode != MODE_LED_CUBE) PORTB |= _BV(BIT_LED_GREEN);
	} else {
		if (myOperatingMode != MODE_LED_CUBE) PORTB &= ~_BV(BIT_LED_GREEN);
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
		PORT_HEAT_1 |= _BV(BIT_HEAT_1);
	}
	else if (faceNo == 2) {
		PORT_HEAT_2 |= _BV(BIT_HEAT_2);
	}
	else if (faceNo == 3) {
		PORT_HEAT_3 |= _BV(BIT_HEAT_3);
	}
	else if (faceNo == 4) {
		PORT_HEAT_4 |= _BV(BIT_HEAT_4);
	}
	else if (faceNo == 5) {
		PORT_HEAT_5 |= _BV(BIT_HEAT_5);
	}
	else if (faceNo == 6) {
		PORT_HEAT_6 |= _BV(BIT_HEAT_6);
	}
}

void SwitchHeatOff (int faceNo) {
	if (faceNo == 1) {
		PORT_HEAT_1 &= ~_BV(BIT_HEAT_1);
	}
	else if (faceNo == 2) {
		PORT_HEAT_2 &= ~_BV(BIT_HEAT_2);
	}
	else if (faceNo == 3) {
		PORT_HEAT_3 &= ~_BV(BIT_HEAT_3);
	}
	else if (faceNo == 4) {
		PORT_HEAT_4 &= ~_BV(BIT_HEAT_4);
	}
	else if (faceNo == 5) {
		PORT_HEAT_5 &= ~_BV(BIT_HEAT_5);
	}
	else if (faceNo == 6) {
		PORT_HEAT_6 &= ~_BV(BIT_HEAT_6);
	}
}

#ifdef VERSION2
	bool ReadSensorValue (int faceNo) 
	{	
		if (faceNo == 1) {
			if (_BV(BIT_SENSOR_1) & PINA) {
				return true;
			}
		}
		else if (faceNo == 2) {
			if (_BV(BIT_SENSOR_2) & PINA) {
				return true;
			}
		}
		else if (faceNo == 3) {
			if (_BV(BIT_SENSOR_3) & PINA) {
				return true;
			}
		}
		else if (faceNo == 4) {
			if (_BV(BIT_SENSOR_4) & PINB) {
				return true;
			}	
		}
		else if (faceNo == 5) {
			if (_BV(BIT_SENSOR_5) & PINA) {
				return true;
			}	
		}
		else if (faceNo == 6) {
			if (_BV(BIT_SENSOR_6) & PINA) {
				return true;
			}		
		}
		
		return false;
	}
	
	void SwitchSensorsOn ()
	{
		PORTC |= _BV(BIT_SENSOR_ON);
	}

	void SwitchSensorsOff ()
	{
		PORTC &= ~_BV(BIT_SENSOR_ON);
	}
		
#endif