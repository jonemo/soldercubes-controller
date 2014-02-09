/*
 * Time.c
 *
 * Created: 5/5/2012 4:41:06 PM
 *  Author: Jonas
 */ 

#include "Time.h"

ISR(TIMER3_COMPA_vect)
{
  // This interrupt gets triggered every 16k cycles, i.e. every millisecond
	time += 1;
  
  // the (RGB) LED cubes use this same interrupt to do pulse width modulation on the LED outputs
  //if (myOperatingMode == MODE_LED_CUBE && time % 5 == 0)
  //{
    //SwitchRed(time % 0xFF < LEDCubeRed);
    //SwitchGreen(time % 0xFF < LEDCubeGreen);
    //SwitchBlue(time % 0xFF < LEDCubeBlue);
  //}
}

void InitializeTimer() 
{
  // Setup Timer 3 for CTC mode (clear counter on match)
	TCCR3A &= (0 << COM3A1) & (0 << COM3A0) & (0 << COM3B1) & (0 << COM3B0) & (0 << 3     ) & (0 << 2     ) & (0 << WGM31 ) && (0 << WGM30 ); 
	TCCR3B &= (0 << ICNC3 ) & (0 << ICES3 ) & (0 << 5     ) & (0 << WGM33 ) &                 (0 << CS32  ) & (0 << CS31  )                 ;
	TCCR3B |=                                                                 (1 << WGM32 ) |                                  ( 1 << CS30 );
	OCR3A = 16000; // Reset counter every 16000 cycles, i.e. every millisecond
	
  // Enable CTC interrupt
	TIMSK3 |= (1 << OCIE3A); 
	
	// initialize volatile (aka global) time variable, get incremented by interrupt
	time = 0;
}