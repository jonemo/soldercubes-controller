/*
 * LED.c
 *
 * Created: 8/11/2013 7:41:22 PM
 *  Author: Jonas
 */ 


#include "LED.h"

void InitializeIOPinsForLEDModule (uint8_t dcRed, uint8_t dcGreen, uint8_t dcBlue)
{
  // set LED RGB pins to output
  DDRB |= _BV(BIT_LED_CUBE_RED);
  DDRD |= _BV(BIT_LED_CUBE_GREEN);
  DDRD |= _BV(BIT_LED_CUBE_BLUE);
  
  // switch the ports to off state
  PORT_LED_CUBE_RED &= ~_BV(BIT_LED_CUBE_RED);
  PORT_LED_CUBE_GREEN &= ~_BV(BIT_LED_CUBE_GREEN);
  PORT_LED_CUBE_BLUE &= ~_BV(BIT_LED_CUBE_BLUE);
 
  // set up timer 1 in fast pwm mode
  // we are pre-scaling by 8 because the waveform has a transient that is larger than the shortest possible pulse
  TCCR1A &=                 (0 << COM1A0) &                 (0 << COM1B0) & (0 << 3     ) & (0 << 2     ) & (0 << WGM11 )                ;
  TCCR1A |= (1 << COM1A1) |                 (1 << COM1B1) |                                                                 (1 << WGM10 );
  TCCR1B &= (0 << ICNC1 ) & (0 << ICES1 ) & (0 << 5     ) & (0 << WGM13 )                 & (0 << CS12  ) &                 (0 << CS10  );
  TCCR1B |=                                                                 (1 << WGM12 ) |                 ( 1 << CS11 )                ;
  
  PWM_LED_CUBE_BLUE  = 0;
  PWM_LED_CUBE_GREEN = 0;
  
  // set up timer 0 in fast pwm mode
  TCCR0A &= (0 << COM0A1) & (0 << COM0A0) &                 (0 << COM0B0) & (0 << 3     ) & (0 << 2     )                                ;
  TCCR0A |=                                 (1 << COM0B1) |                                                 (1 << WGM01 ) | (1 << WGM00 );
  TCCR0B &= (0 << FOC0A ) & (0 << FOC0B ) & (0 << 5     ) & (0 << 4     ) & (1 << WGM02 ) & (0 << CS02  ) &                 (0 << CS00  );
  TCCR0B |=                                                                                                 (1 << CS01  )                ;
   	
  // we are only using one of the pwm outputs (which is the pin one that is otherwise driving the small green led).
  // lets's switch the small red led to off here
  OCR0A = 0;
  PWM_LED_CUBE_RED = 0;
  
  TCNT0 = 0;
  TCNT1 = 0;
       
  // initialize volatile (aka global) time variable, get incremented by interrupt
  time = 0;
}

void SetLED (uint8_t dcRed, uint8_t dcGreen, uint8_t dcBlue) 
{
  // because of the transient we have, even at 0 we get a base brightness
  // to make sure we go totally dark at 0, just disable the output pin entirely
  
  if (dcRed == 0) {
    DDRB &= ~_BV(BIT_LED_CUBE_RED);
  }
  else {
    DDRB |= _BV(BIT_LED_CUBE_RED);
  }
  
  if (dcGreen == 0) {
    DDRD &= ~_BV(BIT_LED_CUBE_GREEN);
  }
  else {
    DDRD |= _BV(BIT_LED_CUBE_GREEN);
  }
  
  if (dcBlue == 0) {
    DDRD &= ~_BV(BIT_LED_CUBE_BLUE);
  }
  else {
    DDRD |= _BV(BIT_LED_CUBE_BLUE);
  }
  
  // divide by 5 because we don't have enough current to burn for anywhere near full brightness
  PWM_LED_CUBE_BLUE  = dcBlue / 10;
  PWM_LED_CUBE_GREEN = dcGreen / 10;
  PWM_LED_CUBE_RED   = dcRed / 10;
}  
