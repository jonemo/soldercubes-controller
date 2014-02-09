/*
 * LED.h
 *
 * Created: 8/11/2013 7:41:56 PM
 *  Author: Jonas
 */ 


#ifndef LED_H_
#define LED_H_

  #define BIT_LED_CUBE_RED 	   PD4
  #define PORT_LED_CUBE_RED    PORTB
  #define PWM_LED_CUBE_RED     OCR0B
  #define TIMER_LED_CUBE_RED   TCCR0A
  
  #define BIT_LED_CUBE_GREEN 	 PD4
  #define PORT_LED_CUBE_GREEN  PORTD
  #define PWM_LED_CUBE_GREEN   OCR1B
  #define TIMER_LED_CUBE_GREEN TCCR1A
  
  #define BIT_LED_CUBE_BLUE 	 PD5
  #define PORT_LED_CUBE_BLUE   PORTD
  #define PWM_LED_CUBE_BLUE    OCR1A
  #define TIMER_LED_CUBE_BLUE  TCCR1A

  void InitializeIOPinsForLEDModule (uint8_t dcRed, uint8_t dcGreen, uint8_t dcBlue);
  void SetLED (uint8_t dcRed, uint8_t dcGreen, uint8_t dcBlue);

#endif /* LED_H_ */