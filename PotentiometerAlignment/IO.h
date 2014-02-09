/*
 * IOFunctions.h
 *
 * Created: 5/2/2012 2:16:02 PM
 *  Author: Jonas
 */ 


#ifndef IO_H_
#define IO_H_


// Hardware related functions
#define BIT_HEAT_1	PC2
#define BIT_HEAT_2	PC3
#define BIT_HEAT_3	PC4
#define BIT_HEAT_4	PC5
#define BIT_HEAT_5	PC6
#define BIT_HEAT_6	PC7

#define BIT_LED_ORANGE PB3
#define BIT_LED_GREEN PB4

#define BIT_RS485_DIRECTION0  PD4
#define BIT_RS485_DIRECTION1  PD5
#define BIT_DATA_DIRECTION0  PD6
#define BIT_DATA_DIRECTION1  PD7


// Functions

void InitializeIOPins(void);
void SetStatusLEDs (int orange, int green);
void LEDMorse(byte cntShort, byte cntLong);
void SwitchHeatOn (int faceNo);
void SwitchHeatOff (int faceNo);


#endif /* IO_H_ */