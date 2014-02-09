/*
 * IOFunctions.h
 *
 * Created: 5/2/2012 2:16:02 PM
 *  Author: Jonas
 */ 


#ifndef IO_H_
#define IO_H_


	// Hardware related functions
	#ifdef VERSION1
	
		#define BIT_HEAT_1		PC2
		#define PORT_HEAT_1		PORTC
		
		#define BIT_HEAT_2		PC3
		#define PORT_HEAT_2		PORTC
		
		#define BIT_HEAT_3		PC4
		#define PORT_HEAT_3		PORTC
		
		#define BIT_HEAT_4		PC5
		#define PORT_HEAT_4		PORTC
		
		#define BIT_HEAT_5		PC6
		#define PORT_HEAT_5		PORTC
		
		#define BIT_HEAT_6		PC7
		#define PORT_HEAT_6		PORTC
		
	#endif

	#ifdef VERSION2

		#define BIT_HEAT_1		PC2
		#define PORT_HEAT_1		PORTC
		
		#define BIT_HEAT_2		PC3
		#define PORT_HEAT_2		PORTC
		
		#define BIT_HEAT_3		PA5
		#define PORT_HEAT_3		PORTA
		
		#define BIT_HEAT_4		PB2
		#define PORT_HEAT_4		PORTB
		
		#define BIT_HEAT_5		PC6
		#define PORT_HEAT_5		PORTC
		
		#define BIT_HEAT_6		PC7
		#define PORT_HEAT_6		PORTC
	
		#define BIT_SENSOR_ON	PC4
	
		#define BIT_SENSOR_1	PA1
		#define PORT_SENSOR_1	PORTA
		#define PIN_SENSOR_1	PINA
		
		#define BIT_SENSOR_2	PA3
		#define PORT_SENSOR_2	PORTA
		#define PIN_SENSOR_2	PINA
		
		#define BIT_SENSOR_3	PA4
		#define PORT_SENSOR_3	PORTA
		#define PIN_SENSOR_3	PINA
		
		#define BIT_SENSOR_4	PB1
		#define PORT_SENSOR_4	PORTB
		#define PIN_SENSOR_4	PINB
		
		#define BIT_SENSOR_5	PA6
		#define PORT_SENSOR_5	PORTA
		#define PIN_SENSOR_5	PINA
		
		#define BIT_SENSOR_6	PA7
		#define PORT_SENSOR_6	PORTA
		#define PIN_SENSOR_6	PINA

	#endif

	#define BIT_LED_ORANGE PB3
	#define BIT_LED_GREEN PB4

	// Functions

	void InitializeIOPins(void);
	void SetStatusLEDs (int orange, int green);
	void LEDMorse(byte cntShort, byte cntLong);
	void SwitchHeatOn (int faceNo);
	void SwitchHeatOff (int faceNo);

	#ifdef VERSION2
		bool ReadSensorValue (int faceNo);
		void SwitchSensorsOn ();
		void SwitchSensorsOff ();
	#endif

#endif /* IO_H_ */