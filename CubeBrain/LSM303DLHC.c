/*
 * LSM303DLHC.c
 *
 * Created: 7/31/2012 4:41:59 PM
 *  Author: Jonas
 */ 

#include "../Communication/twi.h"
#include "LSM303DLHC.h"

void initializeLSM303DLHC ()
{
	// set accelerometer update rate to 1 Hz
	byte input[2];
	input[0] = CTRL_REG1_A;
	input[1] = 0x27; // set update frequency to 10 Hz, normal power mode, x/y/z enabled
	twi_writeTo(TWI_ACCELEROMETER_SLAVE_ADDR, input, 2, 1);
	
	input[0] = CRA_REG_M;
	input[1] = 0x88; // enable temperature sensor, 3.0Hz data rate
	twi_writeTo(TWI_MAGNETOMETER_SLAVE_ADDR, input, 2, 1);
	
	input[0] = CRB_REG_M;
	input[1] = 0x00; // "continuous conversion mode"
	twi_writeTo(TWI_MAGNETOMETER_SLAVE_ADDR, input, 2, 1);
}

int16_t getTwoByteValueFromLSM303DLHC (byte deviceAddr, byte msbAddr, byte lsbAddr)
{
	uint8_t result;
	byte error = 0;
	uint8_t input[1];
	uint8_t lsByte[1];
	uint8_t msByte[1];
	
	input[0] = lsbAddr;
	result = twi_writeTo(deviceAddr, input, 1, 1);
	if (result != 0x00) error++;
	
	result = twi_readFrom(deviceAddr, lsByte, 1); // read the six acceleration bytes
	if (result != 0x01) error++;
	
	input[0] = msbAddr;
	result = twi_writeTo(deviceAddr, input, 1, 1);
	if (result != 0x00) error++;
	
	twi_readFrom(deviceAddr, msByte, 1);
	if (result != 0x01) error++;
	
	int16_t acc = (msByte[0] << 8) + lsByte[0];
	
	return acc;
}

/**

*/
int16_t getXAcceleration()
{
	return getTwoByteValueFromLSM303DLHC(TWI_ACCELEROMETER_SLAVE_ADDR, OUT_X_H_A, OUT_X_L_A);
}

/**

*/
int16_t getYAcceleration()
{
	return getTwoByteValueFromLSM303DLHC(TWI_ACCELEROMETER_SLAVE_ADDR, OUT_Y_H_A, OUT_Y_L_A);
}

/**

*/
int16_t getZAcceleration()
{
	return getTwoByteValueFromLSM303DLHC(TWI_ACCELEROMETER_SLAVE_ADDR, OUT_Z_H_A, OUT_Z_L_A);
}

/**

*/
uint16_t getXMagField()
{
	return getTwoByteValueFromLSM303DLHC(TWI_MAGNETOMETER_SLAVE_ADDR, OUT_X_H_M, OUT_X_L_M);
}

/**

*/
uint16_t getYMagField()
{
	return getTwoByteValueFromLSM303DLHC(TWI_MAGNETOMETER_SLAVE_ADDR, OUT_Y_H_M, OUT_Y_L_M);
}

/**

*/
uint16_t getZMagField()
{
	return getTwoByteValueFromLSM303DLHC(TWI_MAGNETOMETER_SLAVE_ADDR, OUT_Z_H_M, OUT_Z_L_M);
}

/**

*/
void getAccelerometerDebugOutput ()
{
	uint8_t response[16];
	uint8_t input[1];
	
	// accelerometer
	TxRS232_String("\r\nAccelerometer: ");
		
	input[0] = OUT_X_L_A;
	twi_writeTo(TWI_ACCELEROMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_ACCELEROMETER_SLAVE_ADDR, response, 1); // read the six acceleration bytes
	TxRS232_Hex(response[0]);TxRS232_String(" ");
	
	input[0] = OUT_X_H_A;
	twi_writeTo(TWI_ACCELEROMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_ACCELEROMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
	
	input[0] = OUT_Y_L_A;
	twi_writeTo(TWI_ACCELEROMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_ACCELEROMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
	
	input[0] = OUT_Y_H_A;
	twi_writeTo(TWI_ACCELEROMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_ACCELEROMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
	
	input[0] = OUT_Z_L_A;
	twi_writeTo(TWI_ACCELEROMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_ACCELEROMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
	
	input[0] = OUT_Z_H_A;
	twi_writeTo(TWI_ACCELEROMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_ACCELEROMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
}

/**

*/
void getMagnetometerDebugOutput ()
{
	uint8_t response[16];
	uint8_t input[1];
	
	// magnetometer
	TxRS232_String("\r\nMagnetometer: ");
		
	input[0] = OUT_X_H_M;
	twi_writeTo(TWI_MAGNETOMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_MAGNETOMETER_SLAVE_ADDR, response, 1); // read the six acceleration bytes
	TxRS232_Hex(response[0]);TxRS232_String(" ");
		
	input[0] = OUT_X_L_M;
	twi_writeTo(TWI_MAGNETOMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_MAGNETOMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
		
	input[0] = OUT_Y_H_M;
	twi_writeTo(TWI_MAGNETOMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_MAGNETOMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
		
	input[0] = OUT_Y_L_M;
	twi_writeTo(TWI_MAGNETOMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_MAGNETOMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
		
	input[0] = OUT_Z_H_M;
	twi_writeTo(TWI_MAGNETOMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_MAGNETOMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
		
	input[0] = OUT_Z_L_M;
	twi_writeTo(TWI_MAGNETOMETER_SLAVE_ADDR, input, 1, 1);
	twi_readFrom(TWI_MAGNETOMETER_SLAVE_ADDR, response, 1);
	TxRS232_Hex(response[0]);TxRS232_String(" ");
}