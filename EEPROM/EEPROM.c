/*
 * EEPROM.c
 *
 * Created: 7/11/2012 6:34:02 PM
 *  Author: Jonas
 */ 

#include "EEPROM.h"
#include "../CubeBrain/ByteHelpers.h"
#include <avr/eeprom.h>

// this function is taken from the Atmega1284 data sheet (p 27)
void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	// only write if the byte has changed
	if ( EEPROM_read(uiAddress) ==  ucData) return;
	
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	
	/* Set up address and Data Registers */
	EEAR = uiAddress;
	EEDR = ucData;
	
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
}

// this function is taken from the Atmega1284 data sheet (p 28)
unsigned char EEPROM_read(unsigned int uiAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	
	/* Set up address register */
	EEAR = uiAddress;
	
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	
	/* Return data from Data Register */
	return EEDR;
}

void EEPROM_setStored90degIncrement (uint8_t incr)
{
	EEPROM_write (EEPROM_90DEG_INCREMENT, incr);		
}

uint8_t EEPROM_getStored90degIncrement ()
{
	return EEPROM_read(EEPROM_90DEG_INCREMENT);
}

void EEPROM_setStoredAddress (uint16_t address) 
{
	char lsByte = lsb(address);
	char msByte = msb(address);
	
	EEPROM_write (EEPROM_ADDRESS_LSB, lsByte);
	EEPROM_write (EEPROM_ADDRESS_MSB, msByte);
}

uint16_t EEPROM_getStoredAddress ()
{
	// read the two relevant bytes from EEPROM
	char lsByte = EEPROM_read(EEPROM_ADDRESS_LSB);
	char msByte = EEPROM_read(EEPROM_ADDRESS_MSB);
	
	// put them together into one number
	uint16_t address = combineTwoBytes(msByte, lsByte);
	return address;
	
	return 1;
}

void EEPROM_setStoredOperatingMode (uint8_t mode)
{
	EEPROM_write (EEPROM_OPERATING_MODE, mode);	
}

uint8_t EEPROM_getStoredOperatingMode ()
{
	return EEPROM_read(EEPROM_OPERATING_MODE);
}

void EEPROM_setStoredDynamixelBacklash (uint8_t backlash)
{
	EEPROM_write (EEPROM_DYNAMIXEL_BACKLASH, backlash);
}

uint8_t EEPROM_getStoredDynamixelBacklash ()
{
	return EEPROM_read(EEPROM_DYNAMIXEL_BACKLASH);
}

void EEPROM_setStoredCalibPosition (uint8_t id, uint16_t pos) 
{
	if (id > 3) return;
	
	char lsByte = lsb(pos);
	char msByte = msb(pos);
	
	EEPROM_write (EEPROM_0DEG_CALIB_POS_LSB + id*EEPROM_BYTES_PER_CALIB_POS, lsByte);
	EEPROM_write (EEPROM_0DEG_CALIB_POS_MSB + id*EEPROM_BYTES_PER_CALIB_POS, msByte);
}

uint16_t EEPROM_getStoredCalibPosition (uint8_t id)
{
	if (id > 3) return 0xFFFF;
	
	// read the two relevant bytes from EEPROM
	char lsByte = EEPROM_read(EEPROM_0DEG_CALIB_POS_LSB + id*EEPROM_BYTES_PER_CALIB_POS);
	char msByte = EEPROM_read(EEPROM_0DEG_CALIB_POS_MSB + id*EEPROM_BYTES_PER_CALIB_POS);
	
	return combineTwoBytes(msByte, lsByte);
}