/*
 * EEPROM.h
 *
 * Created: 7/11/2012 6:33:24 PM
 *  Author: Jonas
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

#include <avr/eeprom.h>

#define EEPROM_INITIAL_POSITION_MSB	  0
#define EEPROM_INITIAL_POSITION_LSB	  1
#define EEPROM_ADDRESS_MSB			      2
#define EEPROM_ADDRESS_LSB			      3
#define EEPROM_OPERATING_MODE		      4
#define EEPROM_90DEG_INCREMENT		    5
#define EEPROM_DYNAMIXEL_BACKLASH	    6

#define EEPROM_0DEG_CALIB_POS_MSB	    10
#define EEPROM_0DEG_CALIB_POS_LSB	    11
#define EEPROM_BYTES_PER_CALIB_POS	  2
// bytes between 12-17 (incl) are used!
// Note: EEPROM_3DEG_CALIB_POS_LSB	17

void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);

// Note:  The EEPROM data bytes are addressed linearly between 0 and [...] 4096. 

// stores the integer representing which rotation increment we are currently
// at in the actuated modules. values are [1..4]
void EEPROM_setStored90degIncrement (uint8_t mode) ;
uint8_t EEPROM_getStored90degIncrement ();

// the address that the module thinks its known as
// it will only reply to messages sent to this address
void EEPROM_setStoredAddress (uint16_t address) ;
uint16_t EEPROM_getStoredAddress ();

void EEPROM_setStoredOperatingMode (uint8_t mode) ;
uint8_t EEPROM_getStoredOperatingMode ();

void EEPROM_setStoredDynamixelBacklash (uint8_t mode) ;
uint8_t EEPROM_getStoredDynamixelBacklash ();

void EEPROM_setStoredCalibPosition (uint8_t id, uint16_t pos) ;
uint16_t EEPROM_getStoredCalibPosition (uint8_t pos);

#endif /* EEPROM_H_ */