/*
 * ByteHelpers.h
 *
 * Created: 4/3/2013 4:00:36 PM
 *  Author: Jonas
 */ 


#ifndef BYTEHELPERS_H_
#define BYTEHELPERS_H_

// most significant and least significant byte helper functions
inline char msb (uint16_t twoByteVal)
{
	return (twoByteVal >> 8);
}

inline char lsb (uint16_t twoByteVal)
{
	return twoByteVal & 0xff;
}

inline uint16_t combineTwoBytes (char msByte, char lsByte)
{
	return (msByte << 8) + lsByte;
}

// the following comes from Arduino's Wire library twi.h
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#endif /* BYTEHELPERS_H_ */