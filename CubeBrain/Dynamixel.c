/*
 * Dynamixel.c
 *
 * Created: 5/4/2012 6:27:37 PM
 *  Author: Jonas
 */ 


#include "Dynamixel.h"


bool DynamixelInitialize( void )
{
  // check if a Dynamixel is responding on address 1
  byte RS485Parameter[2];
  RS485Parameter[0] = P_PRESENT_VOLTAGE;
  RS485Parameter[1] = 1; //Read Length
  TxRS485_Packet(DYNAMIXEL_ADDRESS, INST_READ, 2, RS485Parameter);

  byte RS485RxBuffer[128];
  byte bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1], RS485RxBuffer);

  // if the Dynamixel did not return anything, return 0 = false.
  if (bRxPacketLength == 0) return false;

  // Do some motor setup
  DynamixelSetComplianceSlopes (DYNAMIXEL_ADDRESS, 0x80); // default = 0x20
  DynamixelSetComplianceMargins (DYNAMIXEL_ADDRESS, 0); // default = 0
  DynamixelSetPunch(DYNAMIXEL_ADDRESS, 0x40); // default value seems to be 0x20 (also the minimum)
  DynamixelSetTorqueEnable (DYNAMIXEL_ADDRESS, 1);
  DynamixelDisableContinuousRotation (DYNAMIXEL_ADDRESS);
  DynamixelSetGoalSpeedWithRetries(DYNAMIXEL_ADDRESS, DYNAMIXEL_MAX_SPEED); // caps the speed during position control mode

  // set the current position, whatever it is to be the goal position,
  // this prevents the motor from starting to make a move after an unproper shutdown
  DynamixelGetMomentaryPosition(0); // sets volatile variable momentaryPos
  DynamixelSetGoalPosition(DYNAMIXEL_ADDRESS, momentaryPos);

  // setup steps that are not involved in case the motor is moving on startup
  DynamixelSetLED(DYNAMIXEL_ADDRESS, 0x00);
  DynamixelSetReturnDelayTime(DYNAMIXEL_ADDRESS, 0x60);
  DynamixelSetHighestLimitTemperature(DYNAMIXEL_ADDRESS, 0x96); // 0x96 is the max perimissible as per documentation
  DynamixelSetLowestLimitVoltage(DYNAMIXEL_ADDRESS, 0x3C); // 0x3C is the default value according to docs
  DynamixelSetHighestLimitVoltage(DYNAMIXEL_ADDRESS, 0xBE); // 0xBE is the default value according to docs
  DynamixelSetMaxTorque(DYNAMIXEL_ADDRESS, 0x140); // 0x3FF is the default according to docs, lowball this to avoid robot destruction
  DynamixelSetTorqueLimit(DYNAMIXEL_ADDRESS, 0x140); // same as above, but for the value stored in RAM (above is EEPROM)
  DynamixelSetAlarmLED(DYNAMIXEL_ADDRESS, 0x7F); // 0x7F means: blink for any error (
  
  // it worked, return 1 = true
  return true;
}


uint8_t DynamixelGetOneByteValue(byte servoId, byte controlTablePosition)
{
	byte RS485Parameter[2];
	RS485Parameter[0] = controlTablePosition;
	RS485Parameter[1] = 1; //Read Length
	TxRS485_Packet(servoId, INST_READ, 2, RS485Parameter);
	
	byte RS485RxBuffer[128];
	byte bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1], RS485RxBuffer);
	 
	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		return RS485RxBuffer[5];
	}
	
	_delay_ms(50);
	
	return 0xff;
}


uint16_t DynamixelGetTwoByteValue (byte servoId, byte controlTablePosition)
{
	byte RS485Parameter[2];
	RS485Parameter[0] = controlTablePosition;
	RS485Parameter[1] = 2; //Read Length
	TxRS485_Packet( servoId, INST_READ, 2, RS485Parameter );
	
	byte RS485RxBuffer[128];
	byte bRxPacketLength = RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1], RS485RxBuffer );
	
	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		return (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
	}
	
	_delay_ms(50);
	
	return 0xffff;
}


bool DynamixelSetOneByteValue(byte servoID, byte controlTablePosition, uint8_t value)
{
	byte RS485Parameter[2];
	RS485Parameter[0] = controlTablePosition;
	RS485Parameter[1] = value;
	TxRS485_Packet(servoID,INST_WRITE,2, RS485Parameter);

	// wait for the reply
	byte rLength, RS485RxBuffer[128];
	rLength = RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE, RS485RxBuffer );

	_delay_ms(50);

	return (rLength != 0);
}


bool DynamixelSetTwoByteValue(byte servoID, byte controlTablePosition, uint16_t value)
{
	byte th, tl;

	th = value >> 8;
	tl = value & 0xff;

	byte RS485Parameter[3];
	RS485Parameter[0] = controlTablePosition;
	RS485Parameter[1] = tl;
	RS485Parameter[2] = th;
	TxRS485_Packet(servoID,INST_WRITE,3, RS485Parameter);
	
	// wait for the reply
	byte rLength, RS485RxBuffer[128];
	rLength = RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE, RS485RxBuffer );

	_delay_ms(50);
	
	return (rLength != 0);
}

void DynamixelRetryErrorHandler () {
  TxRS232_String("[Cube 0x");
  TxRS232_Hex2Byte(myAddress);
  TxRS232_String("] Dynamixel Retry Error.\r\n");
}


/************************************************************************/
/* Getter Functions                                                     */
/************************************************************************/


uint16_t DynamixelGetModelNumber (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_MODEL_NUMBER_L);
}


uint8_t DynamixelGetFirmwareVersion (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_VERSION);
}


uint8_t DynamixelGetMotorID (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_ID);
}


uint8_t DynamixelGetBaudRate (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_BAUD_RATE);
}


uint8_t DynamixelGetReturnDelayTime (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_RETURN_DELAY_TIME);
}


uint16_t DynamixelGetCWAngleLimit (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_CW_ANGLE_LIMIT_L);
}


uint16_t DynamixelGetCCWAngleLimit (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_CCW_ANGLE_LIMIT_L);
}


uint8_t DynamixelGetHighestLimitTemperature (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_LIMIT_TEMPERATURE);
}


uint8_t DynamixelGetLowestLimitVoltage (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_DOWN_LIMIT_VOLTAGE);
}


uint8_t DynamixelGetHighestLimitVoltage (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_UP_LIMIT_VOLTAGE);
}


uint16_t DynamixelGetMaxTorque (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_MAX_TORQUE_L);
}


uint8_t DynamixelGetStatusReturnLevel (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_RETURN_LEVEL);
}


uint8_t DynamixelGetAlarmLED (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_ALARM_LED);
}


uint8_t DynamixelGetAlarmShutdown (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_ALARM_SHUTDOWN);
}


uint8_t DynamixelGetTorqueEnable (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_TORQUE_ENABLE);
}


uint8_t DynamixelGetLED (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_LED);
}


uint8_t DynamixelGetCWComplianceMargin (byte servoID)
{
  return DynamixelGetOneByteValue (servoID, P_CW_COMPL_MARGIN);
}


uint8_t DynamixelGetCCWComplianceMargin (byte servoID)
{
  return DynamixelGetOneByteValue (servoID, P_CCW_COMPL_MARGIN);
}


uint8_t DynamixelGetCWComplianceSlope (byte servoID)
{
  return DynamixelGetOneByteValue (servoID, P_CW_COMPL_SLOPE);
}


uint8_t DynamixelGetCCWComplianceSlope (byte servoID)
{
  return DynamixelGetOneByteValue (servoID, P_CCW_COMPL_SLOPE);
}


uint16_t DynamixelGetGoalPosition (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_GOAL_POSITION_L);
}


uint16_t DynamixelGetGoalSpeed (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_GOAL_SPEED_L);
}


uint16_t DynamixelGetTorqueLimit (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_TORQUE_LIMIT_L);
}


uint16_t DynamixelGetMotorPosition (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_PRESENT_POSITION_L);
}


uint16_t DynamixelGetSpeed (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_PRESENT_SPEED_L);
}


uint16_t DynamixelGetLoad (byte servoID) // this is the current torque
{
	return DynamixelGetTwoByteValue (servoID, P_PRESENT_LOAD_L);
}


uint8_t DynamixelGetVoltage (byte servoID)
{
	return DynamixelGetOneByteValue(servoID, P_PRESENT_VOLTAGE);
}


uint8_t DynamixelGetTemperature (byte servoID)
{
	return DynamixelGetOneByteValue(servoID, P_PRESENT_TEMPERATURE);
}


uint8_t DynamixelGetMoving (byte servoID)
{
	return DynamixelGetOneByteValue(servoID, P_MOVING);
}


uint16_t DynamixelGetPunch (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_PUNCH_L);
}


/************************************************************************/
/* Setter Functions                                                     */
/************************************************************************/

bool DynamixelSetMotorID (byte servoID, uint8_t theID)
{
	if (theID > 0xfd) return false;
	return DynamixelSetOneByteValue(servoID, P_ID, theID);
}


/*
Baud Rate. Determines the communication speed. The computation is done by the following formula.
Speed (BPS) = 2000000 / (Address4 + 1)
Note A maximum Baud Rate error of 3% is within the tolerance of UART communication.
Caution The initial value of Baudrate is set to 1(1000000bps)
*/
bool DynamixelSetBaudRate (byte servoID, uint8_t baudrate)
{
	if (baudrate > 0xfe) return false;
	return DynamixelSetOneByteValue(servoID, P_BAUD_RATE, baudrate);
}


/*
Return Delay Time. The time it takes for the Status Packet to return after the Instruction Packet is sent. 
The delay time is given by 2uSec * Address5 value.
*/
bool DynamixelSetReturnDelayTime (byte servoID, uint8_t rdt)
{
	if (rdt > 0xfe) return false;
	return DynamixelSetOneByteValue(servoID, P_RETURN_DELAY_TIME, rdt);
}


bool DynamixelSetCWAngleLimit (byte servoID, uint16_t cwAngleLimit)
{
	if (cwAngleLimit > 0x3ff) return false;
	return DynamixelSetTwoByteValue(servoID, P_CW_ANGLE_LIMIT_L, cwAngleLimit);
}


bool DynamixelSetCCWAngleLimit (byte servoID, uint16_t ccwAngleLimit)
{
	if (ccwAngleLimit > 0x3ff) return false;
	return DynamixelSetTwoByteValue(servoID, P_CCW_ANGLE_LIMIT_L, ccwAngleLimit);
}


bool DynamixelSetHighestLimitTemperature (byte servoID, uint8_t hlt)
{
	if (hlt > 0x96) return false;
	return DynamixelSetOneByteValue(servoID, P_LIMIT_TEMPERATURE, hlt);
}


bool DynamixelSetLowestLimitVoltage (byte servoID, uint8_t llv)
{
	if (llv < 0x32 || llv > 0xfa) return false;
	return DynamixelSetOneByteValue(servoID, P_DOWN_LIMIT_VOLTAGE, llv);
}


bool DynamixelSetHighestLimitVoltage (byte servoID, uint8_t hlv)
{
	if (hlv < 0x32 || hlv > 0xfa) return false;
	return DynamixelSetOneByteValue(servoID, P_UP_LIMIT_VOLTAGE, hlv);
}


bool DynamixelSetMaxTorque (byte servoID, uint16_t maxTorque)
{
	if (maxTorque > 0x3ff) return false;
	return DynamixelSetTwoByteValue(servoID, P_MAX_TORQUE_L, maxTorque);
}


bool DynamixelSetStatusReturnLevel (byte servoID, uint8_t srl)
{
	if (srl > 2) return false;
	return DynamixelSetOneByteValue(servoID, P_RETURN_LEVEL, srl);
}


bool DynamixelSetAlarmLED (byte servoID, uint8_t alarmLed)
{
	if (alarmLed > 0x7f) return false;
	return DynamixelSetOneByteValue(servoID, P_ALARM_LED, alarmLed);
}


bool DynamixelSetAlarmShutdown (byte servoID, uint8_t alarmShutdown)
{
	if (alarmShutdown > 0x7f) return false;
	return DynamixelSetOneByteValue(servoID, P_ALARM_SHUTDOWN, alarmShutdown);
}


/*
Torque Enable. When the power is first turned on, the Dynamixel actuator enters the Torque Free 
Run condition (zero torque). Setting the value in Address 0x18 to 1 enables the torque.
*/
bool DynamixelSetTorqueEnable(byte servoID,byte onoff01)
{
	if (onoff01 != 0 && onoff01 != 1) return false;
	return DynamixelSetOneByteValue(servoID, P_TORQUE_ENABLE, onoff01);
}


bool DynamixelSetLED(byte servoID,byte onOff01)
{
	if (onOff01 != 0 && onOff01 != 1) return false;
	return DynamixelSetOneByteValue(servoID, P_LED, onOff01);
}


bool DynamixelSetCWComplianceMargin (byte servoID, uint8_t cwComplianceMargin)
{
  if (cwComplianceMargin > 0xfe) return false;
  return DynamixelSetOneByteValue(servoID, P_CW_COMPL_MARGIN, cwComplianceMargin);
}


bool DynamixelSetCCWComplianceMargin (byte servoID, uint8_t ccwComplianceMargin)
{
  if (ccwComplianceMargin > 0xfe) return false;
  return DynamixelSetOneByteValue(servoID, P_CCW_COMPL_MARGIN, ccwComplianceMargin);
}


bool DynamixelSetCWComplianceSlope (byte servoID, uint8_t cwComplianceSlope)
{
  if (cwComplianceSlope > 0xfe) return false;
  return DynamixelSetOneByteValue(servoID, P_CW_COMPL_SLOPE, cwComplianceSlope);
}


bool DynamixelSetCCWComplianceSlope (byte servoID, uint8_t ccwComplianceSlope)
{
  if (ccwComplianceSlope > 0xfe) return false;
  return DynamixelSetOneByteValue(servoID, P_CCW_COMPL_SLOPE, ccwComplianceSlope);
}


/*
Goal Position Requested angular position for the Dynamixel actuator output to move to. Setting 
this value to 0x3ff moves the output shaft to the position at 300°.
*/
bool DynamixelSetGoalPosition (byte servoID, uint16_t goalPosition)
{
	if (goalPosition > 0x3ff) return false;
	return DynamixelSetTwoByteValue(servoID, P_GOAL_POSITION_L, goalPosition);
}

bool DynamixelSetGoalPositionWithRetries (byte servoID, uint16_t goalPosition)
{
  uint8_t errCnt = 0;
  do if (DynamixelSetGoalPosition(servoID, goalPosition)) return true;
  while ( errCnt++ < DYNAMIXEL_MAX_RETRIES);
  DynamixelRetryErrorHandler();
  return false;
}

/*
Moving Speed. Sets the angular velocity of the output moving to the Goal Position. Setting this 
value to its maximum value of 0x3ff moves the output with an angular velocity of 114 RPM, 
provided that there is enough power supplied (The lowest velocity is when this value is set to 1. 
When set to 0, the velocity is the largest possible for the supplied voltage, e.g. no velocity 
control is applied.)
*/
bool DynamixelSetGoalSpeed (byte servoID, uint16_t movingSpeed)
{
	if (movingSpeed > 0x7ff) return false; // Note: the 10th bit can be used in endless turn mode to control the direction
	return DynamixelSetTwoByteValue(servoID, P_GOAL_SPEED_L, movingSpeed);
}

bool DynamixelSetGoalSpeedWithRetries (byte servoID, uint16_t movingSpeed)
{
  uint8_t errCnt = 0;
  do if (DynamixelSetGoalSpeed(servoID, movingSpeed)) return true;
  while ( errCnt++ < DYNAMIXEL_MAX_RETRIES);
  DynamixelRetryErrorHandler();
  return false;
}


bool DynamixelSetTorqueLimit (byte servoID,uint16_t torqueLimit)
{
	if (torqueLimit > 0x3ff) return false;
	return DynamixelSetTwoByteValue(servoID, P_TORQUE_LIMIT_L, torqueLimit);
}


bool DynamixelSetPunch (byte servoID, uint16_t punch)
{
	if (punch > 0x3ff) return false;
	return DynamixelSetTwoByteValue(servoID, P_PUNCH_L, punch);
}



/************************************************************************/
/* Helper Functions                                                     */
/************************************************************************/

bool DynamixelSetComplianceMargins (byte servoID, uint8_t complianceMargin)
{
	return DynamixelSetCWComplianceMargin(servoID, complianceMargin) &&
		DynamixelSetCCWComplianceMargin(servoID, complianceMargin);
}


bool DynamixelSetComplianceSlopes (byte servoID, uint8_t complianceSlope)
{
	return DynamixelSetCWComplianceSlope(servoID, complianceSlope) &&
		DynamixelSetCCWComplianceSlope(servoID, complianceSlope);
}


bool DynamixelMoveTo(byte servoID, uint16_t pos, uint16_t speed)
{

	byte ph,pl,sh,sl;

	ph = pos >> 8;
	pl = pos & 0xff;
	sh = speed >> 8;
	sl = speed & 0xff;
	
	byte RS485Parameter[5];
	RS485Parameter[0] = P_GOAL_POSITION_L;
	RS485Parameter[1] = pl; //Writing Data P_GOAL_POSITION_L
	RS485Parameter[2] = ph; //Writing Data P_GOAL_POSITION_H
	RS485Parameter[3] = sl; //Writing Data P_GOAL_SPEED_L
	RS485Parameter[4] = sh; //Writing Data P_GOAL_SPEED_H
	TxRS485_Packet(servoID,INST_WRITE,5, RS485Parameter);
	
	// wait for the reply
	byte rLength, RS485RxBuffer[128];
	rLength = RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE, RS485RxBuffer );
	
	return (rLength != 0);
}


/*
If both values for the CW Angle Limit and the CCW Angle Limit are set to 0, an Endless Turn mode 
can be implemented by setting the Goal Speed. This feature can be used for implementing a continuously 
rotating wheel.
*/
bool DynamixelEnableContinuousRotation (byte servoID)
{
	return DynamixelSetCWAngleLimit(servoID, 0x00) &&
		DynamixelSetCCWAngleLimit(servoID, 0x00);
}

bool DynamixelEnableContinuousRotationWithRetries (byte servoID)
{
  uint8_t errCnt = 0;
  do if (DynamixelEnableContinuousRotation(servoID)) return true;
  while ( errCnt++ < DYNAMIXEL_MAX_RETRIES);
  DynamixelRetryErrorHandler();
  return false;
}

bool DynamixelDisableContinuousRotation (byte servoID)
{
	return DynamixelSetCWAngleLimit(servoID, 0x00) &&
		DynamixelSetCCWAngleLimit(servoID, 0x3ff);
}

bool DynamixelDisableContinuousRotationWithRetries (byte servoID)
{
  uint8_t errCnt = 0;
  do if (DynamixelDisableContinuousRotation(servoID)) return true;
  while ( errCnt++ < DYNAMIXEL_MAX_RETRIES);
  DynamixelRetryErrorHandler();
  return false;
}

uint8_t DynamixelFindAddress (void)
{
	//TxRS232_String("Searching motor.\r\n");
	
	uint8_t i = 1;
	do
	{
		_delay_ms(50);
		
		uint8_t vol = DynamixelGetFirmwareVersion(i);

		TxRS232_String("Trying Motor ID 0x: "); TxRS232_Hex(i);
		//TxRS232_String(" - Result: 0x"); TxRS232_Hex(vol); TxRS232_String(" / dec: "); TxRS232_Dec(vol);
		TxRS232_String("\r\n");

		if (vol != 0xff) {
			//TxRS232_String("Returning: 0x"); TxRS232_Hex(i);
			//TxRS232_String("\r\n");
			return i;
		}
		
		i++;
	} while (i <= 0xfd);
	
	return 0xff;
}

// as opposed to 
uint16_t DynamixelGetMomentaryPosition(uint32_t maxAge) 
{
  if (momentaryPosLastUpdated < time - maxAge) 
  {
    momentaryPos = 0xffff;
    uint8_t errCnt = 0;
    while (momentaryPos == 0xffff && errCnt++ < DYNAMIXEL_MAX_RETRIES)
    {
      momentaryPos = DynamixelGetMotorPosition(DYNAMIXEL_ADDRESS);
    }
  }
  
  return momentaryPos;
}