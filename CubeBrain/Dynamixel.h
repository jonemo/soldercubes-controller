/*
 * mov_functions.h
 *
 * Created: 4/2/2012 3:46:09 PM
 *  Author: Jonas
 */


#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

  // Application Specific: In the Soldercubes module the Dynamixel always has address 1
  #define DYNAMIXEL_ADDRESS  1

  //EEPROM AREA
  #define P_MODEL_NUMBER_L      0
  #define P_MODOEL_NUMBER_H     1
  #define P_VERSION             2
  #define P_ID                  3
  #define P_BAUD_RATE           4
  #define P_RETURN_DELAY_TIME   5
  #define P_CW_ANGLE_LIMIT_L    6
  #define P_CW_ANGLE_LIMIT_H    7
  #define P_CCW_ANGLE_LIMIT_L   8
  #define P_CCW_ANGLE_LIMIT_H   9
  #define P_SYSTEM_DATA2        10
  #define P_LIMIT_TEMPERATURE   11
  #define P_DOWN_LIMIT_VOLTAGE  12
  #define P_UP_LIMIT_VOLTAGE    13
  #define P_MAX_TORQUE_L        14
  #define P_MAX_TORQUE_H        15
  #define P_RETURN_LEVEL        16
  #define P_ALARM_LED           17
  #define P_ALARM_SHUTDOWN      18
  #define P_OPERATING_MODE      19
  #define P_DOWN_CALIBRATION_L  20
  #define P_DOWN_CALIBRATION_H  21
  #define P_UP_CALIBRATION_L    22
  #define P_UP_CALIBRATION_H    23

  #define P_TORQUE_ENABLE         (24)
  #define P_LED                   (25)
  #define P_CW_COMPL_MARGIN       (26)
  #define P_CCW_COMPL_MARGIN      (27)
  #define P_CW_COMPL_SLOPE        (28)
  #define P_CCW_COMPL_SLOPE       (29)
  #define P_GOAL_POSITION_L       (30)
  #define P_GOAL_POSITION_H       (31)
  #define P_GOAL_SPEED_L          (32)
  #define P_GOAL_SPEED_H          (33)
  #define P_TORQUE_LIMIT_L        (34)
  #define P_TORQUE_LIMIT_H        (35)
  #define P_PRESENT_POSITION_L    (36)
  #define P_PRESENT_POSITION_H    (37)
  #define P_PRESENT_SPEED_L       (38)
  #define P_PRESENT_SPEED_H       (39)
  #define P_PRESENT_LOAD_L        (40)
  #define P_PRESENT_LOAD_H        (41)
  #define P_PRESENT_VOLTAGE       (42)
  #define P_PRESENT_TEMPERATURE   (43)
  #define P_REGISTERED_INSTRUCTION (44)
  #define P_PAUSE_TIME            (45)
  #define P_MOVING                (46)
  #define P_LOCK                  (47)
  #define P_PUNCH_L               (48)
  #define P_PUNCH_H               (49)


  // Dynamixel Instructions
  #define INST_PING           0x01
  #define INST_READ           0x02
  #define INST_WRITE          0x03
  #define INST_REG_WRITE      0x04
  #define INST_ACTION         0x05
  #define INST_RESET          0x06
  #define INST_DIGITAL_RESET  0x07
  #define INST_SYSTEM_READ    0x0C
  #define INST_SYSTEM_WRITE   0x0D
  #define INST_SYNC_WRITE     0x83
  #define INST_SYNC_REG_WRITE 0x84

  bool DynamixelInitialize(void);

  uint8_t DynamixelGetOneByteValue(byte servoId, byte controlTablePosition);
  uint16_t DynamixelGetTwoByteValue (byte servoId, byte controlTablePosition);
  bool DynamixelSetOneByteValue(byte servoID, byte controlTablePosition, uint8_t value);
  bool DynamixelSetTwoByteValue(byte servoID, byte controlTablePosition, uint16_t value);

  void DynamixelRetryErrorHandler ();

  uint16_t DynamixelGetModelNumber (byte servoID);
  uint8_t DynamixelGetFirmwareVersion (byte servoID);
  uint8_t DynamixelGetMotorID (byte servoID);
  uint8_t DynamixelGetBaudRate (byte servoID);
  uint8_t DynamixelGetReturnDelayTime (byte servoID);
  uint16_t DynamixelGetCWAngleLimit (byte servoID);
  uint16_t DynamixelGetCCWAngleLimit (byte servoID);
  uint8_t DynamixelGetHighestLimitTemperature (byte servoID);
  uint8_t DynamixelGetLowestLimitVoltage (byte servoID);
  uint8_t DynamixelGetHighestLimitVoltage (byte servoID);
  uint16_t DynamixelGetMaxTorque (byte servoID);
  uint8_t DynamixelGetStatusReturnLevel (byte servoID);
  uint8_t DynamixelGetAlarmLED (byte servoID);
  uint8_t DynamixelGetAlarmShutdown (byte servoID);

  uint8_t DynamixelGetTorqueEnable (byte servoID);
  uint8_t DynamixelGetLED (byte servoID);
  uint8_t DynamixelGetCWComplianceMargin (byte servoID);
  uint8_t DynamixelGetCWComplianceSlope (byte servoID);
  uint8_t DynamixelGetCCWComplianceMargin (byte servoID);
  uint8_t DynamixelGetCCWComplianceSlope (byte servoID);
  uint16_t DynamixelGetGoalPosition (byte servoID);
  uint16_t DynamixelGetGoalSpeed (byte servoID); // this is a limit setting, not the current speed of the motor
  uint16_t DynamixelGetTorqueLimit (byte servoID);
  uint16_t DynamixelGetMotorPosition (byte servoID);
  uint16_t DynamixelGetSpeed (byte servoID);
  uint16_t DynamixelGetLoad (byte servoID);
  uint8_t DynamixelGetVoltage (byte servoID);
  uint8_t DynamixelGetTemperature (byte servoID);
  uint8_t DynamixelGetMoving (byte servoID);
  uint16_t DynamixelGetPunch (byte servoID);


  bool DynamixelSetMotorID (byte servoID, uint8_t theID);
  bool DynamixelSetBaudRate (byte servoID, uint8_t baudrate);
  bool DynamixelSetReturnDelayTime (byte servoID, uint8_t rdt);
  bool DynamixelSetCWAngleLimit (byte servoID, uint16_t cwAngleLimit);
  bool DynamixelSetCCWAngleLimit (byte servoID, uint16_t cwAngleLimit);
  bool DynamixelSetHighestLimitTemperature (byte servoID, uint8_t hlt);
  bool DynamixelSetLowestLimitVoltage (byte servoID, uint8_t llt);
  bool DynamixelSetHighestLimitVoltage (byte servoID, uint8_t hlv);
  bool DynamixelSetMaxTorque (byte servoID, uint16_t maxTorque);
  bool DynamixelSetStatusReturnLevel (byte servoID, uint8_t srl);
  bool DynamixelSetAlarmLED (byte servoID, uint8_t alarmLed);
  bool DynamixelSetAlarmShutdown (byte servoID, uint8_t alarmShutdown);
  bool DynamixelSetReturnLevel (byte servoID, uint8_t srl);

  bool DynamixelSetTorqueEnable (byte servoID, uint8_t onoff01);
  bool DynamixelSetLED (byte servoID, uint8_t onOff01);
  bool DynamixelSetCWComplianceMargin (byte servoID, uint8_t cwComplianceMargin);
  bool DynamixelSetCCWComplianceMargin (byte servoID, uint8_t ccwComplianceMargin);
  bool DynamixelSetCWComplianceSlope (byte servoID, uint8_t cwComplianceSlope);
  bool DynamixelSetCCWComplianceSlope (byte servoID, uint8_t ccwComplianceSlope);
  bool DynamixelSetGoalPosition (byte servoID, uint16_t goalPosition);
  bool DynamixelSetGoalPositionWithRetries (byte servoID, uint16_t goalPosition);
  bool DynamixelSetGoalSpeed (byte servoID, uint16_t movingSpeed);
  bool DynamixelSetGoalSpeedWithRetries (byte servoID, uint16_t goalPosition);
  bool DynamixelSetTorqueLimit (byte servoID, uint16_t torqueLimit);
  bool DynamixelSetPunch (byte servoID, uint16_t punch);



  // helper functions
  bool DynamixelSetComplianceMargins (byte servoID, uint8_t complianceMargins);
  bool DynamixelSetComplianceSlopes (byte servoID, uint8_t complianceSlope);

  bool DynamixelMoveTo(byte servoID,uint16_t pos,uint16_t speed);

  bool DynamixelEnableContinuousRotation ( byte servoID);
  bool DynamixelEnableContinuousRotationWithRetries (byte servoID);
  bool DynamixelDisableContinuousRotation ( byte servoID);
  bool DynamixelDisableContinuousRotationWithRetries (byte servoID);

  uint8_t DynamixelFindAddress (void);
  uint16_t DynamixelGetMomentaryPosition(uint32_t maxAge); // hits cache first

#endif /* MOV_FUNCTIONS_H_ */
