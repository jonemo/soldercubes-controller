/*
 * CubeBrain.c
 *
 * Created: 4/2/2012 3:04:11 PM
 *  Author: Jonas
 */ 

#define ENABLE_BIT_DEFINITIONS
#define F_CPU                 16000000UL
#define RS232_BAUD_RATE       34   //57600bps at 16MHz
#define RS485_BAUD_RATE       0x01 // 1 MHz
#define RS485_BAUD_RATE_IMP   34 //57600bps at 16MHz

#define MODE_ACTUATED_CUBE    1
#define MODE_PASSIVE_CUBE     2
#define MODE_BATTERY_CUBE     3
#define MODE_ELECTRICIMP_CUBE 4
#define MODE_LED_CUBE         5
#define MODE_WHEEL_CUBE       6

#define DIRECTION_UNDEFINED   0
#define DIRECTION_POSITIVE    1
#define DIRECTION_NEGATIVE    2
/*
  Version 1 
    
#define VERSION1  1
  
  End of Version 1 specific code
*/
  
/*
  Version 2 
*/
#define VERSION2  1
/* 
  End of Version 2 specific code
*/

#include <avr/io.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdio.h>
#include <util/delay.h>

typedef unsigned char byte;
typedef unsigned int word;

#define HEARTBEAT_EVERY_X_MS      500  // how frequent is the heartbeat blink (in ms)
#define HEARTBEAT_BLINK_DURATION  50  // how many milliseconds is the led meant to be on during heartbeat blink

#define DYNAMIXEL_MAX_SPEED                 0x140 // 0x200 // the maximal rotational speed of the dynamixel motor [1..0x03ff]
#define DYNAMIXEL_MIN_SPEED                 0x40
#define DYNAMIXEL_DEFAULT_SPEED             0x140
#define DYNAMIXEL_DEFAULT_BACKLASH_TICKS    0x0   // how many ticks doe the rotation overshoot? compensates for that by modifying the endpoint of each motion
#define DYNAMIXEL_MAX_BACKLASH              35    // how much backlash can be accomodated by the positions allowed for by the initialization routine? in motor ticks
#define DYNAMIXEL_MAX_RETRIES               5     // how often to retry sending a message to the dynamixel
#define DYNAMIXEL_DEAD_BAND                 4     // how close to the actual goal we want to get before we accept it as reaching the goal

// this is counting actual time in milliseconds so that we can do scripted 
// things. note that 2^32 ms = 1193 hours
volatile uint32_t time = 0;        
// the address of this cube (used in RS232 communication, not in RS485 communication)
volatile uint16_t myAddress;
// the operating mode we are in (one of the constants defined above)
volatile uint8_t myOperatingMode;
// due to backlash, we need to compensate on the target positions of rotations. 
// this variable stores by how much
volatile uint8_t myDynamixelBacklashOffset;
// cubes in MODE_ACTIVE_CUBE can go into a Potentiometer Alignment "sub-mode" 
// where the LED indicates if the current potentiometer position is a valid 
// quarter-turn-increment-position
volatile bool amIinPotentiometerAlignment = false;
// for debugging a mode can be entered where the cube regularly broadcasts the
// dynamixel encoder reading. naturally this only works in MODE_ACTIVE_CUBE. no
// checks are implemented for avoiding several cubes talking at the same time.
// this mode does not persist over a shutdown
volatile bool amIinPotentiometerDebugMode = false;
// same thing as above, but for the accelerometer
volatile bool amIinAccelerometerDebugMode = false;
// Calibration is when the endpoints of 90deg rotations get set. This mode also
// gets abused for making small movements during normal operation.
volatile bool amIinCalibrationMode = false;

// variables for managing heaters
volatile bool     heaterState[6];
volatile uint32_t heaterStarted[6];
volatile uint32_t heaterDuration[6];

// variables for managing rotation
volatile uint8_t  currentRotationIncrement = 0; // e [0, 1, 2, 3], stores which quarterly increment the dynamixel should currently be at
volatile uint16_t momentaryPos;
volatile uint32_t momentaryPosLastUpdated = 0;
volatile bool     previousRotationGotStuck = false;

// motorPos and targetPos are used for potentiometer alignment
uint16_t motorPos = 0;
uint16_t targetPos = 0;

// for macros the cube needs to hold the next rotation command
uint16_t bufferedNextTargetPos = 0xFFFF; // set initial value to sth high so that error checking would catch it if it gets actually used by accident
uint8_t bufferedNextSpeed = 0xFF;
uint8_t bufferedNextDirection = 0xFF;
uint8_t bufferedNextGoalIncrement = 0xFF;
uint8_t bufferedStartTime = 0xFF;

#include "ByteHelpers.h"
#include "IO.c"
#include "LED.c"
#include "SerialCom.c"
#include "Dynamixel.c"
#include "Time.c"
#include "LSM303DLHC.c"
#include "../EEPROM/EEPROM.h"
#include "rotate.c"


int main(void)
{  
  // On the debug system we needed some extra time for the crystal to swing in
  _delay_ms(700);
  
  start_booting:
  
  // read address of cube from EEPROM
  myAddress = EEPROM_getStoredAddress();
    
  // setup for newborn cubes
  if (myAddress == 0xFFFF)
  {
    EEPROM_setStoredOperatingMode(MODE_PASSIVE_CUBE);
    EEPROM_setStored90degIncrement(0xFF);
    for (uint8_t i=0; i<4; i++) EEPROM_setStoredCalibPosition(i, 0x00);
    EEPROM_setStoredDynamixelBacklash(DYNAMIXEL_DEFAULT_BACKLASH_TICKS);
  }
  
  // read operating mode from EEPROM and check if it is one of the valid ones
  myOperatingMode = EEPROM_getStoredOperatingMode();
  if (myOperatingMode != MODE_ACTUATED_CUBE &&
  myOperatingMode != MODE_PASSIVE_CUBE &&
  myOperatingMode != MODE_BATTERY_CUBE &&
  myOperatingMode != MODE_ELECTRICIMP_CUBE &&
  myOperatingMode != MODE_LED_CUBE &&
  myOperatingMode != MODE_WHEEL_CUBE)
  {
    // if EEPROM contains bs data, assume it's a passive cube
    myOperatingMode = MODE_PASSIVE_CUBE;
  }  
  
  //Port In/Out Direction Definition
  // This has to happen after the operating mode is determined 
  // or else we'll get a bright red flash from the LED module
  InitializeIOPins();
  
  // flash LEDs to signal that we are alive
  SetStatusLEDs(1,1);
  _delay_ms(100);
  SetStatusLEDs(0,0);
  
  // initialize heater variables
  for (uint16_t i=0; i<6; i++)
  {
    heaterState[i] = false;
    heaterStarted[i] = 0;
    heaterDuration[i] = 0;
  }  
    
  // set up two wire interface (aka I2C)
  twi_init();
    
  // flash LEDs to signal that we are alive
  _delay_ms(100);
  SetStatusLEDs(1,1);
  _delay_ms(100);
  SetStatusLEDs(0,0);  
  
  // set up baud rates for normal serial communication
  RS232Initialize( RS232_BAUD_RATE );
  
  // if this is an actuated cube, set up Dynamixel serial communication
  if (myOperatingMode == MODE_ACTUATED_CUBE || myOperatingMode == MODE_WHEEL_CUBE) 
  {
    RS485Initialize( RS485_BAUD_RATE );
    RS485_RXD; // set RS485 and inter-cube communication to Listen State.
  }
  else if (myOperatingMode == MODE_ELECTRICIMP_CUBE)
  {
    RS485Initialize( RS485_BAUD_RATE_IMP );
  }
  else if (myOperatingMode == MODE_LED_CUBE) 
  {
    InitializeIOPinsForLEDModule(0x05, 0x08, 0x05);
  }
  
  // set up time that keeps global time
  InitializeTimer();
  
  // Enable Interrupt -- Compiler Function
  sei();
  
  // flash LEDs again to signal that we are alive
  _delay_ms(100);
  SetStatusLEDs(1,1);
  _delay_ms(100);
  SetStatusLEDs(0,0);
  
  // initialize the accelerometer
  initializeLSM303DLHC();
  
  if (myOperatingMode == MODE_ACTUATED_CUBE)
  {
    // if initialization fails, we act like a passive cube
    if ( ! DynamixelInitialize() )
    {
      myOperatingMode = MODE_PASSIVE_CUBE;
    }
  }
    
  // read the dynamixel backlash offset value from EEPROM
  myDynamixelBacklashOffset = EEPROM_getStoredDynamixelBacklash();
  // 0x40 is already outrageously much, lets cap it there
  if (myDynamixelBacklashOffset > 0x40) myDynamixelBacklashOffset = 0x40;

  // flash LEDs again to signal that we are alive
  _delay_ms(100);
  SetStatusLEDs(1,1);
  _delay_ms(100);
  SetStatusLEDs(0,0);  
  
  // delay by an as random as possible number so that people dont speak at the same time
  srand(getXAcceleration() + getYAcceleration() + getZAcceleration());
  int randomNumber = rand() % 2000;
  for (int i=0; i<randomNumber; i++) _delay_ms(2);    
  
  sendMessageToBase('~', myOperatingMode, 0, msb(momentaryPos), lsb(momentaryPos));
  
  while(1)
  {
    
    /************************************************************************/
    /* Heartbeat                                                            */
    /************************************************************************/
    
    // motor cubes in alignment mode don't heartbeat
    if (myOperatingMode == MODE_ACTUATED_CUBE && amIinPotentiometerAlignment == true)
    {
      // try reading the motor position five times
      motorPos = DynamixelGetMotorPosition(DYNAMIXEL_ADDRESS);
      
      // catch when no valid motor position could be read and signal it
      if (motorPos == 0xffff)
      {
        LEDMorse(2, 2);
      }
      else
      {
        // Set motor LED according to whether this is a save interval position
        if ( (motorPos > 0 + DYNAMIXEL_MAX_BACKLASH && motorPos < 102 - DYNAMIXEL_MAX_BACKLASH) ||
        (motorPos > 307 + DYNAMIXEL_MAX_BACKLASH && motorPos < 409 - DYNAMIXEL_MAX_BACKLASH) ||
        (motorPos > 615 + DYNAMIXEL_MAX_BACKLASH && motorPos < 717 - DYNAMIXEL_MAX_BACKLASH) ||
        (motorPos > 922 + DYNAMIXEL_MAX_BACKLASH && motorPos < 1024 - DYNAMIXEL_MAX_BACKLASH) )
        {
          SetStatusLEDs(0,1); // green
        }
        else
        {
          SetStatusLEDs(1,0); // red
        }
      }
    }
    // everyone else has a heartbeat
    else if ( time % HEARTBEAT_EVERY_X_MS <= HEARTBEAT_BLINK_DURATION)
    {
      if (myOperatingMode == MODE_ACTUATED_CUBE) 
      {
        SetStatusLEDs(0,1); // green 
      }
      else
      {
        SetStatusLEDs(1,0); // red
      }
    }
    else
    {
      SetStatusLEDs(0,0);
    }

    /************************************************************************/
    /* MOTOR ENCODER DEBUGGING                                              */
    /************************************************************************/
    
    if ( myOperatingMode == MODE_ACTUATED_CUBE && amIinPotentiometerDebugMode && time % 2000 < 5 )
    {
      // update momentaryPos reading
      DynamixelGetMomentaryPosition(50);

      TxRS232_String("[");
      TxRS232_Hex2Byte(myAddress);
      TxRS232_String("] momPos: 0x");
      TxRS232_Hex2Byte(momentaryPos);
      TxRS232_String("\r\n");
    }
    
    if ( amIinAccelerometerDebugMode && time % 2000 < 5 ) 
    {
      TxRS232_String("[");
      TxRS232_Hex2Byte(myAddress);
      TxRS232_String("] Acc: 0x");
      TxRS232_Hex2Byte( getXAcceleration() );
      TxRS232_String(", 0x");
      TxRS232_Hex2Byte( getYAcceleration() );
      TxRS232_String(", 0x");
      TxRS232_Hex2Byte( getZAcceleration() );
      TxRS232_String("\r\n");
    }

    /************************************************************************/
    /* CHECKING THAT WE ARE IN THE RIGHT PLACE                              */
    /************************************************************************/

    if (myOperatingMode == MODE_ACTUATED_CUBE &&      // we can only correct the position to actuated cubes
        ! amIinCalibrationMode &&                     // don't try to correct while we are calibrating
        ! amIinPotentiometerAlignment &&              // don't try to correct while we are assembling the cube
        time % 2000 < 5 &&                            // don't do this at every step, that would be too slow
        EEPROM_getStoredCalibPosition(0) != 0x0000 && EEPROM_getStoredCalibPosition(0) != 0xFFFF && 
        EEPROM_getStoredCalibPosition(1) != 0x0000 && EEPROM_getStoredCalibPosition(1) != 0xFFFF && 
        EEPROM_getStoredCalibPosition(2) != 0x0000 && EEPROM_getStoredCalibPosition(2) != 0xFFFF && 
        EEPROM_getStoredCalibPosition(3) != 0x0000 && EEPROM_getStoredCalibPosition(3) != 0xFFFF
       ) // check that the cube knows about calib positions (part of upgrade process, remove when all cubes now how stuff works)
    {
      uint8_t eeprom_stored90deg_increment = EEPROM_getStored90degIncrement();
      uint16_t endingpos = EEPROM_getStoredCalibPosition(eeprom_stored90deg_increment);  
      
      DynamixelGetMomentaryPosition(0);
      uint8_t direction = DIRECTION_POSITIVE;
      
      if (abs(endingpos - momentaryPos) > DYNAMIXEL_DEAD_BAND && 
          momentaryPos != 0x00 && 
          momentaryPos != 0x3FF )
      {
        if ((endingpos > momentaryPos && endingpos - momentaryPos > 0x266) ||  // 0x266 encoder ticks = 180 degrees
            (endingpos < momentaryPos && momentaryPos - endingpos < 0x266 ) )
        {
          direction = DIRECTION_NEGATIVE;           
        }
        
        TxRS232_String("[");
        TxRS232_Hex2Byte(myAddress);
        TxRS232_String("] Error correct to ");
        TxRS232_Hex2Byte(endingpos);
        TxRS232_String("\r\n");
        rotate(endingpos, DYNAMIXEL_DEFAULT_SPEED, direction);
      }
    }     
    
    /************************************************************************/
    /* DEALING WITH HEATERS                                                 */
    /************************************************************************/
    
    for (uint16_t i=0; i<6; i++) 
    {
      if (heaterState[i]) 
      {
        if (heaterStarted[i] + heaterDuration[i] < time) 
        {
          SwitchHeatOff(i+1); // note: arrays are zero based, heaters are 1 based
          heaterState[i] = false;
        }
      }
    }
    
    
    /************************************************************************/
    /* ELECTRIC IMP MODULE SPECIFIC                                         */
    /************************************************************************/
    
    if (myOperatingMode == MODE_ELECTRICIMP_CUBE)
    {
      char incomingFromElectricImp[128];
      uint8_t incomingFromElectricImpCounter = 0;
      char outgoingToElectricImp[128];
      uint8_t outgoingToElectricImpCounter = 0;
      
      // read up to 128 bytes that came in via RS232 into a temporary buffer
      // yes, we discard everything else. that's because below we have to make sure
      // the imp doesn't it's own content back, and for that we need to set
      // RS232RxBufferReadPointer = RS232RxBufferWritePointer
      while (RS232RxBufferReadPointer != RS232RxBufferWritePointer && outgoingToElectricImpCounter < 0xFF)
      {
        byte outgoingByte = RS232RxInterruptBuffer[(RS232RxBufferReadPointer)++];
        
        // queue up character for re-broadcasting to electric imp
        outgoingToElectricImp[outgoingToElectricImpCounter] = outgoingByte;
        outgoingToElectricImpCounter++;
      }
      
      // read up to 128 bytes coming from the electric imp into a temporary buffer 
      while(RS485RxBufferReadPointer != RS485RxBufferWritePointer && incomingFromElectricImpCounter < 0xFF)
      {
        byte incomingByte = RS485RxInterruptBuffer[(RS485RxBufferReadPointer)++];
        
        // queue up character for re-broadcasting on the RS232 communications bus 
        incomingFromElectricImp[incomingFromElectricImpCounter] = incomingByte;
        incomingFromElectricImpCounter++;
        
        // add this byte to our own incoming RS232 buffer
        RxRS232(incomingByte, time);
      }
      
      // if there were incoming bytes, re-broadcast them
      if (incomingFromElectricImpCounter > 0) 
      {
        // Using RxRS232 results in RS232RxBufferReadPointer != RS232RxBufferWritePointer
        // to make sure the imp doesn't hear itself talk, we set the two equal. this
        // results in loss of any info not yet re-broadcast via the imp
        RS232RxBufferReadPointer = RS232RxBufferWritePointer;
              
        // with all bytes in, take a look at our inbound buffer
        searchMessageInRS232InterruptBuffer();
        
        // re-broadcast what we just heard from the imp locally
        TxRS232_String2(incomingFromElectricImp, incomingFromElectricImpCounter);
      }
      
      if (outgoingToElectricImpCounter > 0)
      {
        TxRS485_String2(outgoingToElectricImp, outgoingToElectricImpCounter);
      }
    }
    
    /************************************************************************/
    /* DEALING WITH MESSAGES                                                */
    /************************************************************************/
    
    // is messageQueueLength more than 0? then we got work to do
    if (! messageQueueEmpty())
    {
      
      
      // START BROADCASTING ENCODER READING                         '('   0x28
      if (messageQueue[messageQueueStart].command == '(')
      {
        amIinPotentiometerDebugMode = (messageQueue[messageQueueStart].databyte[0] != 0x00);
        amIinAccelerometerDebugMode = (messageQueue[messageQueueStart].databyte[1] != 0x00);
        
        // respond
        sendMessageToBase('{', 0, 0, 0, 0);
      }
      
      
      
      // STOP BROADCASTING ENCODER READING                          '('   0x29
      else if (messageQueue[messageQueueStart].command == ')')
      {
        amIinPotentiometerDebugMode = false;
        amIinAccelerometerDebugMode = false;
        
        // respond
        sendMessageToBase('}', 0, 0, 0, 0);
      }
      
      
      
      // CHANGE ADDRESS                                             'A'   0x41
      else if (messageQueue[messageQueueStart].command == 'A')
      {
        char mymsb = messageQueue[messageQueueStart].databyte[0];
        char mylsb = messageQueue[messageQueueStart].databyte[1];
        
        // validate what that would be in a tmp variable first
        uint16_t tmpAddr = combineTwoBytes(mymsb, mylsb);
        
        // new address can't be the base station or broadcast address
        if (tmpAddr != 0x20 && tmpAddr != 0x21) 
        {
          // update the volatile variable
          myAddress = tmpAddr;
          // store in EEPROM
          EEPROM_setStoredAddress(myAddress);
        }
        
        // Respond with current address. Will be the new one if it was updated 
        // or the unchanged one in case of error.
        sendMessageToBase('a', msb(myAddress), lsb(myAddress), 0, 0);
      }
      
      
      
      // READ/CHANGE BACKLASH                                            'B'   0x42
      else if (messageQueue[messageQueueStart].command == 'B')
      {
        char tmpBacklash = messageQueue[messageQueueStart].databyte[0];
        bool write = (messageQueue[messageQueueStart].databyte[3] == 1);
        
        if (write && tmpBacklash < 0x40) 
        {
          // update the volatile variable
          myDynamixelBacklashOffset = tmpBacklash;
          // store in EEPROM
          EEPROM_setStoredDynamixelBacklash(myDynamixelBacklashOffset);
        }
      
        // Respond with current backlash. Will be the new one if it was updated
        // or the unchanged one in case of error.
        sendMessageToBase('b', myDynamixelBacklashOffset, 0, 0, 0);
      }
      
      
      
      // DO CONTINUOUS ROTATION                                     'C'   0x43
      else if (messageQueue[messageQueueStart].command == 'C')
      {
        TxRS232_String("cont rotation\r\n");
        
        char myspeed = messageQueue[messageQueueStart].databyte[0];
        char mydirection = messageQueue[messageQueueStart].databyte[1];
            
        if (mydirection > 0 && mydirection < 3)
        {
          if (myspeed == 0)
          {
            // stop rotating
            DynamixelDisableContinuousRotation(DYNAMIXEL_ADDRESS);
            
            // respond
            // databyte[0] = 2 => continuous rotation ended
            sendMessageToBase('c', 2, 0, 0, 0);
          }
          else
          {
            int speed = (mydirection == 1) ? myspeed : (0x400 ^ myspeed);
            
            // start rotating
            DynamixelEnableContinuousRotation(DYNAMIXEL_ADDRESS);
            DynamixelSetGoalSpeed(DYNAMIXEL_ADDRESS, speed);
            
            // respond
            // databyte[0] = 1 => continuous rotation started
            sendMessageToBase('c', 1, 0, 0, 0);
          }
        }
        else
        {
          // respond
          // databyte[0] = 3 => error
          sendMessageToBase('c', 3, 0, 0, 0);
        }
      }
      
      
      
      // DEBUG MESSAGE                                              'D'   0x44
      else if (messageQueue[messageQueueStart].command == 'D') 
      {
        TxRS232_String("myAddress (stored): 0x");               TxRS232_Hex(EEPROM_getStoredAddress()); TxRS232_String("\r\n");
        TxRS232_String("myOperatingMode: 0x");                  TxRS232_Hex(myOperatingMode); TxRS232_String("\r\n");
        TxRS232_String("myStoredOperatingMode: 0x");            TxRS232_Hex(EEPROM_getStoredOperatingMode()); TxRS232_String("\r\n");
        TxRS232_String("myStored90degIncrement: 0x");           TxRS232_Hex(EEPROM_getStored90degIncrement()); TxRS232_String("\r\n");
        TxRS232_String("myStoredDynamixelBacklash: 0x");        TxRS232_Hex(EEPROM_getStoredDynamixelBacklash()); TxRS232_String("\r\n");
        for (int i=0; i<4; i++) {
          TxRS232_String("myStoredCalibPosition");              TxRS232_Hex(i); TxRS232_String(": 0x"); TxRS232_Hex2Byte(EEPROM_getStoredCalibPosition(i)); TxRS232_String("\r\n");
        }
        
        if (myOperatingMode == MODE_ACTUATED_CUBE) 
        {
          TxRS232_String("Dynamixel Model Number: 0x");           TxRS232_Hex2Byte( DynamixelGetModelNumber (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Firmware Version: 0x");       TxRS232_Hex( DynamixelGetFirmwareVersion (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Motor ID: 0x");               TxRS232_Hex( DynamixelGetMotorID (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Baud Rate: 0x");              TxRS232_Hex( DynamixelGetBaudRate (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Return Delay Time: 0x");      TxRS232_Hex( DynamixelGetReturnDelayTime (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel CW Angle Limit: 0x");         TxRS232_Hex2Byte( DynamixelGetCWAngleLimit (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel CCW Angle Limit: 0x");        TxRS232_Hex2Byte( DynamixelGetCCWAngleLimit (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Highest Limit Temp: 0x");     TxRS232_Hex( DynamixelGetHighestLimitTemperature (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Lowest Limit Voltage: 0x");   TxRS232_Hex( DynamixelGetLowestLimitVoltage (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Highest Limit Voltage: 0x");  TxRS232_Hex( DynamixelGetHighestLimitVoltage (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Max Torque: 0x");             TxRS232_Hex2Byte( DynamixelGetMaxTorque (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Status Return Level: 0x");    TxRS232_Hex( DynamixelGetStatusReturnLevel (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Alarm LED: 0x");              TxRS232_Hex( DynamixelGetAlarmLED (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Alarm Shutdown: 0x");         TxRS232_Hex( DynamixelGetAlarmShutdown (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
        
          TxRS232_String("Dynamixel Torque Enable: 0x");          TxRS232_Hex( DynamixelGetTorqueEnable (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel LED: 0x");                    TxRS232_Hex( DynamixelGetLED (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel CW Compliance Margin: 0x");   TxRS232_Hex( DynamixelGetCWComplianceMargin (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel CW Compliance Slope: 0x");    TxRS232_Hex( DynamixelGetCWComplianceSlope (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel CCW Compliance Margin: 0x");  TxRS232_Hex( DynamixelGetCCWComplianceMargin (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel CCW Compliance Slope: 0x");   TxRS232_Hex( DynamixelGetCCWComplianceSlope (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Goal Position: 0x");          TxRS232_Hex2Byte( DynamixelGetGoalPosition (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Goal Speed: 0x");             TxRS232_Hex2Byte( DynamixelGetGoalSpeed (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n"); // this is a limit setting, not the current speed of the motor
          TxRS232_String("Dynamixel Torque Limit: 0x");           TxRS232_Hex2Byte( DynamixelGetTorqueLimit (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Current Position: 0x");       TxRS232_Hex2Byte( DynamixelGetMotorPosition (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Current Speed: 0x");          TxRS232_Hex2Byte( DynamixelGetSpeed (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Current Load: 0x");           TxRS232_Hex2Byte( DynamixelGetLoad (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Current Voltage: 0x");        TxRS232_Hex( DynamixelGetVoltage (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Current Temperature: 0x");    TxRS232_Hex( DynamixelGetTemperature (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Currently Moving: 0x");       TxRS232_Hex( DynamixelGetMoving (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
          TxRS232_String("Dynamixel Punch: 0x");                  TxRS232_Hex2Byte( DynamixelGetPunch (DYNAMIXEL_ADDRESS) ); TxRS232_String("\r\n");
        }
        
        TxRS232_String("Acceleration X: 0x");                   TxRS232_Hex2Byte( getXAcceleration() ); TxRS232_String("\r\n");
        TxRS232_String("Acceleration Y: 0x");                   TxRS232_Hex2Byte( getYAcceleration() ); TxRS232_String("\r\n");
        TxRS232_String("Acceleration Z: 0x");                   TxRS232_Hex2Byte( getZAcceleration() ); TxRS232_String("\r\n");
        TxRS232_String("Magnetic Field X: 0x");                 TxRS232_Hex2Byte( getXMagField() ); TxRS232_String("\r\n");
        TxRS232_String("Magnetic Field Y: 0x");                 TxRS232_Hex2Byte( getYMagField() ); TxRS232_String("\r\n");
        TxRS232_String("Magnetic Field Z: 0x");                 TxRS232_Hex2Byte( getZMagField() ); TxRS232_String("\r\n");
        
        
        // respond, confirm end of message
        sendMessageToBase('d', 0, 0, 0, 0);
      }
      
      
      
      // FIND MOTOR                                                 'F'   0x46
      else if (messageQueue[messageQueueStart].command == 'F')
      {
        uint8_t dxAddr = DynamixelFindAddress();
        if (dxAddr > 0 && dxAddr < 0xfd)
        {
          TxRS232_String("Found Motor at Address 0x: "); 
          TxRS232_Hex(dxAddr >> 8); 
          TxRS232_Hex(dxAddr & 0xff); 
          TxRS232_String("\r\n");
          
          // set the motor ID to the default
          DynamixelSetMotorID(dxAddr, 1);
          
          // confirm that this worked
          dxAddr = DynamixelFindAddress();
          
          // respond: this should always return 1, unless 
          // setting the new value didn't work for some reason
          sendMessageToBase('f', dxAddr, 0, 0, 0);
        }
        else
        {
          // respond: no dynamixel found, reply 0xFF
          sendMessageToBase('f', 0xFF, 0, 0, 0);
        }
      }
      
      
      
      // SWITCH HEATER                                              'H'   0x48
      else if (messageQueue[messageQueueStart].command == 'H')
      {
        // variable used in the response sent back
        uint8_t respOn = 0, respOff = 0, respTime = 0;
        
        // read the requirements from the data bytes
        uint8_t reqOn = messageQueue[messageQueueStart].databyte[0];
        uint8_t reqOff = messageQueue[messageQueueStart].databyte[1];
        uint8_t reqDuration = messageQueue[messageQueueStart].databyte[2];
        
        // switch valves as per the command
        for (char i=1; i<=6; i++)
        {
          if (reqOn == i)
          {
            SwitchHeatOn(i);
            heaterState[i-1] = true; // note: arrays are zero based, heaters are 1 based
            heaterStarted[i-1] = time;
            
            respOn = i;
            
            // the third byte could be the heater duration
            if (reqDuration > 0 && reqDuration <= 60) 
            {
              heaterDuration[i-1] = reqDuration * 1000;
              respTime = reqDuration;
            }
            else 
            {
              heaterDuration[i-1] = 0;
            }
          }
          if (reqOff == i)
          {
            SwitchHeatOff(i);
            heaterState[i-1] = false;
            respOff = i;
          }
        }

        // respond
        sendMessageToBase('h', respOn, respOff, respTime, 0);
      }
      
      
      
      // ENTER POTENTIOMETER ALIGNMENT MODE                         'I'   0x49
      // (the I originates from "Initialize")
      else if (messageQueue[messageQueueStart].command == 'I')
      {
        // potentiometer alignment is only possible when the cube is an actuated cube
        if (myOperatingMode == MODE_ACTUATED_CUBE)
        {
          // remember that we are in alignment mode
          amIinPotentiometerAlignment = true;
          
          // initialize motorPos and targetPos for this alignment session
          motorPos = DynamixelGetMotorPosition(DYNAMIXEL_ADDRESS);
          // enable endless turn mode
          DynamixelSetTorqueEnable(DYNAMIXEL_ADDRESS, 0);
        }
        
        // respond
        sendMessageToBase('i', myOperatingMode, 0, 0, 0);
      }
      
      
      
      // LEAVE POTENTIOMETER ALIGNMENT MODE                         'J'   0x4A
      else if (messageQueue[messageQueueStart].command == 'J')
      {
        amIinPotentiometerAlignment = false;
        
        // the cube should always be in actuated cube mode, but just in case lets differentiate
        if (0 == DynamixelInitialize()) 
        {
          // dynamixel initialization didn't work
          myOperatingMode = MODE_PASSIVE_CUBE;
        }          
        else 
        {
          myOperatingMode = MODE_ACTUATED_CUBE;
          DynamixelSetTorqueEnable(DYNAMIXEL_ADDRESS, 0x01);
        } 
        
        // respond
        sendMessageToBase('j', myOperatingMode, 0, 0, 0);         
      }
  
  
  
      // SWITCH LED                                                 'L'   0x4C
      else if (messageQueue[messageQueueStart].command == 'L')
      {
        // validate the blink count first, max is 10
        uint8_t myblinkcount = messageQueue[messageQueueStart].databyte[0];
        if (myblinkcount > 10) myblinkcount = 3;
        
        // if the green LED was on for some reason, we switch it off now
        SetStatusLEDs(0,0);
        _delay_ms(20);
        
        // respond (before spending time doing it to free up the comms bus)
        sendMessageToBase('l', myblinkcount, 0, 0, 0);
        
        // first data byte contains how many times we are supposed to blink
        LEDMorse(myblinkcount, 0);
      }
      
      
      
      // CHANGE OPERATING MODE OF THE CUBE                         'M'   0x4D
      else if (messageQueue[messageQueueStart].command == 'M')
      {
        char mynewoperatingmode = messageQueue[messageQueueStart].databyte[0];
        // validate data
        if ( mynewoperatingmode == MODE_ACTUATED_CUBE ||
            mynewoperatingmode == MODE_BATTERY_CUBE ||
            mynewoperatingmode == MODE_PASSIVE_CUBE ||
            mynewoperatingmode == MODE_ELECTRICIMP_CUBE ||
            mynewoperatingmode == MODE_LED_CUBE ||
            mynewoperatingmode == MODE_WHEEL_CUBE)
        {
          // update the volatile variable
          myOperatingMode = (messageQueue[messageQueueStart].databyte[0]);
          // store in EEPROM
          EEPROM_setStoredOperatingMode(myOperatingMode);
        }
        
        // respond
        sendMessageToBase('m', myOperatingMode, 0, 0, 0);
        
        // normally this is done at the end of the while(1) loop, but since we 
        // are "rebooting" we need to move this message out of our incoming buffer
        incrementMessageQueueStart();
        
        // reboot into new operating mode
        goto start_booting;
      }
      
      
      
      // PING (request status)                                      'P' = 0x50
      else if (messageQueue[messageQueueStart].command == 'P')
      {
        if (myOperatingMode == MODE_ACTUATED_CUBE)
        {
          // get a new position reading if the most recent one is more than
          // 500 ms old
          DynamixelGetMomentaryPosition(500);
          //sendMessageToBase('p', 0, 0, msb(momentaryPos), lsb(momentaryPos));
        }
        else 
        {
          //sendMessageToBase('p', 0, 0, 0, 0);
        }
        
        sendMessageToBase('p',
          (0xFF000000 & time) >> 24,
          (0x00FF0000 & time) >> 16,
          (0x0000FF00 & time) >> 8,
          (0x000000FF & time)
        );

      }
      
      
      
      // RATTLE                                                    'Q'   0x51
      else if (messageQueue[messageQueueStart].command == 'Q')
      {
        uint8_t rattleDistance = messageQueue[messageQueueStart].databyte[0];
        uint8_t rattleCount = messageQueue[messageQueueStart].databyte[1];
        uint8_t rattleDelay = messageQueue[messageQueueStart].databyte[2];
        
        // use default values if 0 was given
        if (rattleDistance == 0) rattleDistance = 5;
        if (rattleCount == 0) rattleCount = 3;
        
        DynamixelGetMomentaryPosition(0);
        uint16_t cachedMomentaryPos = momentaryPos;
        
        // determine the limits of the rattling motion
        uint16_t rattleDown = (momentaryPos < rattleDistance) ? 0 : momentaryPos - rattleDistance;
        uint16_t rattleUp = (0x3FF - momentaryPos < rattleDistance) ? 0x3FF : momentaryPos + rattleDistance;
        
        for (uint8_t i=0; i<rattleCount; i++)
        {
          DynamixelSetGoalPositionWithRetries(DYNAMIXEL_ADDRESS, rattleUp);
          for (uint8_t j=0; j<rattleDelay; j++) _delay_ms(100);
          DynamixelSetGoalPositionWithRetries(DYNAMIXEL_ADDRESS, rattleDown);
          for (uint8_t j=0; j<rattleDelay; j++) _delay_ms(100);
        }
        
        DynamixelSetGoalPositionWithRetries(DYNAMIXEL_ADDRESS, cachedMomentaryPos);
        
        // report back that an error occurred
        sendMessageToBase('q', 0, 0, 0, 0);
      }
      
      
      
      // ROTATE                                                    'R' = 0x52
      //
      // Note: 'R' has a different meaning for MODE_LED_CUBE, see below!
      //
      // databyte[0] increment:
      //      1=+90, 2=+180, 3=+270, 4=+360, 
      //      5=-90, 6=-180, 7=-270, 8=-360
      // databyte[1] speed: 
      //      value*10 (deg/sec), max is 9
      // databyte[2] start time:
      //      if set, cube waits until time =< 100*databyte[2]
      //      if set to 0xFF waits for R-broadcast to start
      else if (messageQueue[messageQueueStart].command == 'R' && myOperatingMode == MODE_ACTUATED_CUBE)
      {
        uint8_t myIncrement = messageQueue[messageQueueStart].databyte[0];
        uint8_t mySpeed = messageQueue[messageQueueStart].databyte[1];
        uint8_t myStartTime = messageQueue[messageQueueStart].databyte[2];
        uint8_t myBuffering = messageQueue[messageQueueStart].databyte[3];
        uint32_t messageArrivedAtTime = time;
        
        bool yesDoTheRotation = false;
        uint16_t endingpos;
        uint8_t speed;
        uint8_t direction = DIRECTION_UNDEFINED;
        uint8_t goalincrement;
        
        // if this is a broadcast message to trigger a buffered movement 
        // and we have buffered values, load the buffered value
        if (messageQueue[messageQueueStart].recipient == WURFELSPEAK_ADDR_BROADCAST && 
            myIncrement == 0xFF &&
            mySpeed == 0xFF
        )
        {
          // only do something if we have something inside the buffer,
          // otherwise do nothing, we are simply not affected.
          if (bufferedNextTargetPos != 0xFFFF &&
              bufferedNextSpeed != 0xFF &&
              (bufferedNextDirection == DIRECTION_POSITIVE || bufferedNextDirection == DIRECTION_NEGATIVE) &&
              bufferedStartTime != 0xFF &&
              bufferedNextGoalIncrement >= 0 && bufferedNextGoalIncrement <=3
          )
          {
            // empty the buffer and reset it, so that it's empty even if something goes
            // wrong during the motion.
            endingpos = bufferedNextTargetPos;
            speed = bufferedNextSpeed;
            direction = bufferedNextDirection;
            goalincrement = bufferedNextGoalIncrement;
            myStartTime = bufferedStartTime;
          
            yesDoTheRotation = true;
          
            bufferedNextTargetPos = 0xFFFF;
            bufferedNextSpeed = 0xFF;
            bufferedNextDirection = 0xFF;
            bufferedNextGoalIncrement = 0xFF;
            bufferedStartTime = 0xFF;
          }          
        }
        // below is where non-broadcast "normal" rotation commands are handled
        // including those who set buffer values and will need to be triggered
        // by a broadcast message
        else if (messageQueue[messageQueueStart].recipient != WURFELSPEAK_ADDR_BROADCAST &&
                 myIncrement > 0 && 
                 myIncrement < 9 )
        {
          // cache the increment into a variable
          uint8_t increment = 0;
          
          // figure out target increment and direction for getting there
          if (myIncrement == 1) { increment = 1; direction = DIRECTION_POSITIVE; }
          else if (myIncrement == 2) { increment = 2; direction = DIRECTION_POSITIVE; }
          else if (myIncrement == 3) { increment = 3; direction = DIRECTION_POSITIVE; }
          else if (myIncrement == 4) { increment = 4; direction = DIRECTION_POSITIVE; }
          else if (myIncrement == 5) { increment = 4 - 1; direction = DIRECTION_NEGATIVE; } // + 4 to avoid negative numbers, they work unexpectedly with the modulo operator
          else if (myIncrement == 6) { increment = 4 - 2; direction = DIRECTION_NEGATIVE; }
          else if (myIncrement == 7) { increment = 4 - 3; direction = DIRECTION_NEGATIVE; }
          else if (myIncrement == 8) { increment = 4 - 4; direction = DIRECTION_NEGATIVE; }
          
          // compute the target 90deg increment
          uint8_t eeprom_stored90deg_increment = EEPROM_getStored90degIncrement();
          goalincrement = (eeprom_stored90deg_increment + increment) % 4;  
          
          // compute ending position
          endingpos = EEPROM_getStoredCalibPosition(goalincrement);
          speed = mySpeed; // note: this will be multiplied by four inside the rotation function
          
          // adjust for backlash
          if (direction == DIRECTION_NEGATIVE) {
            endingpos -= myDynamixelBacklashOffset;
          } else {
            endingpos += myDynamixelBacklashOffset;
          }
          
          // catch goal positions in the dead zone. since we do the complex alignment 
          // procedure this can't really happen any longer, but let's keep the check
          // here just in case.
          if (endingpos > 1024)
          {
            TxRS232_String("[");
            TxRS232_Hex2Byte(myAddress);
            TxRS232_String("] Error: Goal position falls into dead zone: 0x");
            TxRS232_Hex2Byte(endingpos);
            
            // report back that an error occurred
            sendMessageToBase('r', 0xFF, 0xFE, 0, 0);
          }
          else if (myBuffering == 0xFF)
          {
            // databyte[2] == 0xFF means that this rotations is meant to be buffered
            bufferedNextTargetPos = endingpos;
            bufferedNextSpeed = mySpeed;
            bufferedNextDirection = direction;
            bufferedNextGoalIncrement = goalincrement;
            bufferedStartTime = myStartTime;
            
            // report back that we buffered something
            sendMessageToBase('r', 0xFF, 0xF1, 0, 0);
          }
          else 
          {  
            yesDoTheRotation = true;
            
            // report back that a rotation is about to begin
            sendMessageToBase('r', msb(endingpos), lsb(endingpos), 0, 0);
          }
          
        }
        else // if the input params where outside the permissible ranges
        {
          // report back that an error occurred
          sendMessageToBase('r', 0xFF, 0xFF, 0, 0);
        }
        
        if (yesDoTheRotation)
        {
          // write target rotation increment to EEPROM so that we know where we are
          // on startup if we die half way through
          EEPROM_setStored90degIncrement( goalincrement );
          
          // delay for the desired amount of time, if any
          // input comes in tenths of seconds, multiply by 100 to get to milliseconds
          while( time < myStartTime*100);
          
          // clear the previousRotationGotStuck flag because if we really got
          // stuck, sending a rotation command is probably going to solve the problem
          previousRotationGotStuck = false;
          
          // do the rotation
          rotate(endingpos, speed, direction);
        }
      }
      
      
      
      // CHANGE COLOR                                                'R' = 0x52
      //
      // Note: 'R' has a different meaning for MODE_ACTUATED_CUBE, see above!
      //
      // databyte[0]: Red
      // databyte[1]: Green
      // databyte[2]: Blue
      else if (messageQueue[messageQueueStart].command == 'R' && myOperatingMode == MODE_LED_CUBE)
      {
        uint8_t LEDCubeRed    = messageQueue[messageQueueStart].databyte[0];
        uint8_t LEDCubeGreen  = messageQueue[messageQueueStart].databyte[1];
        uint8_t LEDCubeBlue   = messageQueue[messageQueueStart].databyte[2];
        
        SetLED(LEDCubeRed, LEDCubeGreen, LEDCubeBlue);

        sendMessageToBase('r', 0, 0, 0, 0);
      }
      
      
      
      // READ SENSORS                                               'S' = 0x53
      else if (messageQueue[messageQueueStart].command == 'S')
      {
        char out = 0;
        char mySensorId = messageQueue[messageQueueStart].databyte[0];
        
        #ifdef VERSION2
          SwitchSensorsOn ();
          
          // delay for having sensors switch on and acquire first data point?
          // todo: confirm that this long is needed
          _delay_ms(500);
        
          // poll sensors as per the command
          for (char i=1; i<=6; i++)
          {
            if (mySensorId == i || mySensorId == 0)
            {
              if ( ReadSensorValue(i) ) 
              {
                out |= (1 << (i-1));
              }
            }
          }
        
          SwitchSensorsOff ();
        #endif
        
        // respond
        sendMessageToBase('s', out, 0, 0, 0);
      }
      
      
      
      // RESET TIME                                                 'T' = 0x54
      else if (messageQueue[messageQueueStart].command == 'T')
      {
        uint32_t oldtime = time;
        time = 0;
        
        // reset the heater variables to something that will result in the same
        // effect as what they were set to in old time
        for (int i=0; i<6; i++) 
        {
          heaterDuration[i] = heaterDuration[i] - (oldtime - heaterStarted[i]);
          heaterStarted[i] = 0;
        }
      }
      
            
      
      // READ/WRITE DYNAMIXEL CONTROL TABLE ENTRY                   'X' = 0x58
      else if (messageQueue[messageQueueStart].command == 'X')
      {
        
        // cache input values
        uint8_t addr = messageQueue[messageQueueStart].databyte[0];
        uint8_t oneByteVal = messageQueue[messageQueueStart].databyte[2];
        uint16_t twoByteVal = combineTwoBytes(messageQueue[messageQueueStart].databyte[1],
          messageQueue[messageQueueStart].databyte[2]);
        uint8_t readWrite = messageQueue[messageQueueStart].databyte[3];
        
        // cache output
        uint16_t outTwoBytes = 0;
        
        if (readWrite == 1) // databyte[3] == 1 ==> write
        {
          if (addr == P_ID)                     DynamixelSetMotorID( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_BAUD_RATE)         DynamixelSetBaudRate( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_RETURN_DELAY_TIME) DynamixelSetReturnDelayTime( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_CW_ANGLE_LIMIT_L)  DynamixelSetCWAngleLimit( DYNAMIXEL_ADDRESS, twoByteVal );
          else if (addr == P_CCW_ANGLE_LIMIT_L) DynamixelSetCCWAngleLimit( DYNAMIXEL_ADDRESS, twoByteVal );
          else if (addr == P_LIMIT_TEMPERATURE) DynamixelSetHighestLimitTemperature( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_DOWN_LIMIT_VOLTAGE)DynamixelSetLowestLimitVoltage( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_UP_LIMIT_VOLTAGE)  DynamixelSetHighestLimitVoltage( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_MAX_TORQUE_L)      DynamixelSetMaxTorque( DYNAMIXEL_ADDRESS, twoByteVal );
          else if (addr == P_RETURN_LEVEL)      DynamixelSetStatusReturnLevel( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_ALARM_LED)         DynamixelSetAlarmLED( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_ALARM_SHUTDOWN)    DynamixelSetAlarmShutdown( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_TORQUE_ENABLE)     DynamixelSetTorqueEnable( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_LED)               DynamixelSetLED( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_CW_COMPL_MARGIN)   DynamixelSetCWComplianceMargin( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_CCW_COMPL_MARGIN)  DynamixelSetCCWComplianceMargin( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_CW_COMPL_SLOPE)    DynamixelSetCWComplianceSlope( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_CCW_COMPL_SLOPE)   DynamixelSetCCWComplianceSlope( DYNAMIXEL_ADDRESS, oneByteVal );
          else if (addr == P_GOAL_POSITION_L)   DynamixelSetGoalPosition( DYNAMIXEL_ADDRESS, twoByteVal );
          else if (addr == P_GOAL_SPEED_L)      DynamixelSetGoalSpeed( DYNAMIXEL_ADDRESS, twoByteVal );
          else if (addr == P_TORQUE_LIMIT_L)    DynamixelSetTorqueLimit( DYNAMIXEL_ADDRESS, twoByteVal );
          else if (addr == P_PUNCH_L)           DynamixelSetPunch( DYNAMIXEL_ADDRESS, twoByteVal );
        }
        // we read for both read and write modes. for read mode we do it,
        // because that's what read mode is. for write mode we do it to be
        // able to that what we set is actually there.
        
        if (readWrite == 0 || readWrite == 1) // databyte[3] == 0 ==> write
        {
          if (addr == P_MODEL_NUMBER_L)         outTwoBytes = DynamixelGetModelNumber( DYNAMIXEL_ADDRESS );
          else if (addr == P_VERSION)           outTwoBytes = DynamixelGetFirmwareVersion( DYNAMIXEL_ADDRESS );
          else if (addr == P_ID)                outTwoBytes = DynamixelGetMotorID( DYNAMIXEL_ADDRESS );
          else if (addr == P_BAUD_RATE)         outTwoBytes = DynamixelGetBaudRate( DYNAMIXEL_ADDRESS );
          else if (addr == P_RETURN_DELAY_TIME) outTwoBytes = DynamixelGetReturnDelayTime( DYNAMIXEL_ADDRESS );
          else if (addr == P_CW_ANGLE_LIMIT_L)  outTwoBytes = DynamixelGetCWAngleLimit( DYNAMIXEL_ADDRESS );
          else if (addr == P_CCW_ANGLE_LIMIT_L) outTwoBytes = DynamixelGetCCWAngleLimit( DYNAMIXEL_ADDRESS );
          else if (addr == P_LIMIT_TEMPERATURE) outTwoBytes = DynamixelGetHighestLimitTemperature( DYNAMIXEL_ADDRESS );
          else if (addr == P_DOWN_LIMIT_VOLTAGE)outTwoBytes = DynamixelGetLowestLimitVoltage( DYNAMIXEL_ADDRESS );
          else if (addr == P_UP_LIMIT_VOLTAGE)  outTwoBytes = DynamixelGetHighestLimitVoltage( DYNAMIXEL_ADDRESS );
          else if (addr == P_MAX_TORQUE_L)      outTwoBytes = DynamixelGetMaxTorque( DYNAMIXEL_ADDRESS );
          else if (addr == P_RETURN_LEVEL)      outTwoBytes = DynamixelGetStatusReturnLevel( DYNAMIXEL_ADDRESS );
          else if (addr == P_ALARM_LED)         outTwoBytes = DynamixelGetAlarmLED( DYNAMIXEL_ADDRESS );
          else if (addr == P_ALARM_SHUTDOWN)    outTwoBytes = DynamixelGetAlarmShutdown( DYNAMIXEL_ADDRESS );
          else if (addr == P_TORQUE_ENABLE)     outTwoBytes = DynamixelGetTorqueEnable( DYNAMIXEL_ADDRESS );
          else if (addr == P_LED)               outTwoBytes = DynamixelGetLED( DYNAMIXEL_ADDRESS );
          else if (addr == P_CW_COMPL_MARGIN)   outTwoBytes = DynamixelGetCWComplianceMargin( DYNAMIXEL_ADDRESS );
          else if (addr == P_CCW_COMPL_MARGIN)  outTwoBytes = DynamixelGetCCWComplianceMargin( DYNAMIXEL_ADDRESS );
          else if (addr == P_CW_COMPL_SLOPE)    outTwoBytes = DynamixelGetCWComplianceSlope( DYNAMIXEL_ADDRESS );
          else if (addr == P_CCW_COMPL_SLOPE)   outTwoBytes = DynamixelGetCCWComplianceSlope( DYNAMIXEL_ADDRESS );
          else if (addr == P_GOAL_POSITION_L)   outTwoBytes = DynamixelGetGoalPosition( DYNAMIXEL_ADDRESS );
          else if (addr == P_GOAL_SPEED_L)      outTwoBytes = DynamixelGetGoalSpeed( DYNAMIXEL_ADDRESS );
          else if (addr == P_TORQUE_LIMIT_L)    outTwoBytes = DynamixelGetTorqueLimit( DYNAMIXEL_ADDRESS );
          else if (addr == P_PRESENT_POSITION_L)outTwoBytes = DynamixelGetMotorPosition( DYNAMIXEL_ADDRESS );
          else if (addr == P_PRESENT_SPEED_L)   outTwoBytes = DynamixelGetSpeed( DYNAMIXEL_ADDRESS );
          else if (addr == P_PRESENT_LOAD_L)    outTwoBytes = DynamixelGetLoad( DYNAMIXEL_ADDRESS );
          else if (addr == P_PRESENT_VOLTAGE)   outTwoBytes = DynamixelGetVoltage( DYNAMIXEL_ADDRESS );
          else if (addr == P_PRESENT_TEMPERATURE) outTwoBytes = DynamixelGetTemperature( DYNAMIXEL_ADDRESS );
          else if (addr == P_PUNCH_L)           outTwoBytes = DynamixelGetPunch( DYNAMIXEL_ADDRESS );
        }
                
        // reply
        sendMessageToBase('x', addr, msb(outTwoBytes), lsb(outTwoBytes), 0);
      }
      
      
      
      
      // START ALIGNMENT SESSION                                    '*' = 0x2A
      else if (messageQueue[messageQueueStart].command == '*') 
      {
        // initialize motorPos and targetPos for this alignment session
        targetPos = DynamixelGetMotorPosition(DYNAMIXEL_ADDRESS);
        DynamixelSetComplianceSlopes (DYNAMIXEL_ADDRESS, 0x20); 
        DynamixelSetComplianceMargins (DYNAMIXEL_ADDRESS, 0x01);
        DynamixelSetPunch(DYNAMIXEL_ADDRESS, 0x10);
        
        TxRS232_String("start alignment with pos: 0x");
        TxRS232_Hex2Byte(targetPos);
        TxRS232_String("\r\n");
        
        amIinCalibrationMode = true;
        
        // reply
        sendMessageToBase('\'', msb(targetPos), lsb(targetPos), 0, 0);
      }
      
      
          
      // INCREMENT POSITION                                         '+' = 0x2B
      else if (messageQueue[messageQueueStart].command == '+')
      {
        TxRS232_String("Incrementing Target Position: 0x"); 
        TxRS232_Hex2Byte(targetPos); 
        TxRS232_String(" --> 0x");
        targetPos = (targetPos + messageQueue[messageQueueStart].databyte[0]) % 0x3ff; // 0x3ff = 1023
        DynamixelSetGoalPositionWithRetries(DYNAMIXEL_ADDRESS, targetPos);
        TxRS232_Hex2Byte(targetPos); 
        TxRS232_String("\r\n");
        
        _delay_ms(1000);
        
        DynamixelGetMomentaryPosition(0);
        
        TxRS232_String("endingpos: 0x");
        TxRS232_Hex2Byte(targetPos);
        TxRS232_String(", actual: 0x");
        TxRS232_Hex2Byte(momentaryPos);
        TxRS232_String("\r\n");
        
        // reply
        sendMessageToBase(',', msb(targetPos), lsb(targetPos), msb(momentaryPos), lsb(momentaryPos));
      }
      
      
      
      // DECREMENT POSITION                                         '-' = 0x2D
      else if (messageQueue[messageQueueStart].command == '-')
      {
        TxRS232_String("Decrementing Target Position: 0x");
        TxRS232_Hex2Byte(targetPos);
        TxRS232_String(" --> 0x");
        targetPos = (targetPos - messageQueue[messageQueueStart].databyte[0]) % 0x3ff; // 0x3ff = 1023
        DynamixelSetGoalPositionWithRetries(DYNAMIXEL_ADDRESS, targetPos);
        TxRS232_Hex2Byte(targetPos);
        TxRS232_String("\r\n"); 
        
        _delay_ms(1000);
        
        DynamixelGetMomentaryPosition(0);
        
        TxRS232_String("endingpos: 0x");
        TxRS232_Hex2Byte(targetPos);
        TxRS232_String(", actual: 0x");
        TxRS232_Hex2Byte(momentaryPos);
        TxRS232_String("\r\n");
        
        // reply
        sendMessageToBase(',', msb(targetPos), lsb(targetPos), msb(momentaryPos), lsb(momentaryPos));
      }
      
      
      
      // STORE INCRMENT ENCODER COUNT                                '=' = 0x3D
      else if (messageQueue[messageQueueStart].command == '=')      
      {
        uint8_t posId = messageQueue[messageQueueStart].databyte[0];
        
        if (posId > 3)
        {
          TxRS232_String("pos parameter must be between 0 and 3, not 0x");
          TxRS232_Hex2Byte(posId);
          TxRS232_String("\r\n");
          
          // respond with error
          sendMessageToBase(',', 0xFF, 0xFF, 0xFF, 0xFF);
        }
        else
        {
          DynamixelGetMomentaryPosition(0);
  
          EEPROM_setStoredCalibPosition(posId, momentaryPos);
          EEPROM_setStored90degIncrement(posId);
  
          TxRS232_String("Stored Encoder Count 0x");
          TxRS232_Hex2Byte(momentaryPos);
          TxRS232_String(" for increment ");
          TxRS232_Dec(posId);
          TxRS232_String("\r\n");
  
          sendMessageToBase(',', msb(momentaryPos), lsb(momentaryPos), msb(momentaryPos), lsb(momentaryPos));
        }
      }
      
      
      
      // END ALIGNMENT SESSION                                       '.' = 0x2E
      else if (messageQueue[messageQueueStart].command == '.')
      {
        EEPROM_setStoredOperatingMode(MODE_ACTUATED_CUBE);
        DynamixelInitialize();
        
        amIinCalibrationMode = false;
        previousRotationGotStuck = false; // reset that flag, calibration might have solved the problem
        
        // respond
        DynamixelGetMomentaryPosition(0);
        sendMessageToBase('/', msb(momentaryPos), lsb(momentaryPos), 0, 0);
      }
      
      
      
      // clear message from queue and move anyone else who is in 
      // the queue to the front. this is written to be interrupt-proof
      // i.e. if an interrupt happens throughout this loop, it's ok
      incrementMessageQueueStart();
      
    } // end of if (! messageQueueEmpty())
  } // end of while (1)
} // end of main ()