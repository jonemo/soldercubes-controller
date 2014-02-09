/*
 * rotate.c
 *
 * Created: 8/13/2013 2:52:52 PM
 *  Author: Jonas
 */ 

void rotate (uint16_t endingpos, uint8_t speedParam, uint8_t direction)
{
  uint16_t speed, absSpeed, initPos, iterationCounter;
  uint32_t initTime, partTwoInitTime;
  bool waitingFor0x0000 = false, waitingFor0x03FF = false;
  
  // don't even attempt to rotate if a previous rotation failed
  // and the corresponding flag hasn't been cleared yet
  if (previousRotationGotStuck)
  {
    TxRS232_String("[");
    TxRS232_Hex2Byte(myAddress);
    TxRS232_String("] Not moving because previousRotationGotStuck flag is set.");
    return;
  }
  
  // compute the absolute speed, it's 4 times the parameter (so that we can use one
  // byte to encode the speed, but still cover the full range the dynamixel allows
  // us to use
  absSpeed = speedParam*4;
  
  // if the speed is below the minimal speed or above the maximal speed,
  // we use the default value
  if (absSpeed == 0) absSpeed = DYNAMIXEL_DEFAULT_SPEED;
  else if (absSpeed > DYNAMIXEL_MAX_SPEED) absSpeed = DYNAMIXEL_MAX_SPEED;
  else if (absSpeed < DYNAMIXEL_MIN_SPEED) absSpeed = DYNAMIXEL_MIN_SPEED;
  
  // make a space for recording the position history into queues
  int32_t timeHistory[256];
  int32_t posHistory[256];
  byte currentPosPointer = 255; // s
  
  // switch on the indicator LED to red to show that we are in the coarse
  // part of the turn
  SetStatusLEDs(1, 0);
  
  // initialize buffers and other bookkeeping vals
  DynamixelGetMomentaryPosition(0);
  initPos = momentaryPos; // remember: momentaryPos is a volatile "global" variable
  initTime = time;
  iterationCounter = 0;
  
  // Set the target position now, so we dont have to do it later
  // speed = 0 means maximal speed, so we set it to 1
  uint8_t errCnt = 0;
  DynamixelSetGoalSpeedWithRetries(DYNAMIXEL_ADDRESS, 1);
  DynamixelSetGoalPositionWithRetries(DYNAMIXEL_ADDRESS, endingpos);
  
  // start on continuous rotation in the correct direction
  speed = (direction == DIRECTION_POSITIVE) ? absSpeed : (0x400 ^ absSpeed);
  DynamixelEnableContinuousRotationWithRetries(DYNAMIXEL_ADDRESS);
  DynamixelSetGoalSpeedWithRetries(DYNAMIXEL_ADDRESS, speed);

  // start the coarse positioning loop, bringing the rotation near the target
  // before handing over to the feedback-ish final positioning
  while ( //time - inittime < 2000 &&         // keep looping before 2s timeout, and
  ( momentaryPos >= 0x03ff ||               // while inside the deadzone "above" 0x03ff
    waitingFor0x0000 ||                     // or while inside the deadzone going from 0x03ff to 0x0000
    momentaryPos <= 0x0000 ||               // or while inside the dead zone "below" 0x0000
    waitingFor0x03FF ||                     // or while inside the deadzone going from 0x0000 to 0x03ff
    abs(endingpos - momentaryPos) > 0x40 )  // or while nowhere near the target position
  )
  {
    // artificially delay things a little in debug mode to limit the amount of data on the
    // RS232 bus.
    if(amIinPotentiometerDebugMode) _delay_ms(50); // debugging

    // read the new motor position
    DynamixelGetMomentaryPosition(0);
    
    // cache current position
    currentPosPointer++;
    timeHistory[currentPosPointer] = time;
    posHistory[currentPosPointer] = momentaryPos;
    
    // if debugging, now is a good time to send the output
    if(amIinPotentiometerDebugMode)
    {
      TxRS232_Hex2Byte(momentaryPos);
      if (waitingFor0x0000 || waitingFor0x03FF) TxRS232_String(".");
      TxRS232_String("\r\n");
    }
    
    // The following section is for dealing with the dead zone
    if ( iterationCounter > 3 )
    {
      // There are two ways to show that we are outside the dead zone:
      // one actually detects leaving the zone, the other triggers if we
      // missed the point when we left but are now outside.
      
      // compute flags for dead zone check
      bool leaving03FFDescending = false,
      leaving0000Ascending = false,
      detectingSmallIncrementsDescending = false,
      detectingSmallIncrementsAscending = false;
      
      // (1) We look for *leaving* the 0x03ff region, so this must hold
      // check the history if we were in the dead zone.
      if ( momentaryPos != 0x03ff &&
      posHistory[currentPosPointer-4] == 0x03ff &&
      posHistory[currentPosPointer-3] == 0x03ff &&
      posHistory[currentPosPointer-2] == 0x03ff &&
      posHistory[currentPosPointer-1] == 0x03ff )
      {
        leaving03FFDescending = true;
      }

      if ( momentaryPos != 0x0000 &&
      posHistory[currentPosPointer-4] == 0x0000 &&
      posHistory[currentPosPointer-3] == 0x0000 &&
      posHistory[currentPosPointer-2] == 0x0000 &&
      posHistory[currentPosPointer-1] == 0x0000 )
      {
        leaving0000Ascending = true;
      }
      
      // (2) If we left the dead zone in decreasing direction, we should detect
      // at least two small-ish steps downwards. Check if we can see those.
      if ( posHistory[currentPosPointer-1] - momentaryPos > 0 &&
      posHistory[currentPosPointer-1] - momentaryPos < 0x0F &&
      posHistory[currentPosPointer-2] - posHistory[currentPosPointer-1] > 0 &&
      posHistory[currentPosPointer-2] - posHistory[currentPosPointer-1] < 0x0F &&
      posHistory[currentPosPointer-3] - posHistory[currentPosPointer-2] > 0 &&
      posHistory[currentPosPointer-3] - posHistory[currentPosPointer-2] < 0x0F &&
      posHistory[currentPosPointer-4] - posHistory[currentPosPointer-3] > 0 &&
      posHistory[currentPosPointer-4] - posHistory[currentPosPointer-3] < 0x0F )
      {
        detectingSmallIncrementsDescending = true;
      }
      
      if ( momentaryPos - posHistory[currentPosPointer-1] > 0 &&
      momentaryPos - posHistory[currentPosPointer-1] < 0x0F &&
      posHistory[currentPosPointer-1] - posHistory[currentPosPointer-2] > 0 &&
      posHistory[currentPosPointer-1] - posHistory[currentPosPointer-2] < 0x0F &&
      posHistory[currentPosPointer-2] - posHistory[currentPosPointer-3] > 0 &&
      posHistory[currentPosPointer-2] - posHistory[currentPosPointer-3] < 0x0F &&
      posHistory[currentPosPointer-3] - posHistory[currentPosPointer-4] > 0 &&
      posHistory[currentPosPointer-3] - posHistory[currentPosPointer-4] < 0x0F )
      {
        detectingSmallIncrementsAscending = true;
      }
      
      // if this happens while we were waiting for 0x03ff, it means
      // that we transitioned from 0x0000 to 0x03ff and are now all
      // the way through the dead zone
      if ( waitingFor0x03FF && ( leaving03FFDescending || detectingSmallIncrementsDescending ) )
      {
        waitingFor0x03FF = false;
        if(amIinPotentiometerDebugMode) TxRS232_String("Not waiting for 0x03FF any more.\r\n");
      }
      // otherwise it means we entered the dead zone and have reached
      // the point where we go from 0x03ff to 0x0000
      else if ( leaving03FFDescending )
      {
        waitingFor0x0000 = true;
        SetStatusLEDs(1, 1);
        if(amIinPotentiometerDebugMode) TxRS232_String("Waiting for 0x0000.\r\n");
      }
      // if this happens while we were waiting for 0x0000, it means
      // that we transitioned from 0x03ff to 0x0000 and are now all
      // the way through the dead zone
      else if ( waitingFor0x0000 && ( leaving0000Ascending || detectingSmallIncrementsAscending ))
      {
        waitingFor0x0000 = false;
        if(amIinPotentiometerDebugMode) TxRS232_String("Not waiting for 0x0000 any more.\r\n");
      }
      // otherwise it means we entered the dead zone and have reached
      // the point where we go from 0x0000 to 0x03ff
      else if ( leaving0000Ascending )
      {
        waitingFor0x03FF = true;
        SetStatusLEDs(1, 1);
        if(amIinPotentiometerDebugMode) TxRS232_String("Waiting for 0x03FF.\r\n");
      }
      // putting the getting stuck test here because we can only detect being stuck
      // when we are not in the dead zone
      else if ( iterationCounter > 3 &&
                momentaryPos != 0x0000 &&
                momentaryPos != 0x03FF &&
                ! waitingFor0x0000 &&
                ! waitingFor0x03FF &&
                posHistory[currentPosPointer-4] == posHistory[currentPosPointer-3] &&
                posHistory[currentPosPointer-3] == posHistory[currentPosPointer-2] &&
                posHistory[currentPosPointer-2] == posHistory[currentPosPointer-1] &&
                posHistory[currentPosPointer-1] == momentaryPos &&
                time - initTime > 500 &&
                abs(initPos-momentaryPos) < 10 )
      {
        previousRotationGotStuck = true;
        TxRS232_String("[0x");
        TxRS232_Hex2Byte(myAddress);
        TxRS232_String("] Got stuck at ");
        TxRS232_Hex2Byte(momentaryPos);
        TxRS232_String(" initPos: ");
        TxRS232_Hex2Byte(initPos);
        TxRS232_String("\r\n");
        break;
      }
    }

    iterationCounter++;
  }

  SetStatusLEDs(0, 1);

  partTwoInitTime = time;

  // do our own "feedback-ish" approach to the target
  while ( abs(endingpos - momentaryPos) > DYNAMIXEL_DEAD_BAND && time - partTwoInitTime < 2000 && ! previousRotationGotStuck)
  {
    int percentageOfMaxSpeed = 20 + 80 * abs(endingpos - momentaryPos) / 30;
    // sometimes the values coming in as momentary pos are just not useful
    if (percentageOfMaxSpeed > 100) percentageOfMaxSpeed = 100;

    
    uint16_t newAbsSpeed = absSpeed * percentageOfMaxSpeed / 100;
    if (newAbsSpeed < DYNAMIXEL_MIN_SPEED) newAbsSpeed = DYNAMIXEL_MIN_SPEED;
    
    if (endingpos > momentaryPos)
    speed = newAbsSpeed;
    else
    speed = (0x400 ^ newAbsSpeed);

    // send the speed we calculated to the Dynamixel
    DynamixelSetGoalSpeedWithRetries(DYNAMIXEL_ADDRESS, speed);
    
    // read the new momentary pos to be used in the next loop iteration
    DynamixelGetMomentaryPosition(0);

    // debug output
    if(amIinPotentiometerDebugMode)
    {
      TxRS232_String("t: ");
      TxRS232_Hex2Byte(time);
      TxRS232_String(", m: ");
      TxRS232_Hex2Byte(momentaryPos);
      TxRS232_String(", e: ");
      TxRS232_Hex2Byte(endingpos);
      TxRS232_String(", %: ");
      TxRS232_Hex2Byte(percentageOfMaxSpeed);
      TxRS232_String("\r\n");
    }
  }
  
  // iff previousRotationGotStuck is set, it is the current rotation that got stuck
  // in that case we should reset the target position before switching off continuous
  // rotation or else Dynamixel will still keep trying to get there
  if (previousRotationGotStuck)
  {
    DynamixelSetGoalPositionWithRetries(DYNAMIXEL_ADDRESS, initPos);
  }

  // switch off rotation
  DynamixelDisableContinuousRotationWithRetries(DYNAMIXEL_ADDRESS);
  DynamixelSetGoalSpeedWithRetries(DYNAMIXEL_ADDRESS, DYNAMIXEL_MAX_SPEED);

  // switch off status led, it will go back to heartbeat
  SetStatusLEDs(0, 0);

  // debug output
  if(amIinPotentiometerDebugMode)
  {
    TxRS232_String("t: ");
    TxRS232_Dec(time-initTime);
    TxRS232_String("endingpos: 0x");
    TxRS232_Hex2Byte(endingpos);
    TxRS232_String(", actual: 0x");
    TxRS232_Hex2Byte(momentaryPos);
    TxRS232_String("\r\n");
  }
}