/*
 * heartbeat.c
 *
 * Created: 5/5/2013 4:00:28 PM
 *  Author: Jonas
 */ 

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
    else
    {
      if ( time % HEARTBEAT_EVERY_X_MS <= HEARTBEAT_BLINK_DURATION)
      {
        SetStatusLEDs(0,1); // green
      }
      else
      {
        SetStatusLEDs(0,0);
      }
    }