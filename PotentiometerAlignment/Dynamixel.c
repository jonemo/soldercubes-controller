/*
 * Dynamixel.c
 *
 * Created: 5/4/2012 6:27:37 PM
 *  Author: Jonas
 */ 


#include "Dynamixel.h"


void initializeDynamixels( void )
{
	byte i =0;
	do
	{
		_delay_ms(50);
		
		uint16_t vol = DynamixelGetVoltage(i);
		if (vol == 0xffff) {
			TxRS232_Dec(i); TxRS232_String(" timed out.\r\n");
		} else {
			TxRS232_Dec(i); TxRS232_String(" did not time out.\r\n");
			dynamixelAddress = i;
			break;
		}
		
		i++;
	} while (i < 0xff);
	
	//Put Motors in Servo Mode
	setServoMode(dynamixelAddress,1);
	setComplianceSlopes(dynamixelAddress, 128);
}


uint16_t DynamixelGetMotorPosition(byte servoID)
{
	byte bTxPacketLength, bRxPacketLength;
	uint16_t curpos;
	RS485Parameter[0] = P_PRESENT_POSITION_L;
	RS485Parameter[1] = 2; //Read Length
	bTxPacketLength = TxRS485_Packet( servoID, INST_READ, 2 );
	bRxPacketLength = RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] );
	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		curpos = (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
		return curpos;
	}
	return 0xffff;
}
 
void DynamixelSetLED(byte servoID,byte onOff01)
{
	byte bTxPacketLength;
 
	if (onOff01 > 1) onOff01 = 1;
	RS485Parameter[0] = P_LED; //Address of LED
	RS485Parameter[1] = onOff01;// 0 = Off, 1 = On
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,2);
	
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );
}


void DynamixelSetTorque(byte servoID,uint16_t torque)
{
	byte th, tl, bTxPacketLength;

	th = torque >> 8;
	tl = torque & 0xff;

	RS485Parameter[0] = P_TORQUE_LIMIT_L;
	RS485Parameter[1] = tl;
	RS485Parameter[2] = th;
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,3);
	
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );
	
	// now enable toque limit
	TorqueEnable(servoID,1);
	return;
}

void TorqueEnable(byte servoID,byte onoff01)
{
	byte bTxPacketLength;

	RS485Parameter[0] = P_TORQUE_ENABLE;
	RS485Parameter[1] = onoff01; // OnOff
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,2);
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );
	
	return;
} 

void setSpeed(byte servoID,uint16_t speed)
{
	byte th,tl,bTxPacketLength;

	th = speed >> 8;
	tl = speed & 0xff;

	RS485Parameter[0] = P_GOAL_SPEED_L;
	RS485Parameter[1] = tl;
	RS485Parameter[2] = th;
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,3);
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );
	
	return;
} 


void DynamixelSetCWAngle(byte servoID,uint16_t CWAngle)
{
	byte th,tl,bTxPacketLength;

	th = CWAngle >> 8;
	tl = CWAngle & 0xff;

	RS485Parameter[0] = P_CW_ANGLE_LIMIT_L;
	RS485Parameter[1] = tl;
	RS485Parameter[2] = th;
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,3);
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );
  
	_delay_ms(1000);
	return;
} 


void DynamixelSetCCWAngle(byte servoID,uint16_t CCWAngle)
{
	byte th, tl, bTxPacketLength;

	th = CCWAngle >> 8;
	tl = CCWAngle & 0xff;

	RS485Parameter[0] = P_CCW_ANGLE_LIMIT_L;
	RS485Parameter[1] = tl;
	RS485Parameter[2] = th;
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,3);
  
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );
} 


void DynamixelSetGoalPosition(byte servoID,uint16_t GoalPosition)
{
	byte th,tl,bTxPacketLength;

	th = GoalPosition >> 8;
	tl = GoalPosition & 0xff;

	RS485Parameter[0] = P_GOAL_POSITION_L;
	RS485Parameter[1] = tl;
	RS485Parameter[2] = th;
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,3);

	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );  

	return;
} 

void setComplianceSlopes(byte servoID,uint16_t slope)
{
	byte bTxPacketLength;

	RS485Parameter[0] = P_GOAL_POSITION_L;
	RS485Parameter[1] = slope;
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,2);

	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );  

	return;
} 


uint16_t readCCWAngle(byte servoID)
{
	byte bTxPacketLength,bRxPacketLength;
	uint16_t curpos;
	RS485Parameter[0] = P_CCW_ANGLE_LIMIT_L;
	RS485Parameter[1] = 2; //Read Length
	bTxPacketLength = TxRS485_Packet(servoID,INST_READ,2);
	bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE+RS485Parameter[1]);
	
	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		curpos = (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
		return curpos;
	}
	return 0xffff;
} 

uint16_t readCWAngle(byte servoID)
{
	byte bTxPacketLength,bRxPacketLength;
	uint16_t curpos;
	RS485Parameter[0] = P_CW_ANGLE_LIMIT_L;
	RS485Parameter[1] = 2; //Read Length
	bTxPacketLength = TxRS485_Packet(servoID,INST_READ,2);
	bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE+RS485Parameter[1]);
	
	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		curpos = (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
		return curpos;
	}
	return 0xffff;
} 


void setAngle(byte servoID,uint16_t CWAngle, uint16_t CCWAngle)
{
	DynamixelSetCWAngle(servoID,CWAngle);
	DynamixelSetCCWAngle(servoID,CCWAngle);
}


 
 uint16_t DynamixelGetVoltage(byte servoID)
{
	byte bTxPacketLength,bRxPacketLength;
	uint16_t curvol;
	RS485Parameter[0] = P_PRESENT_VOLTAGE;
	RS485Parameter[1] = 1; //Read Length
	bTxPacketLength = TxRS485_Packet(servoID,INST_READ,2);
	bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE+RS485Parameter[1]);
	
	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		curvol = (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
		return curvol;
	}
	return 0xffff;
} 


uint16_t GetMotorGoal(byte servoID)
{
	byte bTxPacketLength,bRxPacketLength;
	uint16_t curpos;
	RS485Parameter[0] = P_GOAL_POSITION_L;
	RS485Parameter[1] = 2; //Read Length
	bTxPacketLength = TxRS485_Packet(servoID,INST_READ,2);
	bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE+RS485Parameter[1]);
	if(bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE+RS485Parameter[1])
	{
		curpos = (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
		return curpos;
	}
	return 0xffff;
} 


void MoveTo(byte servoID,uint16_t pos, uint16_t speed)
{

	byte ph,pl,sh,sl,bTxPacketLength;

	ph = pos >> 8;
	pl = pos & 0xff;
	sh = speed >> 8;
	sl = speed & 0xff;
 
	RS485Parameter[0] = P_GOAL_POSITION_L; 
	RS485Parameter[1] = pl; //Writing Data P_GOAL_POSITION_L
	RS485Parameter[2] = ph; //Writing Data P_GOAL_POSITION_H
	RS485Parameter[3] = sl; //Writing Data P_GOAL_SPEED_L
	RS485Parameter[4] = sh; //Writing Data P_GOAL_SPEED_H
	bTxPacketLength = TxRS485_Packet(servoID,INST_WRITE,5);
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );
} 

uint16_t  getTorque(byte servoID)
{
  byte bTxPacketLength,bRxPacketLength;
  uint16_t curpos;

  RS485Parameter[0] = P_PRESENT_LOAD_L;
  RS485Parameter[1] = 2; //Read Length
  bTxPacketLength = TxRS485_Packet(servoID,INST_READ,2);
  bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE+RS485Parameter[1]);

	if(bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE+RS485Parameter[1])
	{
		curpos = (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
		return curpos;
	}
  return -1;
}



int Moving(byte servoID)
{
	byte bTxPacketLength,bRxPacketLength;
	uint16_t curpos;

	RS485Parameter[0] = P_MOVING;
	RS485Parameter[1] = 2; //Read Length
	bTxPacketLength = TxRS485_Packet(servoID,INST_READ,2);
	bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE+RS485Parameter[1]);

	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		curpos = (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
		return curpos;
	}
	return -1;
}

/*
//Move Continuously until we hit a line.
//Define Servo ID, Sensor Board ID, Sensor to Read from, Speed, and how many lines to skip.
void ContMotion(byte servoID, byte SensorBoardID, byte Sensor,  uint16_t speed, uint16_t skip)
{
	uint16_t sense;
	uint16_t position;
	DynamixelSetCWAngle(servoID,0);
	while(readCWAngle(servoID) == 65535 || readCCWAngle(servoID) == -1);
	DynamixelSetCCWAngle(servoID,0);
	while(readCCWAngle(servoID) == 65535 || readCCWAngle(servoID) == -1);
	
	
	setSpeed(servoID, speed);
	delay(3);
	sense = GetSensorValue(SensorBoardID, Sensor);//First Value will be wrong
	while(GetSensorValue(SensorBoardID, Sensor) == 65535 || GetSensorValue(SensorBoardID, Sensor) == -1);
	
	
	//Repeat depending on number of lines to skip.
	for (uint16_t i = 0; i <= skip; i++)
	{
		sense = GetSensorValue(SensorBoardID, Sensor);
		TxDString("Looking for Line ");
		TxD32Dec(i + 1);	
		TxDString("\r\n");	
		while(sense == 1 || sense == 65535) //Keep Turning until we hit a line
		{
			sense = GetSensorValue(SensorBoardID, Sensor);
		}
		TxDString("Found Line ");
		TxD32Dec(i + 1);	
		TxDString("\r\n");
		
		//if we are skipping this line, make sure we pass the "zero zone" before starting to look for the next line
		if (i < skip) 
		{
			
			sense = GetSensorValue(SensorBoardID, Sensor);
			RS485RxBufferReadPointer = RS485RxBufferWritePointer = 0; //RS485 RxBuffer Clearing.
			delay(3);
			/*while(sense == 0);
			{
				sense = GetSensorValue(SensorBoardID, Sensor);
				TxD32Dec(sense);
				TxDString("\r\n");
			}* /
		}
	}
	position=DynamixelGetMotorPosition(servoID);
	TxDString("Position ");
	TxD32Dec(position);	
	TxDString("\r\n");	
	setSpeed(servoID, 0); // stop continous rotation
	TorqueEnable(servoID,0);
	DynamixelSetCCWAngle(servoID,position);
	while(readCCWAngle(servoID) == 65535 || readCCWAngle(servoID) == -1);
	TorqueEnable(servoID,1);
	MoveTo(servoID, position, 400);
	waitMoving(servoID);	
}

*/



//mode = 0 -->continuous, mode = 1 -->servo
void setServoMode(byte servoID, int mode)
{
	if (mode == 0)
	{
		DynamixelSetCCWAngle(servoID,0);
	}
	else
	{
		DynamixelSetCCWAngle(servoID,1023);
	}
	
	// delay seems to be necessary
	_delay_ms(50);
	
	uint16_t ccwangle = readCCWAngle(servoID);
	
	while( ccwangle == 0xFFFF) {
		TxRS232_String("while setting servo mode, read ccwangle: "); TxRS232_Dec(ccwangle); TxRS232_String("\r\n");
		_delay_ms(50);
		ccwangle = readCCWAngle( servoID );
	}
}