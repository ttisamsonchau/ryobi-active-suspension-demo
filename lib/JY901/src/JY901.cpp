#include "JY901.h"
#include "string.h"
#include "Wire.h"

CJY901 ::CJY901 ()
{
	ucDevAddr =0x50;
}
void CJY901::StartIIC(TwoWire * twi)
{
	ucDevAddr = 0x50;
	twi_i2c = twi;
}
void CJY901::StartIIC(TwoWire * twi, unsigned char ucAddr)
{
	ucDevAddr = ucAddr;
	twi_i2c = twi;
}
void CJY901 ::CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) 
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
	}
}
void CJY901::readRegisters(unsigned char deviceAddr,unsigned char addressToRead, unsigned char bytesToRead, char * dest)
{
  twi_i2c -> beginTransmission(deviceAddr);
  twi_i2c -> write(addressToRead);
  twi_i2c -> endTransmission(false); //endTransmission but keep the connection active

  twi_i2c -> requestFrom(deviceAddr, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(twi_i2c -> available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = twi_i2c -> read();    
}
void CJY901::writeRegister(unsigned char deviceAddr,unsigned char addressToWrite,unsigned char bytesToRead, char *dataToWrite)
{
  twi_i2c -> beginTransmission(deviceAddr);
  twi_i2c -> write(addressToWrite);
  for(int i = 0 ; i < bytesToRead ; i++)
  twi_i2c -> write(dataToWrite[i]);
  twi_i2c -> endTransmission(); //Stop transmitting
}

short CJY901::ReadWord(unsigned char ucAddr)
{
	short sResult;
	readRegisters(ucDevAddr, ucAddr, 2, (char *)&sResult);
	return sResult;
}
void CJY901::WriteWord(unsigned char ucAddr,short sData)
{	
	writeRegister(ucDevAddr, ucAddr, 2, (char *)&sData);
}
void CJY901::ReadData(unsigned char ucAddr,unsigned char ucLength,char chrData[])
{
	readRegisters(ucDevAddr, ucAddr, ucLength, chrData);
}

void CJY901::GetTime()
{
	readRegisters(ucDevAddr, 0x30, 8, (char*)&stcTime);	
}
void CJY901::GetAcc()
{
	readRegisters(ucDevAddr, AX, 6, (char *)&stcAcc);
}
void CJY901::GetGyro()
{
	readRegisters(ucDevAddr, GX, 6, (char *)&stcGyro);
}

void CJY901::GetAngle()
{
	readRegisters(ucDevAddr, Roll, 6, (char *)&stcAngle);
}
void CJY901::GetMag()
{
	readRegisters(ucDevAddr, IMU_HX, 6, (char *)&stcMag);
}
void CJY901::GetPress()
{
	readRegisters(ucDevAddr, PressureL, 8, (char *)&stcPress);
}
void CJY901::GetDStatus()
{
	readRegisters(ucDevAddr, D0Status, 8, (char *)&stcDStatus);
}
void CJY901::GetLonLat()
{
	readRegisters(ucDevAddr, LonL, 8, (char *)&stcLonLat);
}
void CJY901::GetGPSV()
{
	readRegisters(ucDevAddr, GPSHeight, 8, (char *)&stcGPSV);
}
