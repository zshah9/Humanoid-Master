//LX16A Library created by Shehab Attia - shehabattia96@gmail.com - April 2018
//Based on Arduino code by LewanSoul
#ifndef LX16A_h
#define LX16A_h

#include "Arduino.h"

class LX16A
{
  public:
    void LobotSerialServoMove(uint8_t id, int16_t position, uint16_t time);
    void LobotSerialServoSetID(uint8_t oldID, uint8_t newID);
    void LobotSerialServoCtrlLED(uint8_t id, int onoff);
    int LobotSerialServoReadVin(uint8_t id);
    int LobotSerialServoReadPosition(uint8_t id);
    void LobotSerialServoUnload( uint8_t id);
    void LXLobotSerialServoLoad( uint8_t id);
    void LobotSerialServoSetMode( uint8_t id, uint8_t Mode, int16_t Speed);
    void LobotSerialServoStopMove( uint8_t id);
    void setSerial(Stream *streamObject)
    {
      _SerialPort=streamObject;
    }
    
  private:
    byte LobotCheckSum(byte buf[]);
    int LobotSerialServoReceiveHandle( byte *ret);
    
    Stream *_SerialPort;
};

#endif
