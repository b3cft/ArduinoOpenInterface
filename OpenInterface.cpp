/*
Copyright (c) 2011 Andy "Bob" Brockhurst

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "OpenInterface.h"
//#define DEBUG_SERIAL

OpenInterface::OpenInterface(HardwareSerial* serialPort)
{
  serial = serialPort;
  sensor[OI_SENSOR_OI_MODE] = OI_MODE_OFF;
  sensor[OI_SENSOR_IR]      = 255;
}

/**
 * Initialise the OpenInterface
 * Create a serial connection at the specified baud rate
 */
void OpenInterface::init(long speed)
{
  serial->begin(speed);
}

/**
 * Should be call in the Arduino loop() function to handle interaction with controller
 */
void OpenInterface::handle()
{
  if (serial->available())
  {
    handleOpCode(serial->read());
  }
}

/**
 * Process OI Op Codes passed by controller
 */
void OpenInterface::handleOpCode(byte oc)
{
#ifdef DEBUG_SERIAL
  switch (oc)
  {
    case ('s'):
        oc = OC_SAFE;
    break;
    case ('d'):
        oc = OC_DIRECT_DRIVE;
    break;
    case ('g'):
        oc = OC_SENSORS;
    break;
  }
#endif
  switch (oc)
  {

     case(OC_LOW_SIDE_DRIVERS):
       callCallbackWithOneByte(controlLsdOutputCallback);
     break;

     case(OC_LEDS):
       callCallbackWithThreeBytes(controlLedsCallback);
     break;

     case(OC_PWM_LOW_SIDE_DRIVERS):
       callCallbackWithThreeBytes(controlLsdPwmCallback);
     break;

     case(OC_DIGITAL_OUTPUTS):
       callCallbackWithOneByte(controlDigitalOutputCallback);
     break;

     case(OC_STREAM):
         stream();
     break;

     case(OC_PAUSE_RESUME_STREAM):
     break;

     case(OC_SEND_IR):
       callCallbackWithOneByte(sendIrCallback);
     break;

     case(OC_SHOW_SCRIPT):
       showScript();
     break;

     case(OC_WAIT_EVENT):
       callCallbackWithOneByte(waitEventCallback);
     break;

     case(OC_PLAY_SCRIPT):
        scriptPlay();
     break;

     case(OC_SCRIPT):
       scriptSet();
     break;

     case(OC_WAIT_ANGLE):
         if (waitAngleCallback)
         {
           waitAction(waitAngleCallback);
         }
     break;

     case(OC_WAIT_DISTANCE):
         if (waitDistanceCallback)
         {
           waitAction(waitDistanceCallback);
         }
     break;

     case(OC_WAIT_TIME):
       waitTime();
     break;

     case(OC_DEMO):
     case(OC_SPOT):
     case(OC_COVER):
     case(OC_COVER_AND_DOCK):
       // These are all demo codes. Implement?
     break;

     case(OC_QUERY_LIST):
       queryList();
     break;

     case (OC_START):
       sensor[OI_SENSOR_OI_MODE] = OI_MODE_PASSIVE;
     break;

     case (OC_BAUD):
     break;

     case (OC_CONTROL):
     case (OC_SAFE):
       sensor[OI_SENSOR_OI_MODE] = OI_MODE_SAFE;
     break;

     case (OC_SENSORS):
       getSensors();
     break;

     case (OC_SONG):
       song();
     break;

     case(OC_PLAY_SONG):
       songPlay();
     break;

     case (OC_DIRECT_DRIVE):
       driveDirect();
     break;

     case (OC_DRIVE):
       drive();
     break;

     case (OC_FULL):
       sensor[OI_SENSOR_OI_MODE] = OI_MODE_FULL;
     break;
  }
}

/**
 * Helper function to read a specified number of bytes from serial interface with a timeout
 */
bool OpenInterface::readBytes(uint8_t* bytesIn, uint8_t count)
{
  while (count-- > 0)
  {
    unsigned long start = millis();
    while(!serial->available())
    {
      if (millis() > start + READ_TIMEOUT)
      {
#ifdef DEBUG_SERIAL
      serial->println("Timeout");
#endif
        return false;
      }
    }
    *bytesIn++ = serial->read();
  }
  return true;
}

/**
 * Helper function to read a specified number of bytes from serial interface with a timeout
 */
bool OpenInterface::readByte(uint8_t* byteIn)
{
  unsigned long start = millis();
  while(!serial->available())
  {
    if (millis() > start + READ_TIMEOUT)
    {
#ifdef DEBUG_SERIAL
    serial->println("Timeout");
#endif
      return false;
    }
  }
  *byteIn = serial->read();
  return true;
}
/**
 * Return true if the passed packet id is a double byte packet
 */
bool OpenInterface::isDoublePacket(uint8_t packet)
{
  uint8_t max, doublePackets[] = {19,20,22,23,25,26,27,28,29,30,31,33,39,40,41,42};
  max = sizeof(doublePackets);
  for(int i=0; i<max; i++)
  {
    if (doublePackets[i]==packet)
    {
      return true;
    }
  }
  return false;
}

/**
 * Return the number of bytes the opcode should read from the serial interface
 */
uint8_t OpenInterface::opCodeDataLen(uint8_t opCode)
{
  switch(opCode)
  {
    case (OC_DIRECT_DRIVE):
    case (OC_DRIVE):
      return 4;
    break;

    case (OC_LEDS):
    case (OC_PWM_LOW_SIDE_DRIVERS):
      return 3;
    break;

    case (OC_WAIT_ANGLE):
    case (OC_WAIT_DISTANCE):
      return 2;
    break;

    case (OC_DEMO):
    case (OC_BAUD):
    case (OC_SPOT):
    case (OC_COVER):
    case (OC_COVER_AND_DOCK):
    case (OC_LOW_SIDE_DRIVERS):
    case (OC_DIGITAL_OUTPUTS):
    case (OC_SEND_IR):
    case (OC_PLAY_SONG):
    case (OC_SENSORS):
    case (OC_PAUSE_RESUME_STREAM):
    case (OC_WAIT_TIME):
    case (OC_WAIT_EVENT):
      return 1;
    break;

    case (OC_SAFE):
    case (OC_START):
    case (OC_CONTROL):
    case (OC_FULL):
    case (OC_PLAY_SCRIPT):
    case (OC_SHOW_SCRIPT):
      return 0;
    break;

    case (OC_SONG):
    case (OC_QUERY_LIST):
    case (OC_STREAM):
    case (OC_SCRIPT):
      return false;
    break;
  }
}

/**
 * ************ Sensor Interactions ****************
 */

/**
 * Helper function to set battery information for sensor return
 */
void OpenInterface::setBatteryInfo(int voltage, int current, int charge, int chargeEstimate, uint8_t temperature)
{
  sensorInt[OI_SENSOR_BAT_VOLTAGE]       = voltage;
  sensorInt[OI_SENSOR_BAT_CURRENT]       = current;
  sensorInt[OI_SENSOR_BAT_CHARGE]        = charge;
  sensorInt[OI_SENSOR_BAT_CHARGE_REMAIN] = chargeEstimate;
  sensor[OI_SENSOR_BAT_TEMPERATURE]      = temperature;
}

/**
 * Helper function to set battery information for sensor return
 */
void OpenInterface::updateBatteryVoltageCurrent(int voltage, int current)
{
  sensorInt[OI_SENSOR_BAT_VOLTAGE] = voltage;
  sensorInt[OI_SENSOR_BAT_CURRENT] = current;
}

/**
 * Helper function to set battery information for sensor return
 */
void OpenInterface::updateBatteryCurrent(int current)
{
  sensorInt[OI_SENSOR_BAT_CURRENT] = current;
}

/**
 * Helper function to set battery information for sensor return
 */
void OpenInterface::updateBatteryChargeEstimate(int chargeEstimate)
{
  sensorInt[OI_SENSOR_BAT_CHARGE_REMAIN] = chargeEstimate;
}

/**
 * Helper function to set battery information for sensor return
 */
void OpenInterface::updateBatteryVoltage(int voltage)
{
  sensorInt[OI_SENSOR_BAT_VOLTAGE] = voltage;
}

/**
 * Helper function to set battery information for sensor return
 */
void OpenInterface::updateBatteryTemperature(uint8_t temperature)
{
  sensor[OI_SENSOR_BAT_TEMPERATURE] = temperature;
}

/**
 * Set a value of a sensor ready for return to the controller
 */
void OpenInterface::setSensorValue(uint8_t sensorKey, uint8_t sensorValue)
{
  sensor[sensorKey] = sensorValue;
}

/**
 * Set a value of a sensor ready for return to the controller
 */
void OpenInterface::setSensorValue(uint8_t sensorKey, int sensorValue)
{
  sensorInt[sensorKey] = sensorValue;
}

/**
 * ************ Handle Op Codes ****************
 */

/**
 * Handle wait callback that accepts a single int, e.g. waitAngle, waitDistance
 */
void OpenInterface::waitAction(callbackWithOneInt action)
{
  uint8_t raw[2];
  bool result = readBytes(raw, 2);
  if (result)
  {
    int distance = int(word(raw[0], raw[1]));
    (*action)(distance);
  }
}

/**
 * Handle wait time Op Code. Units of 15ms
 */
void OpenInterface::waitTime()
{
  uint8_t time;
  bool result = readByte(&time);
  if (result)
  {
    delay(15*time);
  }
}

/**
 * Handle the sensors Op Code. Handles groups and individual sensors
 */
void OpenInterface::getSensors()
{
    uint8_t id[1], pStart, pStop, sensorLen;
    char strId[3];
    readBytes(id, 1);
#ifdef DEBUG_SERIAL
    id[0] = PACKET_GRP_7_42;
#endif
    switch (id[0])
    {
      case (PACKET_GRP_7_26):
        pStart    = 7;
        pStop     = 26;
        sensorLen = 26;
      break;

      case (PACKET_GRP_7_16):
        pStart    = 7;
        pStop     = 16;
        sensorLen = 16;
      break;

      case (PACKET_GRP_17_20):
        pStart    = 17;
        pStop     = 20;
        sensorLen = 6;
      break;

      case (PACKET_GRP_21_26):
        pStart    = 21;
        pStop     = 26;
        sensorLen = 10;
      break;

      case (PACKET_GRP_27_34):
        pStart    = 27;
        pStop     = 34;
        sensorLen = 14;
      break;

      case (PACKET_GRP_35_42):
        pStart    = 35;
        pStop     = 42;
        sensorLen = 12;
      break;

      case(PACKET_GRP_7_42):
        pStart    = 7;
        pStop     = 42;
        sensorLen = 52;
      break;

      default:
        pStart = pStop = id[0];
        sensorLen = (isDoublePacket(id[0])) ? 2 : 1;
      break;
    }
    byte sensorData[sensorLen];
    uint8_t packet=0;
    for (uint8_t p=pStart ; p<=pStop ; p++)
    {
      if(isDoublePacket(p))
      {
#ifdef DEBUG_SERIAL
        serial->print("byte[");serial->print(packet, DEC);serial->print("] packet[");serial->print(p, DEC);serial->print("]: ");serial->println(sensorInt[p], HEX);
#endif
        sensorData[packet++] = highByte(sensorInt[p]);
        sensorData[packet++] = lowByte(sensorInt[p]);
      }
      else
      {
#ifdef DEBUG_SERIAL
        serial->print("byte[");serial->print(packet, DEC);serial->print("] packet[");serial->print(p, DEC);serial->print("]: ");serial->println(sensor[p], HEX);
#endif
        sensorData[packet++] = sensor[p];
      }
    }
    serial->write(sensorData, sensorLen);
}

/**
 * Handle the query list Op Code. Return values of a request list of sensors
 */
void OpenInterface::queryList()
{
  uint8_t details[1];
  bool result = readBytes(details, 1);
  if (result)
  {
    uint8_t sensors[details[0]];
    result = readBytes(sensors, details[0]);
    if (result)
    {
      for (int i=0 ; i<details[0] ; i++)
      {
        if (isDoublePacket(i))
        {
          serial->write(highByte(sensorInt[i]));
          serial->write(lowByte(sensorInt[i]));
        }
        else
        {
          serial->write(sensor[i]);
        }
      }
    }
  }
}

/**
 * Handle calling a callback function that uses three bytes as parameters
 * e.g. leds, pwmLowSideDrivers
 */
void OpenInterface::callCallbackWithThreeBytes(callbackWithThreeBytes callbackFunction)
{
  uint8_t status[3];
  bool result = readBytes(status, 3);
  if (result && callbackFunction)
  {
    (*callbackFunction)(status[0], status[1], status[2]);
  }
}

/**
 * Handle calling a callback function that uses one byte as it's parameter
 * e.g. controlDigitalOutputs, SendIr, waitEvent, controlLsdOutput
 */
void OpenInterface::callCallbackWithOneByte(callbackWithOneByte callbackFunction)
{
  uint8_t status;
  bool result = readByte(&status);
  if (result && callbackFunction)
  {
    (*callbackFunction)(status);
  }
}

/**
 * @todo to be implemented
 */
void OpenInterface::stream()
{

}

/**
 * @todo to be implemented
 */
void OpenInterface::showScript()
{

}

/**
 * Handle recording a script of Op Codes being sent
 */
void OpenInterface::scriptSet()
{
  uint8_t commands;
  bool result = readByte(&commands);
  if (result)
  {
    if (commands == 0x00)
    {
      for (int i=0; i<sizeof(script); i++)
      {
        script[i] = 0x00;
      }
      return;
    }
    script[0] = commands;
    for (int i=1; i<commands; i++)
    {
      uint8_t opCode;
      result = readByte(&opCode);
      if (result)
      {
        script[i++] = opCode;
        uint8_t opLen = opCodeDataLen(opCode);
        for (int j=0; j<opLen; j++)
        {
          uint8_t opData;
          result = readByte(&opData);
          if (result)
          {
            script[i++] = opData;
          }
        }
      }
    }
  }
}

/**
 * Loop through the script array and action the requests
 */
void OpenInterface::scriptPlay()
{
  uint8_t scriptLen = script[0];
  for (int i=1; i<scriptLen; i++)
  {
    switch(script[i])
    {
      case (OC_PLAY_SCRIPT):
          // restart the script. i.e. play for ever!
          i=1;
      break;

      case (OC_PWM_LOW_SIDE_DRIVERS):
        if (controlLsdPwmCallback)
        {
          (*controlLsdPwmCallback)(script[i++], script[i++], script[i++]);
        }
      break;

      case (OC_LEDS):
        if (controlLedsCallback)
        {
          (*controlLedsCallback)(script[i++], script[i++], script[i++]);
        }
      break;

      case (OC_WAIT_ANGLE):
        if (waitAngleCallback)
        {
          (*waitAngleCallback)(int(word(script[i++],script[i++])));
        }
      break;

      case (OC_WAIT_DISTANCE):
        if (waitDistanceCallback)
        {
          (*waitDistanceCallback)(int(word(script[i++],script[i++])));
        }
      break;

      case (OC_LOW_SIDE_DRIVERS):
        if (controlLsdOutputCallback)
        {
          (*controlLsdOutputCallback)(script[i++]);
        }
      break;

      case (OC_DIGITAL_OUTPUTS):
        if (controlDigitalOutputCallback)
        {
          (*controlDigitalOutputCallback)(script[i++]);
        }
      break;

      case (OC_SEND_IR):
        if (sendIrCallback)
        {
          (*sendIrCallback)(script[i++]);
        }
      break;

      case (OC_PLAY_SONG):
        if (songCallback)
        {
          (*songCallback)(songs[script[i++]]);
        }
      break;

      case (OC_WAIT_TIME):
        delay(script[i++]*15);
      break;

      case (OC_WAIT_EVENT):
        if (waitEventCallback)
        {
          (*waitEventCallback)(script[i++]);
        }
      break;
    }
  }
}

/**
 * Handle the song Op Code store the song for playback later.
 */
void OpenInterface::song()
{
  uint8_t details[2];
  bool result = readBytes(details, 2);
  if (result)
  {
    uint8_t notes[details[1]];
    result = readBytes(songs[details[0]], details[1]);
  }
}

/**
 * Handle the song play op code, callback to user function if defined
 */
void OpenInterface::songPlay()
{
  uint8_t details[1];
  bool result = readBytes(details, 1);
  if (result && songCallback)
  {
    (*songCallback)(songs[details[0]]);
  }
}

/**
 * Handle the drive Op Code, callback to user function if defined
 */
void OpenInterface::drive()
{
  uint8_t raw[4];
  bool result = readBytes(raw, 4);

  if (result)
  {
    sensorInt[OI_SENSOR_REQ_VEL]       = int(word(raw[0], raw[1]));
    sensorInt[OI_SENSOR_REQ_RADIUS]    = int(word(raw[2], raw[3]));
    sensorInt[OI_SENSOR_REQ_LEFT_VEL]  = 0;
    sensorInt[OI_SENSOR_REQ_RIGHT_VEL] = 0;
    if (driveCallback)
    {
      (*driveCallback)(sensorInt[OI_SENSOR_REQ_VEL], sensorInt[OI_SENSOR_REQ_RADIUS]);
    }
  }
}

/**
 * Handle the driveDirect op Code, callback to user function if defined
 */
void OpenInterface::driveDirect()
{
  uint8_t raw[4];
  bool result = readBytes(raw, 4);

  if (result)
  {
    sensorInt[OI_SENSOR_REQ_RIGHT_VEL] = int(word(raw[0], raw[1]));
    sensorInt[OI_SENSOR_REQ_LEFT_VEL]  = int(word(raw[2], raw[3]));
    sensorInt[OI_SENSOR_REQ_RADIUS]    = 0;
    sensorInt[OI_SENSOR_REQ_VEL]       = 0;
    if (driveDirectCallback)
    {
      (*driveDirectCallback)(sensorInt[OI_SENSOR_REQ_LEFT_VEL], sensorInt[OI_SENSOR_REQ_RIGHT_VEL]);
    }
  }
}
