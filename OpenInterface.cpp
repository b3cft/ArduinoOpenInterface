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

OpenInterface::OpenInterface()
{
  sensor[OI_SENSOR_OI_MODE] = OI_MODE_OFF;
  sensor[OI_SENSOR_IR]      = 255;
}

/**
 * Initialise the OpenInterface
 * Create a serial connection at the specified baud rate
 */
void OpenInterface::init(long speed)
{
  Serial.begin(speed);
}

/**
 * Should be call in the Arduino loop() function to handle interaction with controller
 */
void OpenInterface::handle()
{
  if (Serial.available())
  {
    handleOpCode(Serial.read());
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
     case(OC_LEDS):
     case(OC_PWM_LOW_SIDE_DRIVERS):
     case(OC_DIGITAL_OUTPUTS):
     case(OC_STREAM):
     case(OC_PAUSE_RESUME_STREAM):
     case(OC_SEND_IR):

     case(OC_SCRIPT):
     case(OC_PLAY_SCRIPT):
     case(OC_SHOW_SCRIPT):

     case(OC_WAIT_EVENT):
       // To be implemented
     break;

     case(OC_WAIT_ANGLE):
         waitAngle();
     break;

     case(OC_WAIT_DISTANCE):
       waitDistance();
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
    while(!Serial.available())
    {
      if (millis() > start + READ_TIMEOUT)
      {
#ifdef DEBUG_SERIAL
      Serial.println("Timeout");
#endif
        return false;
      }
    }
    *bytesIn++ = Serial.read();
  }
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
 * ************ Sensor Interactions ****************
 */

void OpenInterface::setBatteryInfo(int voltage, int current, int charge, int chargeEstimate, uint8_t temperature)
{
  sensorInt[OI_SENSOR_BAT_VOLTAGE]       = voltage;
  sensorInt[OI_SENSOR_BAT_CURRENT]       = current;
  sensorInt[OI_SENSOR_BAT_CHARGE]        = charge;
  sensorInt[OI_SENSOR_BAT_CHARGE_REMAIN] = chargeEstimate;
  sensor[OI_SENSOR_BAT_TEMPERATURE]      = temperature;
}
void OpenInterface::updateBatteryVoltageCurrent(int voltage, int current)
{
  sensorInt[OI_SENSOR_BAT_VOLTAGE] = voltage;
  sensorInt[OI_SENSOR_BAT_CURRENT] = current;
}
void OpenInterface::updateBatteryCurrent(int current)
{
  sensorInt[OI_SENSOR_BAT_CURRENT] = current;
}
void OpenInterface::updateBatteryChargeEstimate(int chargeEstimate)
{
  sensorInt[OI_SENSOR_BAT_CHARGE_REMAIN] = chargeEstimate;
}
void OpenInterface::updateBatteryVoltage(int voltage)
{
  sensorInt[OI_SENSOR_BAT_VOLTAGE] = voltage;
}
void OpenInterface::updateBatteryTemperature(uint8_t temperature)
{
  sensor[OI_SENSOR_BAT_TEMPERATURE] = temperature;
}

void OpenInterface::setSensorValue(uint8_t sensorKey, uint8_t sensorValue)
{
  sensor[sensorKey] = sensorValue;
}

void OpenInterface::setSensorValue(uint8_t sensorKey, int sensorValue)
{
  sensorInt[sensorKey] = sensorValue;
}

/**
 * ************ Handle Op Codes ****************
 */

/**
 * Handle wait angle Op Code
 * @todo refactor this and wait distance into a single function
 */
void OpenInterface::waitAngle()
{
  uint8_t raw[2];
  bool result = readBytes(raw, 2);
  if (result)
  {
    int distance = int(word(raw[0], raw[1]));
    if (waitAngleCallback)
    {
      (*waitAngleCallback)(distance);
    }
  }
}

/**
 * Handle wait distance Op Code
 * @todo refactor this and wait angle into a single function
 */
void OpenInterface::waitDistance()
{
  uint8_t raw[2];
  bool result = readBytes(raw, 2);
  if (result)
  {
    int distance = int(word(raw[0], raw[1]));
    if (waitDistanceCallback)
    {
      (*waitDistanceCallback)(distance);
    }
  }
}

/**
 * Handle wait time Op Code. Units of 15ms
 */
void OpenInterface::waitTime()
{
  uint8_t time[1];
  bool result = readBytes(time, 1);
  if (result)
  {
    delay(15*time[0]);
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
        Serial.print("byte[");Serial.print(packet, DEC);Serial.print("] packet[");Serial.print(p, DEC);Serial.print("]: ");Serial.println(sensorInt[p], HEX);
#endif
        sensorData[packet++] = highByte(sensorInt[p]);
        sensorData[packet++] = lowByte(sensorInt[p]);
      }
      else
      {
#ifdef DEBUG_SERIAL
        Serial.print("byte[");Serial.print(packet, DEC);Serial.print("] packet[");Serial.print(p, DEC);Serial.print("]: ");Serial.println(sensor[p], HEX);
#endif
        sensorData[packet++] = sensor[p];
      }
    }
    Serial.write(sensorData, sensorLen);
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
          Serial.write(highByte(sensorInt[i]));
          Serial.write(lowByte(sensorInt[i]));
        }
        else
        {
          Serial.write(sensor[i]);
        }
      }
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
