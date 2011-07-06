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
     case (OC_START):
       OIMode = 1; // passive
     break;

     case (OC_BAUD):
     break;

     case (OC_CONTROL):
     case (OC_SAFE):
       OIMode = 2; //safe
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
       OIMode = 3; // full
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
 * Return the sensor packet value
 */
uint8_t OpenInterface::getPacketId(uint8_t id, uint8_t num=1)
{
  /**
   * @todo Move sensors to definitions
   */
  switch(id)
  {
    case(17): // IR Sensor 255 = no signal
      return 255;
    break;

    case(22): // Battery voltage in mV  2 bytes
      if (num==1) return (batteryVoltage & 0xff00) >> 8;
      if (num==2) return (batteryVoltage & 0xff);
    break;

    case(23): // battery current in/out in mA 2 bytes
      if (num==1) return (batteryCurrent & 0xff00) >> 8;
      if (num==2) return (batteryCurrent & 0xff);
    break;

    case(24): // Battery temp in C
      return batteryTemperature;
    break;

    case(25): // Battery charge in mAh 2 bytes
      if (num==1) return (batteryCharge & 0xff00) >> 8;
      if (num==2) return (batteryCharge & 0xff);
    break;

    case(26): // Battery charge estimated in mAh  2 bytes
      if (num==1) return (batteryChargeEstimate & 0xff00) >> 8;
      if (num==2) return (batteryChargeEstimate & 0xff);
    break;

    case(35): // OI Mode
      return OIMode;
    break;

    case(39): // requested velocity in mm/s -500 / 500. 2 bytes
      if (num==1) return (reqVelocity & 0xff00) >> 8;
      if (num==2) return (reqVelocity & 0xff);
    break;

    case(40): // Requested radius 2 bytes
      if (num==1) return (reqRadius & 0xff00) >> 8;
      if (num==2) return (reqRadius & 0xff);
    break;

    case(41): // Right requested velocity mm/s -500/500. 2 bytes
      if (num==1) return (reqRightVelocity & 0xff00) >> 8;
      if (num==2) return (reqRightVelocity & 0xff);
    break;

    case(42): // Left requested velocity mm/s -500/500. 2 bytes
      if (num==1) return (reqLeftVelocity & 0xff00) >> 8;
      if (num==2) return (reqLeftVelocity & 0xff);
    break;

    /*
    These are sensors yet to be implemented
    case(7):  // B 000, caster, l wheeldrop, r wheeldrop, bump l, bump r
    case(8):  // wall sensor
    case(9):  // left cliff
    case(10): // left front cliff
    case(11): // right front cliff
    case(12): // right cliff
    case(13): // virtual wall
    case(14): // B 000. l wheel overcurrent, r wheel oc, LD2, LD1, LD0
    case(18): // B 00000, advance, 0, play
    case(19): // distance in mm travelled since last update. neg for backwards. Sum both wheels/2
    case(19): // ^^ two bytes
    case(20): // angle in degrees turned since last update, CCW positive, Clockwise neg. 2 bytes
    case(21): // charging state
    case(27): // Wall sensor stength. 2 bytes
    case(28): // Cliff left stength. 2 bytes
    case(29): // Cliff left front stength. 2 bytes
    case(30): // Cliff right front stength. 2 bytes
    case(31): // Cliff right stength. 2 bytes
    case(32): // 000 device detect, DIO3, DIO2, DIO1, DIO0
    case(33): // analog input 0-1023. 2 bytes
    case(34): // charging sources
    case(36): // current song
    case(37): // song playing
    case(38): // number of data stream packets
    case(39): // requested velocity in mm/s -500 / 500. 2 bytes
    case(40): // Requested radius 2 bytes
    case(41): // Right requested velocity mm/s -500/500. 2 bytes
    case(42): // Left requested velocity mm/s -500/500. 2 bytes
    */
    default:
      return 0;
    break;
  }
}

/**
 * Helper function to return whether one int is contained within an array of ints
 */
bool OpenInterface::in_array(uint8_t needle, uint8_t* haystack, uint8_t max)
{
    if (max==0) return false;

    for(int i=0; i<max; i++)
        if (haystack[i]==needle)
            return true;
    return false;
}

/**
 * ************ Sensor Interactions ****************
 */

void OpenInterface::setBatteryInfo(int voltage, int current, int charge, int chargeEstimate, uint8_t temperature)
{
  batteryVoltage        = voltage;
  batteryCurrent        = current;
  batteryCharge         = charge;
  batteryChargeEstimate = chargeEstimate;
  batteryTemperature    = temperature;
}
void OpenInterface::updateBatteryVoltageCurrent(int voltage, int current)
{
  batteryVoltage = voltage;
  batteryCurrent = current;
}
void OpenInterface::updateBatteryCurrent(int current)
{
  batteryCurrent = current;
}
void OpenInterface::updateBatteryChargeEstimate(int chargeEstimate)
{
  batteryChargeEstimate = chargeEstimate;
}
void OpenInterface::updateBatteryVoltage(int voltage)
{
  batteryVoltage = voltage;
}
void OpenInterface::updateBatteryTemperature(uint8_t temperature)
{
  batteryTemperature = temperature;
}

/**
 * ************ Handle Op Codes ****************
 */

/**
 * Handle the sensors Op Code. Handles groups and individual sensors
 */
void OpenInterface::getSensors()
{
    uint8_t doublePackets[] = {19,20,22,23,25,26,27,28,29,30,31,33,39,40,41,42};
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
        sensorLen = (in_array(id[0], doublePackets, sizeof(doublePackets)/sizeof(uint8_t))) ? 2 : 1;
      break;
    }
    byte sensor[sensorLen];
    uint8_t packet=0;
    for (uint8_t p=pStart ; p<=pStop ; p++)
    {
#ifdef DEBUG_SERIAL
      Serial.print("byte[");Serial.print(packet, DEC);Serial.print("] packet[");Serial.print(p, DEC);Serial.print("]: ");Serial.println(getPacketId(p), DEC);
#endif
      sensor[packet++] = getPacketId(p);
      if(in_array(p, doublePackets, sizeof(doublePackets)/sizeof(uint8_t)))
      {
#ifdef DEBUG_SERIAL
        Serial.print("byte[");Serial.print(packet, DEC);Serial.print("] packet[");Serial.print(p, DEC);Serial.print("]: ");Serial.println(getPacketId(p, 2), DEC);
#endif
        sensor[packet++] = getPacketId(p, 2);
      }
    }
    Serial.write(sensor, sensorLen);
}

/**
 * Handle the song Op Code, callback to user function if defined
 */
void OpenInterface::song()
{
    uint8_t details[2];
    bool result = readBytes(details, 2);
    if (result)
    {
      uint8_t notes[details[1]];
      result = readBytes(notes, details[1]);
      if (result)
      {
        // play song notes here.
      }
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
    reqVelocity      = int(word(raw[0], raw[1]));
    reqRadius        = int(word(raw[2], raw[3]));
    reqLeftVelocity  = 0;
    reqRightVelocity = 0;
    if (driveCallback)
    {
      (*driveCallback)(reqVelocity, reqRadius);
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
    reqRightVelocity = int(word(raw[0], raw[1]));
    reqLeftVelocity  = int(word(raw[2], raw[3]));
    reqVelocity      = 0;
    reqRadius        = 0;
    if (driveDirectCallback)
    {
      (*driveDirectCallback)(reqLeftVelocity, reqRightVelocity);
    }
  }
}
