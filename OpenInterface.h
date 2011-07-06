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
#ifndef OpenInterface_h
#define OpenInterface_h

//OP Codes
#define OC_START                0x80 //128
#define OC_BAUD                 0x81 //129
#define OC_CONTROL              0x82 //130
#define OC_SAFE                 0x83 //131
#define OC_FULL                 0x84 //132
#define OC_POWER                0x85 //133
#define OC_SPOT                 0x86 //134
#define OC_CLEAN                0x87 //135
#define OC_MAX                  0x88 //136
#define OC_DRIVE                0x89 //137
#define OC_LOW_SIDE_DRIVERS     0x8a //138
#define OC_LEDS                 0x8b //139
#define OC_SONG                 0x8c //140
#define OC_PLAY_SONG            0x8d //141
#define OC_SENSORS              0x8e //142
#define OC_FORCE_SEEKING_DOCK   0x8f //143
#define OC_PWM_LOW_SIDE_DRIVERS 0x90 //144
#define OC_DIRECT_DRIVE         0x91 //145
#define OC_DIGITAL_OUTPUTS      0x93 //147
#define OC_STREAM               0x94 //148
#define OC_QUERY_LIST           0x95 //149
#define OC_PAUSE_RESUME_STREAM  0x96 //150
#define OC_SEND_IR              0x97 //151
#define OC_SCRIPT               0x98 //152
#define OC_PLAY_SCRIPT          0x99 //153
#define OC_SHOW_SCRIPT          0x9a //154
#define OC_WAIT_TIME            0x9b //155
#define OC_WAIT_DISTANCE        0x9c //156
#define OC_WAIT_ANGLE           0x9d //157
#define OC_WAIT_EVENT           0x9e //158

// Sensor Packet Groups
#define PACKET_GRP_7_26   0
#define PACKET_GRP_7_16   1
#define PACKET_GRP_17_20  2
#define PACKET_GRP_21_26  3
#define PACKET_GRP_27_34  4
#define PACKET_GRP_35_42  5
#define PACKET_GRP_7_42   6

// Other stuff
#define READ_TIMEOUT 1000

#include <inttypes.h>
#include <WProgram.h>

/**
 * Callback functions for user implemented features
 */
typedef void (*drivedirect_callback)(int, int);
typedef void (*drive_callback)(int, int);

class OpenInterface
{
private:
	uint8_t OIMode;

	uint8_t batteryTemperature;
	int batteryVoltage;
	int batteryCurrent;
	int batteryCharge;
	int batteryChargeEstimate;

	int reqVelocity;
	int reqRadius;
	int reqLeftVelocity;
	int reqRightVelocity;

	drivedirect_callback driveDirectCallback;
	drive_callback driveCallback;

	void handleOpCode(byte);

	void getSensors();

	void song();

	void drive();

	void driveDirect();

	bool readBytes(uint8_t*, uint8_t);

	uint8_t getPacketId(uint8_t, uint8_t);

	bool in_array(uint8_t, uint8_t*, uint8_t);

public:
	/**
	 * Constructor
	 */
	OpenInterface();

	/**
	 * Initialise interface
	 */
	void init(long);

	/**
	 * Main loop
	 */
	void handle();

	/**
	 * Register callbacks for user implemented features
	 */
	void registerDriveDirect(drivedirect_callback f){driveDirectCallback = f;}
  void registerDrive(drive_callback f){driveCallback = f;}

  /**
   * Battery sensor information
   */
  void setBatteryInfo(int, int, int, int, uint8_t);
  void updateBatteryVoltageCurrent(int, int);
  void updateBatteryCurrent(int);
  void updateBatteryVoltage(int);
  void updateBatteryTemperature(uint8_t);
  void updateBatteryChargeEstimate(int);
};

#endif
