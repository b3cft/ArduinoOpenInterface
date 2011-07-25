#include <OpenInterface.h>

// Create an instance of the Open Interface library
// Pass in the serial port to communicate with the
// controller. Mega's could use Serial1/2/3 etc Serial for Arduinos
OpenInterface oi(&Serial);

//variables for the battery data
int batVoltage=6000, batCurrent=-250, batRemain=4500, batCharge=4500;
uint8_t batTemp=24;

/**
 * Bootup commands
 */
void setup()
{
  // Init interface at 57600 Baud
  oi.init(57600);

  // All oi.registerXXXXXX functions register a function that the controller
  // will

  // Register callback to driveDirect() defined in this sketch
  oi.registerDriveDirect(driveDirect);

  // Register callback to drive() defined in this sketch
  oi.registerDrive(drive);

  // Register callback to play songs, gets passed an array of notes, timings
  oi.registerSong(songPlay);

  // Register callback to handle Wait Distance commands, gets passed an int of mm to travel
  oi.registerWaitDistance(waitDistance);

  // Register callback to handle Wait Angle commands, gets passed an int of degrees to turn clockwise
  // Note neg values = anti-clockwise
  oi.registerWaitAngle(waitAngle);

  // Register callback to handle Wait Event commands, gets passed an 8bit bitmap of the event to look for
  oi.registerWaitEvent(waitEvent);

  // Register callback to handle LED Control commands, gets passed an 8bit bitmap of the led states
  oi.registerLedControl(controlLeds);

  // Register callback to handle PWM output commands, gets passed 3 bytes with 0-128 PWM levels for each output
  oi.registerLsdPwmControl(controlPwmLevels);

  // Register callback to handle PWM output commands, gets passed 1 bitmap byte with on/off values for each output
  oi.registerLsdOutputCallback(controlPwmOutput);

  // Register callback to handle Digital output commands, gets passed 1 bitmap byte with on/off values for each output
  oi.registerDigitalOutputCallback(controlDigitalOutput);

  // Register callback to handle IR output commands, gets passed 1 byte to send over IR
  oi.registerSendIrCallback(sendIrCommand);

  // Setup battery information
  // Voltage (mV), Current (mA)(Neg = discharging), Charge (mAh), Charge Remaining (mAh), Temp (C)
  oi.setBatteryInfo(batVoltage, batCurrent, batRemain, batCharge, batTemp);
}

/**
 * Receive one byte of data to transmit over IR
 */
void sendIrCommand(uint8_t command)
{
}

/**
 * Receive one byte of data containing a bitmap of Digital Output states
 */
void controlDigitalOutput(uint8_t ioBitmap)
{
}

/**
 * Receive one byte of data containing a bitmap of PWM Output states, i.e on/off
 */
void controlPwmOutput(uint8_t pwmBitmap)
{
}

/**
 * Receive three bytes of data containing a PWM Output levels, range 0-128
 */
void controlPwmLevels(uint8_t one, uint8_t two, uint8_t three)
{
}

/**
 * Receive three bytes of data containing LED bit, Power LED colour, Power LED intensity
 * ledsBitmap bits 1,3 for Play and Advance LEDs respectively
 * powerColour 0-255, 0=green, 255=red. Sets Power Led colour to any level between.
 * powerBrightness 0-255, 0=off, 255=full. Sets Power LED brightness
 */
void controlLeds(uint8_t ledsBitmap, uint8_t powerColour, uint8_t powerBrightness)
{
}

/**
 * Receive one int for distance to travel in mm.
 */
void waitDistance(int mm)
{
}

/**
 * Receive one int for angle to rotate in degrees. Positive values Anticlockwise, neg values clockwise
 */
void waitAngle(int degrees)
{
}

/**
 * Receive on byte. Wait until that event occurs before returning.
 */
void waitEvent(uint8_t event)
{
/*
Event                Number  Inverse
Wheel Drop           1       255
Front Wheel Drop     2       254
Left Wheel Drop      3       253
Right Wheel Drop     4       252
Bump                 5       251
Left Bump            6       250
Right Bump           7       249
Virtual Wall         8       248
Wall                 9       247
Cliff                10      246
Left Cliff           11      245
Front Left Cliff     12      244
Front Right Cliff    13      243
Right Cliff          14      242
Home Base            15      241
Advance Button       16      240
Play Button          17      239
Digital Input 0      18      238
Digital Input 1      19      237
Digital Input 2      20      236
Digital Input 3      21      235
OI Mode = Passive    22      234
*/
}


/**
 * Callback function to handle songs
 * Notes are passed as a array consisting of pairs; pitch & duration.
 * See Create Open Interface V2 documentation (op code
 */
void songPlay(uint8_t notes[])
{
}

/**
 * Callback function to handle driveDirect commands
 */
void driveDirect(int leftVel, int rightVel)
{
  Serial.print("Left:");Serial.println(leftVel,DEC);
  Serial.print("Right:");Serial.println(rightVel,DEC);
}

/**
 * Callback function to handle drive commands
 */
void drive(int velocity, int radius)
{
  Serial.print("Velocity:");Serial.println(velocity,DEC);
  Serial.print("Radius:");Serial.println(radius,DEC);
}

/**
 * Callback function to handle a call to wait distance
 */
void driveDistance(int distance)
{
  // Put your method for travelling a certain distance here.
}

void loop()
{
  /**
   * Handle requests from the OI controller
   */
  oi.handle();

  /**
   * Read data from inputs to update sensor values if they differ from previous values.
   */
  oi.updateBatteryVoltageCurrent(batVoltage, batCurrent);
  oi.updateBatteryTemperature(batTemp);
  oi.updateBatteryChargeEstimate(batCharge);
}
