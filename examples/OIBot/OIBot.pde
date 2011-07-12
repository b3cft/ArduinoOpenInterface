#include <OpenInterface.h>

// Create an instance of the
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

  // Register callback to driveDirect() defined in this sketch
  // Will be called when OI received a driveDirect Command from controller
  oi.registerDriveDirect(driveDirect);

  // Register callback to drive() defined in this sketch
  // Will be called when OI received a drive Command from controller
  oi.registerDrive(drive);

  // Setup battery information
  // Voltage (mV), Current (mA)(Neg = discharging), Charge (mAh), Charge Remaining (mAh), Temp (C)
  oi.setBatteryInfo(batVoltage, batCurrent, batRemain, batCharge, batTemp);
}

/**
 * Callback function to handle songs
 * Notes are passed as a array consisting of pairs; pitch & duration.
 * See Create Open Interface V2 documentation (op code
 */
void playSong(uint8_t notes[])
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
