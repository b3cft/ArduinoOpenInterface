#include <OpenInterface.h>

OpenInterface oi;

void setup()
{
  oi.init(57600);
  oi.registerDriveDirect(driveDirect);
  oi.registerDrive(drive);
}

void driveDirect(int leftVel, int rightVel)
{
  Serial.print("Left:");Serial.println(leftVel,DEC);
  Serial.print("Right:");Serial.println(rightVel,DEC);
}

void drive(int velocity, int radius)
{
  Serial.print("Velocity:");Serial.println(velocity,DEC);
  Serial.print("Radius:");Serial.println(radius,DEC);
}

void loop()
{
  oi.handle();
}