#include <OpenInterface.h>

OpenInterface oi;

void setup()
{
  oi.init(57600);
  oi.registerDirectDrive(directDrive);
}

void directDrive(int leftVel, int rightVel)
{
  Serial.print("Left:");Serial.println(leftVel,DEC);
  Serial.print("Right:");Serial.println(rightVel,DEC);
}

void loop()
{
  oi.handle();
}