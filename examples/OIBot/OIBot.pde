#include <EEPROM.h>
#include <OpenInterface.h>

/* Pin assignments */
#define IR1Read         0 //Analog
#define IR2Read         1 //Analog

#define Interrupt0      2 //Do not use
#define Interrupt1      3 //Do not use

#define rightStall      4
#define rightDirection  5
#define rightSpeed      6 //PWM

#define leftStall       8
#define leftDirection   9
#define leftSpeed       10 //PWM

// Create an instance of the
OpenInterface oi(&Serial);

/**
 * Init OI, then setup the Motor H-bridge
 */
void setup() {
  oi.init(57600);
  motorSetup();
}

/**
 * Loop forever
 */
void loop()
{
  checkMotorStall();
  oi.handle();
}
