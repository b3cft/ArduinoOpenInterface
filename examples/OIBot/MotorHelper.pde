#define MAX_SPEED 255
#define FORWARD HIGH
#define REVERSE LOW
#define LEFT_ODO_ADDR 0;
#define RIGHT_ODO_ADDR 4;

volatile int leftCounter = 0;
volatile int rightCounter = 0;
volatile long leftOdometer  = 0;
volatile long rightOdometer = 0;
int leftTarget    = 0;
int rightTarget   = 0;

/**
 * Setup Motor Pinouts etc.
 */
void motorSetup()
{
//  restoreOdometer();
  pinMode(leftSpeed, OUTPUT);
  pinMode(rightSpeed, OUTPUT);
  pinMode(leftDirection, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  pinMode(leftStall, INPUT);
  pinMode(rightStall, INPUT);
  digitalWrite(leftDirection, FORWARD);
  digitalWrite(rightDirection, FORWARD);
  analogWrite(leftSpeed, 0);
  analogWrite(rightSpeed, 0);

  oi.registerDriveDirect(driveDirect);

  /* pins 2&3 are intterupts for rotary encoders */
  attachInterrupt(0, leftInterrupt, CHANGE);  //pin 2
  attachInterrupt(1, rightInterrupt, CHANGE); //pin 3
}

/**
 * Store odometer readings in EEPROM
 */
void storeOdometer()
{
  for (int i=0; i<4; i++)
  {
    uint8_t leftByte, rightByte, lAddr, rAddr;
    leftByte  = byte((leftOdometer >> (8*i)));
    rightByte = byte((rightOdometer >> (8*i)));
    lAddr = LEFT_ODO_ADDR+i;
    rAddr = RIGHT_ODO_ADDR+i;
    EEPROM.write(leftByte, lAddr);
    EEPROM.write(rightByte, rAddr);
  }
}

/**
 * Restore odometer readings from EEPROM
 */

void restoreOdometer()
{
  long leftTot=0, rightTot=0;
  for (int i=0; i<4; i++)
  {
    uint8_t leftByte, rightByte, lAddr, rAddr;
    lAddr = LEFT_ODO_ADDR+i;
    rAddr = RIGHT_ODO_ADDR+i;
    leftByte  = EEPROM.read(lAddr);
    rightByte = EEPROM.read(rAddr);
    leftTot  += (leftByte << (8*i));
    rightTot += (rightByte << (8*i));
  }
  if (leftTot == 0xffffffff && rightTot == 0xffffffff)
  {
    // No value set or rollover (Backwards!) so reset to 0;
    leftOdometer  = 0;
    rightOdometer = 0;
//    storeOdometer();
  }
  else
  {
    leftOdometer  = leftTot;
    rightOdometer = rightTot;
  }
}

void leftInterrupt() {
  leftCounter++;
  leftOdometer++;
  if (leftTarget != 0 && leftCounter >= leftTarget)
  {
    leftStop();
  }
}

void rightInterrupt() {
  rightCounter++;
  rightOdometer++;
  if (rightTarget != 0 && rightCounter >= rightTarget)
  {
    rightStop();
  }
}

void checkMotorStall()
{
  if (digitalRead(leftStall) == HIGH || digitalRead(rightStall) == HIGH)
  {
    allStop();
  }
}

void moveLeft(int distance, int dir=FORWARD, int speed=100)
{
  leftTarget  = distance;
  leftCounter = 0;
  digitalWrite(leftDirection, dir);
  analogWrite(leftSpeed, speed);
}

void moveRight(int distance, int dir=FORWARD, int speed=100)
{
  rightTarget  = distance;
  rightCounter = 0;
  digitalWrite(rightDirection, dir);
  analogWrite(rightSpeed, speed);
}

void leftStop()
{
  analogWrite(leftSpeed, 0);
}

void rightStop()
{
  analogWrite(rightSpeed, 0);
}

void allStop()
{
  leftStop();
  rightStop();
}

/**
 * Handle a request to drive at a constant speed in x mm/s per wheel
 */
void driveDirect(int leftVel, int rightVel)
{
  int LV = map(abs(leftVel), 0, 500, 0, MAX_SPEED);
  int RV = map(abs(rightVel), 0, 500, 0, MAX_SPEED);
  if (leftVel < 0)
    moveLeft(0, REVERSE, LV);
  else
    moveLeft(0, FORWARD, LV);
  if (rightVel < 0)
    moveRight(0, REVERSE, RV);
  else
    moveRight(0, FORWARD, RV);
}
