//OP Codes
#define oc_start 128
#define oc_baud 129
#define oc_control 130
#define oc_safe 131
#define oc_full 132
#define oc_power 133
#define oc_spot 134
#define oc_clean 135
#define oc_max 136
#define oc_drive 137
#define oc_low_side_drivers 138
#define oc_leds 139
#define oc_song 140
#define oc_play 141
#define oc_sensors 142
#define oc_force_seeking_dock 143
#define oc_low_side_drivers 138
#define oc_song 140
#define oc_play_song 141
#define oc_pwm_low_side_drivers 144
#define oc_drive_direct 145
#define oc_digital_outputs 147
#define oc_stream 148
#define oc_query_list 149
#define oc_pause_resume_stream 150
#define oc_send_ir 151
#define oc_script 152 
#define oc_play_script 153
#define oc_show_script 154
#define oc_wait_time 155
#define oc_wait_distance 156
#define oc_wait_angle 157
#define oc_wait_event 158

// Other stuff
#define READ_TIMEOUT 1000

#include <LCD4Bit_mod.h> 
LCD4Bit_mod lcd = LCD4Bit_mod(2); 


byte OIMode = 3;

void setup() {
  Serial.begin(57600);
  lcd.init();
  lcd.clear();
  lcdWrite("iRobotCreateEmu");
}

void lcdWrite(char newline[])
{
  static char lcd1[32];
  static char lcd2[32];
  memcpy(lcd1, lcd2, 32);
  memcpy(lcd2, newline, 32);  
  lcd.clear();
  lcd.cursorTo(1,0);
  lcd.printIn(lcd1);
  lcd.cursorTo(2,0);
  lcd.printIn(lcd2);
  delay(100);
}


void loop() {
  if (Serial.available())
  {
    handleOpCode(Serial.read());
  }
}

bool readBytes(uint8_t* bytesIn, uint8_t count)
{
  while (count-- > 0)
  {  
    unsigned long start = millis();
    while(!Serial.available())
    {  
      if (millis() > start + READ_TIMEOUT)
      {
        lcdWrite("Timeout reading");
        return false;
      }  
    }
    *bytesIn++ = Serial.read(); 
  }
  return true;
}

void handleOpCode(byte oc)
{
  switch (oc)
  {
     case (oc_start):
       lcdWrite("Passive Mode");
       OIMode = 1; // passive
     break;
     
     case (oc_baud):
     break;

     case (oc_control):
     case (oc_safe):
       lcdWrite("Safe Mode");
       OIMode = 2; //safe
     break;
     
     case (oc_sensors):
       lcdWrite("Get Sensors");
       get_sensors();
     break;
     
     case (oc_song):
       lcdWrite("Song");
       song();
     break;
     
     case (oc_drive_direct):
       lcdWrite("Direct Drive");
       driveDirect();
     break;

     case (oc_drive):
       lcdWrite("Drive Command");
       drive();
     break;

     case (oc_full):
       lcdWrite("Full Mode");
       OIMode = 3; // full
     break;
  }
}

void song()
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

void drive()
{
       uint8_t raw[4];
       word vel, radius;
       bool result = readBytes(raw, 4);
       
       if (result)
       {
         vel    = word(raw[0], raw[1]);
         radius = word(raw[2], raw[3]);
       }
       return;
}

void driveDirect()
{
       uint8_t raw[4];
       word rightVel, leftVel;
       bool result = readBytes(raw, 4);
       
       if (result)
       {
         rightVel = word(raw[0], raw[1]);
         leftVel  = word(raw[2], raw[3]);
       }
       return;
}


uint8_t getPacketId(uint8_t id)
{ 
  switch(id)
  {
    case(10): // IR Sensor 255 = no signal
      return 255;
    break;
    
    case(22): // Battery voltage in mV  2 bytes 
      return 0x1770;
    break;
    
    case(23): // battery current in/out in mA 2 bytes
      return 0x02EE;
    break;

    case(24): // Battery temp in C
      return 24;
    break;

    case(25): // Battery charge in mAh 2 bytes
      return 0x17EE; //6000mAh
    break;
    
    case(26): // Battery charge estimated in mAh  2 bytes
      return 0x17EE;
    break;

    case(34): // OI Mode
      return OIMode;
    break;    
    
    /*
    These are sensors to be implemented
    case(7): // B 000, caster, l wheeldrop, r wheeldrop, bump l, bump r
    case(8): // wall sensor
    case(9): // left cliff
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


void get_sensors()
{
    uint8_t id[1];
    char strId[3];
    readBytes(id, 1);
    byte sensor[52];
    switch (id[0])
    {
       case(6):
         sensor[0] = getPacketId(7);  // 7, B 000, caster, l wheeldrop, r wheeldrop, bump l, bump r
         sensor[1] = 0;       // 8, wall sensor
         sensor[2] = 0;       // 9, left cliff
         sensor[3] = 0;       // 10, left front cliff
         sensor[4] = 0;       // 11, right front cliff
         sensor[5] = 0;       // 12, right cliff
         sensor[6] = 0;       // 12, virtual wall
         sensor[7] = 0;       // 14, B 000. l wheel overcurrent, r wheel oc, LD2, LD1, LD0
         sensor[8] = 0;       // 15, not used
         sensor[9] = 0;       // 16, not used    
         sensor[10] = 255;    // 17, IR, no value = 255
         sensor[11] = 0;      // 18, B 00000, advance, 0, play
         sensor[12] = 0;      // 19, distance in mm travelled since last update. neg for backwards. Sum both wheels/2
         sensor[13] = 0;      // 19, ^^ two bytes
         sensor[14] = 0;      // 20, angle in degrees turned since last update, CCW positive, Clockwise neg
         sensor[15] = 0;      // 20, ^^ two bytes
         sensor[16] = 0;      // 21, charging state
         sensor[17] = 0x17;   // 22, Voltage in mV using 6v
         sensor[18] = 0x70;   // 22, ^^ two bytes
         sensor[19] = 0x02;   // 23, Current in/out mA using 750mA
         sensor[20] = 0xEE;   // 23, ^^ two bytes
         sensor[21] = 24;     // 24, battery temp in C
         sensor[22] = 0x17;   // 25, battery charge in mAh using 6000mAh
         sensor[23] = 0xEE;   // 25, ^^ two bytes
         sensor[24] = 0x17;   // 26, battery charge estimated in mAh using 6000mAh
         sensor[25] = 0xEE;   // 26, ^^ two bytes
         sensor[26] = 0;      // 27, Wall sensor stength
         sensor[27] = 0;      // 27, ^^ two bytes 
         sensor[28] = 0;      // 28, Cliff left stength
         sensor[29] = 0;      // 28, ^^ two bytes 
         sensor[30] = 0;      // 29, Cliff left front stength
         sensor[31] = 0;      // 29, ^^ two bytes 
         sensor[32] = 0;      // 30, Cliff right front stength
         sensor[33] = 0;      // 30, ^^ two bytes 
         sensor[34] = 0;      // 31, Cliff right stength
         sensor[35] = 0;      // 31, ^^ two bytes 
         sensor[36] = 0;      // 32, 000 device detect, DIO3, DIO2, DIO1, DIO0
         sensor[37] = 0;      // 33, analog input 0-1023
         sensor[38] = 0;      // 33, ^^ two bytes 
         sensor[39] = 0;      // 34, charging sources
         sensor[40] = OIMode; // 35, OI mode
         sensor[41] = 0;      // 36, current song
         sensor[42] = 0;      // 37, song playing
         sensor[43] = 0;      // 38, number of data stream packets
         sensor[44] = 0;      // 39, requested velocity in mm/s -500 / 500
         sensor[45] = 0;      // 39, ^^ two bytes 
         sensor[46] = 0;      // 40, Requested radius 
         sensor[47] = 0;      // 40, ^^ two bytes
         sensor[48] = 0;      // 41, Right requested velocity mm/s -500/500
         sensor[49] = 0;      // 41, ^^ two bytes
         sensor[50] = 0;      // 42, Left requested velocity mm/s -500/500
         sensor[51] = 0;      // 42, ^^ two bytes
       break;
    }
    Serial.write(sensor, 52);
}
