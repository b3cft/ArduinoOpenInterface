//OP Codes
#define oc_start                0x80 //128
#define oc_baud                 0x81 //129
#define oc_control              0x82 //130
#define oc_safe                 0x83 //131
#define oc_full                 0x84 //132
#define oc_power                0x85 //133
#define oc_spot                 0x86 //134
#define oc_clean                0x87 //135
#define oc_max                  0x88 //136
#define oc_drive                0x89 //137
#define oc_low_side_drivers     0x8a //138
#define oc_leds                 0x8b //139
#define oc_song                 0x8c //140
#define oc_play_song            0x8d //141
#define oc_sensors              0x8e //142
#define oc_force_seeking_dock   0x8f //143
#define oc_pwm_low_side_drivers 0x90 //144
#define oc_drive_direct         0x91 //145
#define oc_digital_outputs      0x93 //147
#define oc_stream               0x94 //148
#define oc_query_list           0x95 //149
#define oc_pause_resume_stream  0x96 //150
#define oc_send_ir              0x97 //151
#define oc_script               0x98 //152 
#define oc_play_script          0x99 //153
#define oc_show_script          0x9a //154
#define oc_wait_time            0x9b //155
#define oc_wait_distance        0x9c //156
#define oc_wait_angle           0x9d //157
#define oc_wait_event           0x9e //158

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
#define DEBUG_LCD 1
//#define DEBUG_SERIAL 1
extern "C" {
  #include <inttypes.h>
}

#ifdef DEBUG_LCD
#include <LCD4Bit_mod.h> 
LCD4Bit_mod lcd = LCD4Bit_mod(2); 
#endif

byte OIMode = 3;

void setup() {
  Serial.begin(57600);
#ifdef DEBUG_LCD
  lcd.init();
  lcd.clear();
  lcdWrite("iRobotCreateEmu");
#endif
}

void lcdWrite(char newline[])
{
  static char lcd1[32];
  static char lcd2[32];
  memcpy(lcd1, lcd2, 32);
  memcpy(lcd2, newline, 32);  
#ifdef DEBUG_LCD
  lcd.clear();
  lcd.cursorTo(1,0);
  lcd.printIn(lcd1);
  lcd.cursorTo(2,0);
  lcd.printIn(lcd2);
  delay(100);
#endif
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
#ifdef DEBUG_LCD
        lcdWrite("Timeout reading");
#endif
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
#ifdef DEBUG_LCD
       lcdWrite("Passive Mode");
#endif
       OIMode = 1; // passive
     break;
     
     case (oc_baud):
     break;

     case (oc_control):
     case (oc_safe):
#ifdef DEBUG_LCD
       lcdWrite("Safe Mode");
#endif
       OIMode = 2; //safe
     break;
     
     case (oc_sensors):
#ifdef DEBUG_LCD
       lcdWrite("Get Sensors");
#endif
       get_sensors();
     break;
     
     case (oc_song):
#ifdef DEBUG_LCD
       lcdWrite("Song");
#endif
       song();
     break;
     
     case (oc_drive_direct):
#ifdef DEBUG_LCD
       lcdWrite("Direct Drive");
#endif
       driveDirect();
     break;

     case (oc_drive):
#ifdef DEBUG_LCD
       lcdWrite("Drive Command");
#endif
       drive();
     break;

     case (oc_full):
#ifdef DEBUG_LCD
       lcdWrite("Full Mode");
#endif
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
       int vel, radius;
       bool result = readBytes(raw, 4);
       
       if (result)
       {
         vel    = int(word(raw[0], raw[1]));
         radius = int(word(raw[2], raw[3]));
       }
       return;
}

void driveDirect()
{
       uint8_t raw[4];
       int rightVel, leftVel;
       bool result = readBytes(raw, 4);
       
       if (result)
       {
         rightVel = int(word(raw[0], raw[1]));
         leftVel  = int(word(raw[2], raw[3]));
       }
#ifdef DEBUG_LCD       
       if (rightVel==0 && leftVel==0)
       {
         lcdWrite("All Stop");
       }
#endif
       return;
}

uint8_t getPacketId(uint8_t id, uint8_t num=1)
{ 
  switch(id)
  {
    case(17): // IR Sensor 255 = no signal
      return 255;
    break;
    
    case(22): // Battery voltage in mV  2 bytes
      if (num==1) return 0x17;
      if (num==2) return 0x70;
    break;
    
    case(23): // battery current in/out in mA 2 bytes
      if (num==1) return 0x02;
      if (num==2) return 0xEE;
    break;

    case(24): // Battery temp in C
      return 24;
    break;

    case(25): // Battery charge in mAh 2 bytes
      if (num==1) return 0x17;
      if (num==2) return 0x70;
    break;
    
    case(26): // Battery charge estimated in mAh  2 bytes
      if (num==1) return 0x17;
      if (num==2) return 0x70;
    break;

    case(35): // OI Mode
      return OIMode;
    break;    
    
    /*
    These are sensors yet to be implemented
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

bool in_array(uint8_t needle, uint8_t* haystack, uint8_t max)
{
    if (max==0) return false;

    for(int i=0; i<max; i++)
        if (haystack[i]==needle)
            return true;
    return false;
}

void get_sensors()
{
    uint8_t doublePackets[] = {19,20,22,23,25,26,27,28,29,30,31,33,39,40,41,42};
    uint8_t id[1], pStart, pStop, sensorLen;
    char strId[3];
    readBytes(id, 1);
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
        Serial.print("byte[");Serial.print(packet, DEC);Serial.print("] packet[");Serial.print(p, DEC);Serial.print("]: ");Serial.println(getPacketId(p), DEC);
#endif
        sensor[packet++] = getPacketId(p, 2);
      }
    }
   
    Serial.write(sensor, sensorLen);
}
