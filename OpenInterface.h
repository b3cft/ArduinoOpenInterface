#ifndef OpenInterface_h
#define OpenInterface_h

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
};

#endif
