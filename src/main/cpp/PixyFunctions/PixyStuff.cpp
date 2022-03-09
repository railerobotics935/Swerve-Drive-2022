#include "PixyFunctions/PixyStuff.h"
#include "PixyFunctions/PixyDefines.h"
#include "PixyFunctions/PRmsgParser.h"
#include "PixyFunctions/defines.h"
#include "frc/Timer.h"

#include <iostream>
#include <string>
#include <iomanip> 
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

// Development definitions
//#define DEBUG_SERIAL
//#define ECHO_SERIAL1
//#define DEBUG_TURRET_DATA_TO_COUT

using namespace std;

PIXY_BLOCK_TYPE filtered_blocks[MAX_VALID_BLOCKS];
PIXY_BLOCK_TYPE read_blocks[MAX_NR_OF_BLOCKS];

// Create an instance of the Serial data parser
PRmsgParser myPRmsgParser;

int i, j;

unsigned int delta_x;

float exp_filter_constant;
float exp_filter_oneMinConst;

bool new_ball_location = false;
int ball_x_location = -1;
int ball_y_location = -1;
int ball_delta_x = 0;
int ball_delta_y = 0;
static double ball_distance_ = 0.0;
int ball_block_index = -1;
int _ballColorIndex = 3;
//float ball_x_velocity = 0.0;
//float ball_y_velocity = 0.0;

// module local network table entries
nt::NetworkTableEntry nte_kickwheel_speed;
nt::NetworkTableEntry nte_hood_angle;
nt::NetworkTableEntry nte_target_distance;

void SetBallColorIndex(int ballColorIndex){

  _ballColorIndex = ballColorIndex;
}
void blockDataHandler(uint8_t camera_number, uint32_t camera_timestamp, uint8_t n_parsed_blocks, 
                        PIXY_BLOCK_TYPE *parsed_blocks);

void turretDataHandler(uint32_t turret_timestamp, uint16_t kickwheel_speed, uint16_t hood_angle_1,
                        uint16_t hood_angle_2, int target_distance);

void PixyStuffInit(string nt_table_name) 
{
  // initialize filter
  exp_filter_constant = EXP_FILT_SPEED_CONSTANT;
  exp_filter_oneMinConst = 1.0 - exp_filter_constant;

  // Initialize Serial Data handlers
  myPRmsgParser.setPRBLKEventHandler(blockDataHandler);
  myPRmsgParser.setPRTUREventHandler(turretDataHandler);

  // Setup network table entries for visualizing Pixy/Teensy data
	auto nt_inst = nt::NetworkTableInstance::GetDefault();
	auto nt_table = nt_inst.GetTable(nt_table_name);
  nte_kickwheel_speed = nt_table->GetEntry("Kickwheel Speed");
  nte_hood_angle = nt_table->GetEntry("Hood Angle");
  nte_target_distance = nt_table->GetEntry("Target Distance");
}

int PixyProcessData(int n_bytes_read, char uartbuffer[])
{
  // Use NMEA style data parser to parse the data received from the Teensy/Pixy
  int buffer_index = 0;

  while (buffer_index < n_bytes_read)
  {
    myPRmsgParser.parseChar(uartbuffer[buffer_index]);
    buffer_index++;
  }

  return 0;
}

void turretDataHandler(uint32_t turret_timestamp, uint16_t kickwheel_speed, uint16_t hood_angle_1, uint16_t hood_angle_2, int target_distance)
{
  nte_kickwheel_speed.SetDouble(kickwheel_speed);
  nte_hood_angle.SetDouble(hood_angle_1);
  nte_target_distance.SetDouble(target_distance);

#ifdef DEBUG_TURRET_DATA_TO_COUT
  printf("Turret %lu, %u, %u, %d\n", turret_timestamp, kickwheel_speed, hood_angle_1, target_distance);
#endif
}

void blockDataHandler(uint8_t camera_number, uint32_t camera_timestamp, uint8_t n_parsed_blocks, PIXY_BLOCK_TYPE *parsed_blocks)
{
/*
  std::cout << "block data handler " << int(n_parsed_blocks);
  std::cout << " timestamp " << int(camera_timestamp);
  for (uint8_t i=0; i<n_parsed_blocks; i++)
  {
    std::cout << "," << int(parsed_blocks[i].sig);
    std::cout << "," << int(parsed_blocks[i].x);
    std::cout << "," << int(parsed_blocks[i].y);
    std::cout << "," << int(parsed_blocks[i].w);
    std::cout << "," << int(parsed_blocks[i].h);
  }
  std::cout << std::endl;
  return;
*/
  ball_block_index = -1;

  for (uint8_t i=0; i<n_parsed_blocks; i++)
  {
    if (_ballColorIndex == parsed_blocks[i].sig)
    {
      ball_block_index = i;
      i = n_parsed_blocks;
    }
  }

  if (ball_block_index >= 0 && parsed_blocks[ball_block_index].w != 0)
  {
    ball_delta_x = parsed_blocks[ball_block_index].x + (parsed_blocks[ball_block_index].w/2) - 200;
    ball_delta_y = parsed_blocks[ball_block_index].y + (parsed_blocks[ball_block_index].h/2)- 160;
    
    new_ball_location = true;
  //y=0.3716x2-10.218x+149.29
    ball_distance_ = 0.3716*double(parsed_blocks[ball_block_index].y)*double(parsed_blocks[ball_block_index].y) 
- 10.218*double(parsed_blocks[ball_block_index].y) + 149.29;
  
    nte_target_distance.SetDouble(ball_distance_);
    //cout << "ball y: " << parsed_blocks[ball_block_index].y  << " h: " << parsed_blocks[ball_block_index].h << endl;
 
  }
  else
  {
    ball_distance_ = 0.0;
    nte_target_distance.SetDouble(0);  
  }
}

bool PixyAdjustCamera(double *servo_pan, double *servo_tilt, bool *ball_visible, double *ball_distance)
{
  bool servo_adjusted = false;
  if (new_ball_location)
  {
    new_ball_location = false;
    *servo_pan += (((double)abs(ball_delta_x) * (double)ball_delta_x) / 260000.0);
    if (*servo_pan < 0.2)
      *servo_pan = 0.2;

    if (*servo_pan > 0.8)
      *servo_pan = 0.8;
  
    *servo_tilt += (((double)abs(ball_delta_y) * (double)ball_delta_y) / 260000.0);
    if (*servo_tilt < 0.3)
      *servo_tilt = 0.3;

    if (*servo_tilt > 0.7)
      *servo_tilt = 0.7;
    servo_adjusted = true;

    *ball_visible = true;
    *ball_distance = ball_distance_;
#if 0
    double timestamp = frc::GetTime();
    std::cout << std::fixed << std::cout.precision(3);
    std::cout << "servo_adjusted " << timestamp << std::endl;
#endif
  }
  else
  {
    *ball_visible = false;
  }
  
  return servo_adjusted;
}
