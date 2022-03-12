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
#include <frc/controller/PIDController.h>
#include <units/angular_velocity.h>

// Development definitions
//#define DEBUG_SERIAL
//#define ECHO_SERIAL1
//#define DEBUG_TURRET_DATA_TO_COUT
//#define DEBUG_CAMERA_ONE

using namespace std;

int i;

PIXY_BLOCK_TYPE filtered_blocks[MAX_VALID_BLOCKS];
PIXY_BLOCK_TYPE read_blocks[MAX_NR_OF_BLOCKS];

// Create an instance of the Serial data parser
PRmsgParser myPRmsgParser;

unsigned int delta_x;

float exp_filter_constant;
float exp_filter_oneMinConst;

// module local turret data variables
static bool target_visible_;
static int target_lateral_pos_;
int target_block_index = -1;
int _targetColorIndex = 1;
//float target_x_velocity = 0.0;
//float target_y_velocity = 0.0;
double yawKp = 1.0;
double yawKi = 0.0;
double yawKd = 0.0;

double yCenterOfTarget;
double xCenterOfTarget;

// module local network table entries
nt::NetworkTableEntry nte_target_data_x;
nt::NetworkTableEntry nte_target_data_y;
nt::NetworkTableEntry nte_target_data_w;
nt::NetworkTableEntry nte_target_data_h;
nt::NetworkTableEntry nte_yawKp;
nt::NetworkTableEntry nte_yawKi;
nt::NetworkTableEntry nte_yawKd;
nt::NetworkTableEntry nte_yCenterOfTarget;
nt::NetworkTableEntry nte_xCenterOfTarget;
nt::NetworkTableEntry nte_centerTargetNum;
nt::NetworkTableEntry nte_target_visible;

void blockDataHandler(uint8_t camera_number, uint32_t camera_timestamp, uint8_t n_parsed_blocks, 
                        PIXY_BLOCK_TYPE *parsed_blocks);

void PixyStuffInit(string nt_table_name) 
{
  // initialize filter
  exp_filter_constant = EXP_FILT_SPEED_CONSTANT;
  exp_filter_oneMinConst = 1.0 - exp_filter_constant;

  // Initialize Serial Data handlers
  myPRmsgParser.setPRBLKEventHandler(blockDataHandler);

  // Setup network table entries for visualizing Pixy/Teensy data
	auto nt_inst = nt::NetworkTableInstance::GetDefault();
	auto nt_table = nt_inst.GetTable(nt_table_name);
  nte_target_data_x = nt_table->GetEntry("Target/Data/x");
  nte_target_data_y = nt_table->GetEntry("Target/Data/y");
  nte_target_data_w = nt_table->GetEntry("Target/Data/w");
  nte_target_data_h = nt_table->GetEntry("Target/Data/h");
  nte_yawKp = nt_table->GetEntry("Yaw Pid/Kp");
  nte_yawKi = nt_table->GetEntry("Yaw Pid/Ki");
  nte_yawKd = nt_table->GetEntry("Yaw Pid/Kd");
  nte_yCenterOfTarget = nt_table->GetEntry("Target Data/X Center");
  nte_xCenterOfTarget = nt_table->GetEntry("Target Data/Y Center");
  nte_centerTargetNum = nt_table->GetEntry("Target/Center Target Number");
  nte_target_visible = nt_table->GetEntry("Target/Target Visible");
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

void blockDataHandler(uint8_t camera_number, uint32_t camera_timestamp, uint8_t n_parsed_blocks, PIXY_BLOCK_TYPE *parsed_blocks)
{
  int target_index = -1;
  int i = -1;
#ifdef DEBUG_CAMERA_ONE
  std::cout << "block data handler " << int(n_parsed_blocks);
  std::cout << " timestamp " << int(camera_timestamp);
  for (i=0; i<n_parsed_blocks; i++)
  {
    std::cout << "," << int(parsed_blocks[i].sig);
    std::cout << "," << int(parsed_blocks[i].x);
    std::cout << "," << int(parsed_blocks[i].y);
    std::cout << "," << int(parsed_blocks[i].w);
    std::cout << "," << int(parsed_blocks[i].h);
  }
  std::cout << std::endl;
#endif 
  target_index = -1;

  for (uint8_t i=0; i<n_parsed_blocks; i++)
  {
    if (1 == parsed_blocks[i].sig)
    {
      target_index = i;
      i = n_parsed_blocks;
    }
  }

  if (target_index >= 0 && parsed_blocks[target_index].w != 0)
  { 
    double center_number_target = nte_centerTargetNum.GetDouble(200.0);     
    target_visible_ = true;
    target_lateral_pos_ = (int)center_number_target - (int)(parsed_blocks[target_index].x + (parsed_blocks[target_index].w/2));
    nte_target_data_x.SetDouble(parsed_blocks[target_index].x);
    nte_target_data_y.SetDouble(parsed_blocks[target_index].y);
    nte_target_data_w.SetDouble(parsed_blocks[target_index].w);
    nte_target_data_h.SetDouble(parsed_blocks[target_index].h);
    nte_target_visible.SetDouble(1);
  }
  else
  {
    target_visible_ = false;
    target_lateral_pos_ = 0;
    nte_target_data_x.SetDouble(0);
    nte_target_data_y.SetDouble(0);
    nte_target_data_w.SetDouble(0);
    nte_target_data_h.SetDouble(0);
    nte_target_visible.SetDouble(0);
  }
}

void CreateYawPID()
{
  // Getting the PID values from shuffleboard
  yawKp = nte_yawKp.GetDouble(1.0);
  yawKi = nte_yawKi.GetDouble(0.0);
  yawKd = nte_yawKd.GetDouble(0.0);

  // Using the new values, create the PID controller
  frc::PIDController ShooterYawPID{yawKp, yawKi, yawKd};
  ShooterYawPID.EnableContinuousInput((double)-units::radian_t(wpi::numbers::pi), (double)units::radian_t(wpi::numbers::pi));
}

void CenterRobot(Drivetrain &m_drive)
{
  // TODO: create a funciton to drive while looking at the target.
}