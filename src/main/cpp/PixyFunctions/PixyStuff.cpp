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

double targetAngleOffset;
double targetShooterAngle;
double targetShooterPower;

// module local target data variables
int _targetColorIndex = 1;

// module local network table entries
nt::NetworkTableEntry nte_targetAngleOffset;
nt::NetworkTableEntry nte_validBlock;
nt::NetworkTableEntry nte_targetDistance;
nt::NetworkTableEntry nte_targetShooterAngle;
nt::NetworkTableEntry nte_targetShooterPower;


void targetDataHandler(bool valid_blocks, double target_angle_offset, int target_distance);

void PixyStuffInit(string nt_table_name) 
{
  // Initialize Serial Data handlers
  myPRmsgParser.setPRTGAEventHandler(targetDataHandler);

  // Setup network table entries for visualizing Pixy/Teensy data
	auto nt_inst = nt::NetworkTableInstance::GetDefault();
	auto nt_table = nt_inst.GetTable(nt_table_name);
  nte_targetAngleOffset = nt_table->GetEntry("Target/Target Angle Offset");
  nte_validBlock = nt_table->GetEntry("Target/Valid Block");
  nte_targetDistance = nt_table->GetEntry("Target/Target Distance");
  nte_targetShooterAngle = nt_table->GetEntry("Shooter/Target Angle");
  nte_targetShooterPower = nt_table->GetEntry("Shooter/Target Power");


  // set default values
  nte_targetDistance.SetDouble(0.0);

  targetAngleOffset = 0.0;
  targetShooterAngle = 350;
  targetShooterPower = 0.7;
}

// Process the datat from the pixy
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

void targetDataHandler(bool valid_blocks, double target_angle_offset, int target_distance)
{
  if (valid_blocks)
  {
    // set values used for PID targeting
    targetAngleOffset = target_angle_offset;

    // Origonal equaiton y = 1.2547x + 77.083
    targetShooterAngle = (1.2547 * target_distance) + 77.083;

    // origonal equation y = 0.1317x + 24
    if(target_distance > 350)
      targetShooterPower =  ((0.1317 * target_distance) + 24)/100;
    else
      targetShooterPower = 0.7;
  }
  else
  {
    // set values used for PID targeting
    targetAngleOffset = -0.08;
    targetShooterAngle = 350;
    targetShooterPower = 0.7;
  }

  // Update nte values
  nte_targetShooterAngle.SetDouble(targetShooterAngle);
  nte_targetShooterPower.SetDouble(targetShooterPower);
  nte_targetDistance.SetDouble(target_distance);
  nte_targetAngleOffset.SetDouble(target_angle_offset);
  nte_validBlock.SetBoolean(valid_blocks);
}
