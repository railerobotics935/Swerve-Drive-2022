

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "RobotFunction.h"

RobotFunction::RobotFunction()
{

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");
  nte_intakeLiftEncoderValue = nt_table->GetEntry("Intake/Encoder");
  nte_colorsensorRed = nt_table->GetEntry("Color Sensor/Red");
  nte_colorsensorGreen = nt_table->GetEntry("Color Sensor/Green");
  nte_colorsensorBlue = nt_table->GetEntry("Color Sensor/Blue");
  nte_colorsensorProximity = nt_table->GetEntry("Color Sensor/Proximity");

  // Set intakeLiftMotorL to follow intakeLiftMotorR
  intakeLiftMotorL.Follow(intakeLiftMotorR);
  intakeLiftMotorR.SetInverted(false);
  intakeLiftMotorL.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster); // Set left to mirror right

  // Set neutral mode of lift motors to brake mode - more resistant
  intakeLiftMotorR.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  intakeLiftMotorL.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  // Config Motor controler
  intakeRoller.ConfigFactoryDefault();
  intakeLiftMotorR.ConfigFactoryDefault();
  intakeLiftMotorL.ConfigFactoryDefault();

  // Reset the encoder
  intakeLiftEncoder.Reset();
}

// Set power for Intake Roller
void RobotFunction::SetIntakeRoller(double power)
{
  intakeRoller.Set(power);
}

// Set power for ball storage belt
void RobotFunction::SetBallStorageBelt(double power)
{
  ballStorageBelt.Set(power);
}

// Set power for shooter feeder
void RobotFunction::SetShooterFeeder(double power)
{
  shooterFeeder.Set(-power);
}

// Moves the intake lift either up or down depending on the previous pos
void RobotFunction::SetIntakeLift(bool intakeDown)
{
  if(intakeDown)
  {
    if(intakeLiftEncoder.Get() > 70)
      intakeLiftMotorR.Set(-0.2);
    else
      intakeLiftMotorR.Set(0);
  }
  else
  {
    if(intakeLiftEncoder.Get() < 250)
      intakeLiftMotorR.Set(0.5);
    else
      intakeLiftMotorR.Set(0);
  }
}

// Sets Color values to Networktables and returns red or blue
std::string RobotFunction::GetSensorColor()
{
  // Get color and proximity from the color sensor
  detectedColor = colorSensor.GetColor();
  proximity = colorSensor.GetProximity();
  
  // Set the network table entries
  nte_colorsensorRed.SetDouble(detectedColor.red);
  nte_colorsensorGreen.SetDouble(detectedColor.green);
  nte_colorsensorBlue.SetDouble(detectedColor.blue);

  // Determine the one with the largest color
  if(detectedColor.red > detectedColor.blue)
    return "blue";
  else
    return "red";
}

// Returns the proximity
double RobotFunction::GetSensorProximity()
{
  // Get color and proximity from the color sensor
  proximity = colorSensor.GetProximity();

  // Set the network table entries
  nte_colorsensorProximity.SetDouble(proximity);
  
  return proximity;
}

// Updates the Network tables entrys
void RobotFunction::UpdateNTE()
{
  nte_intakeLiftEncoderValue.SetDouble(intakeLiftEncoder.Get());
}
