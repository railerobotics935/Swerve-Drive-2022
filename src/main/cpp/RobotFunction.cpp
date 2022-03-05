

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
  nte_shooterAngleEncoderValue = nt_table->GetEntry("Shooter Angle/Encoder");
  nte_colorSensorRed = nt_table->GetEntry("Color Sensor/Red");
  nte_colorSensorGreen = nt_table->GetEntry("Color Sensor/Green");
  nte_colorSensorBlue = nt_table->GetEntry("Color Sensor/Blue");
  nte_colorSensorProximity = nt_table->GetEntry("Color Sensor/Proximity");

  // Set intakeLiftMotorL to follow intakeLiftMotorR
  intakeLiftMotorL.Follow(intakeLiftMotorR);
  intakeLiftMotorR.SetInverted(false);
  intakeLiftMotorL.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster); // Set left to mirror right

  // Set shooterMotor2 and shooterMotor3 to follow shooterMotor1
  shooterMotor2.Follow(shooterMotor1);
  shooterMotor3.Follow(shooterMotor1);
  shooterMotor1.SetInverted(false);
  shooterMotor2.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
  shooterMotor3.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);

  // Set neutral mode of lift motors to brake mode - more resistant
  intakeLiftMotorR.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  intakeLiftMotorL.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  // Config Motor controler
  intakeRollerMotor.ConfigFactoryDefault();
  intakeLiftMotorR.ConfigFactoryDefault();
  intakeLiftMotorL.ConfigFactoryDefault();
  shooterMotor1.ConfigFactoryDefault();
  shooterMotor2.ConfigFactoryDefault();
  shooterMotor3.ConfigFactoryDefault();
  shooterAngle.ConfigFactoryDefault();

  // Reset the encoder
  intakeLiftEncoder.Reset();
  shooterAngleEncoder.Reset();
}

// Set power for Intake Roller
void RobotFunction::SetIntakeRoller(double power)
{
  intakeRollerMotor.Set(power);
}

// Set power for ball storage belt
void RobotFunction::SetBallStorageBelt(double power)
{
  ballStorageBeltMotor.Set(power);
}

// Set power for shooter feeder
void RobotFunction::SetShooterFeeder(double power)
{
  shooterFeederMotor.Set(-power);
}

// Set power for shooter
void RobotFunction::SetShooter(double power)
{
  shooterMotor1.Set(power);
}

// Set power for shooter angle
void RobotFunction::SetShooterAngle(double power)
{
  shooterAngle.Set(power);
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
    if(intakeLiftEncoder.Get() < 300)
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
  
  return proximity;
}

// Updates the Network tables entrys
void RobotFunction::UpdateNTE()
{
  // Get encoder values
  nte_intakeLiftEncoderValue.SetDouble(intakeLiftEncoder.Get());
  nte_shooterAngleEncoderValue.SetDouble(shooterAngleEncoder.Get());

  // Get color and proximity from the color sensor
  detectedColor = colorSensor.GetColor();
  proximity = colorSensor.GetProximity();

  // Update nte for color sensor
  nte_colorSensorProximity.SetDouble(proximity);
  nte_colorSensorRed.SetDouble(detectedColor.red);
  nte_colorSensorGreen.SetDouble(detectedColor.green);
  nte_colorSensorBlue.SetDouble(detectedColor.blue);
}
