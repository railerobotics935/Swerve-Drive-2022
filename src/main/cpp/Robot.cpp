// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Robot.h"

#include <iostream>

#include <ctre/phoenix/motorcontrol/InvertType.h>
#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

//#include "Drivetrain.h"

//#define ALAN_CONTROL
#ifdef  ALAN_CONTROL
#define AXIS1_X 2
#define AXIS1_Y 3
#define AXIS2_X 0
#else
#define AXIS1_X 1
#define AXIS1_Y 0
#define AXIS2_X 2
#endif

void Robot::RobotInit()
{
  m_fieldRelative = true;

  shooterOn = false;

  // Set intakeLiftMotorL to follow intakeLiftMotorR
  intakeLiftMotorL.Follow(intakeLiftMotorR);
  intakeLiftMotorR.SetInverted(false);
  intakeLiftMotorL.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster); // Set left to mirror right

  // Set neutral mode of lift motors to brake mode - more resistant
  intakeLiftMotorR.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  intakeLiftMotorL.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  // Initialize the 2D field widget
  frc::SmartDashboard::PutData("Field", &m_field);

  intakeRoller.ConfigFactoryDefault();
  intakeLiftMotorR.ConfigFactoryDefault();
  intakeLiftMotorL.ConfigFactoryDefault();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  // Resetting Pose2d and the odometry
  frc::Pose2d m_Pose{(units::meter_t)0.0, (units::meter_t)0.0, frc::Rotation2d((units::radian_t)0.0)};
  m_drive.ResetOdometry(m_Pose);
}

void Robot::TeleopPeriodic()
{
  // Switching between robot relative and field relative (blue)
  if (m_driveController.GetRawButtonPressed(1))
  {
    m_fieldRelative = !m_fieldRelative;
    if (m_fieldRelative)
      std::cout << "Drive set to FIELD Relative\n\r";
    else
      std::cout << "Drive set to ROBOT Relative\n\r";
  }

  // Reset the gyro by pressing a button (green)
  if (m_driveController.GetRawButtonPressed(2))
    m_drive.ResetGyro();

  // Initialize the sequence state of the automated function
  if (m_driveController.GetRawButtonPressed(3))
    m_Tricks.LocateAndLoadBall(m_drive, "none", AutomatedFunctions::FunctionCmd::kStartFunction);

  // Drive the robot, keep Red Button pressed to run an automated function
  if (m_driveController.GetRawButton(3))
  {
    if (frc::DriverStation::GetAlliance	() == frc::DriverStation::Alliance::kRed)
      m_Tricks.LocateAndLoadBall(m_drive, "red ball", AutomatedFunctions::FunctionCmd::kRunFunction);
    else if (frc::DriverStation::GetAlliance	() == frc::DriverStation::Alliance::kBlue)
      m_Tricks.LocateAndLoadBall(m_drive, "blue ball", AutomatedFunctions::FunctionCmd::kRunFunction);
    else
      std::cout << "Please specify the Alliance\n\r";
  }
  else
  {
    DriveWithJoystick(m_fieldRelative);
  }

  // Reset the sequence state of the automated function
  if (m_driveController.GetRawButtonReleased(3))
    m_Tricks.LocateAndLoadBall(m_drive, "none", AutomatedFunctions::FunctionCmd::kStopFunction);
}

void Robot::DisabledInit()
{
  // brake to 0 speed and rotation
  m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
}

void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::DriveWithJoystick(bool fieldRelative)
{
  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed = -m_xspeedLimiter.Calculate(
                frc::ApplyDeadband(m_driveController.GetRawAxis(AXIS1_X), 0.05)) *
                Drivetrain::kMaxSpeed;

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed = -m_yspeedLimiter.Calculate(
                frc::ApplyDeadband(m_driveController.GetRawAxis(AXIS1_Y), 0.05)) *
                Drivetrain::kMaxSpeed;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  const auto rot = -m_rotLimiter.Calculate(
                frc::ApplyDeadband(m_driveController.GetRawAxis(AXIS2_X), 0.05)) *
                Drivetrain::kMaxAngularSpeed;

//  printf("JS x,y,r: %.1f, %.1f, %.2f\n\r", xSpeed, ySpeed, rot);

  m_drive.Drive(xSpeed, ySpeed, rot, fieldRelative);

  m_field.SetRobotPose(m_drive.GetPose());

  // Controls for the intake
  if(m_OpController.GetRawButton(6))
    intakeRoller.Set(1.0);
  else if(m_OpController.GetRawButton(5))
    intakeRoller.Set(-1.0);
  else
    intakeRoller.Set(0.0);

  // Control the hood
  intakeLiftMotorR.Set(0.5 * frc::ApplyDeadband(m_OpController.GetRawAxis(1), 0.05));

  // Control for ball storage 
  if(m_OpController.GetRawButton(8))
    ballStorageBelt.Set(0.75);
  else if(m_OpController.GetRawButton(7))
    ballStorageBelt.Set(-0.75);
  else
    ballStorageBelt.Set(0.0);

  // Control for shooter feeder - only while pressed
  if(m_OpController.GetRawButton(4))
    shooterFeeder.Set(-1);
  else
    shooterFeeder.Set(0.0);

/*
  // Control for shooter feeder - press once to turn on, press again to turn off
  if(m_OpController.GetRawButtonPressed(4))
    shooterOn = !shooterOn;
    if(shooterOn)
      shooterFeeder.Set(0.5);
    else
      shooterFeeder.Set(0.0);
*/
}

void Robot::SimulationPeriodic()
{
#if 0
//  std::cout << "simulation periodic is called" << std::endl;

  // run in a static circle
  m_driveSim.SetInputs(m_leftMotor.Get() * units::volt_t(12.0), m_rightMotor.Get() * units::volt_t(12.0));

  // Advance the model by 20 ms. Note that if you are running this
  // subsystem in a separate thread or have changed the nominal timestep
  // of TimedRobot, this value needs to match it.
  m_driveSim.Update(20_ms);

  // Update all of our sensors.
  m_leftEncoderSim.SetDistance(m_driveSim.GetLeftPosition().value());
  m_leftEncoderSim.SetRate(m_driveSim.GetLeftVelocity().value());
  m_rightEncoderSim.SetDistance(m_driveSim.GetRightPosition().value());
  m_rightEncoderSim.SetRate(m_driveSim.GetRightVelocity().value());
  m_gyroSim.SetAngle((double)(-m_driveSim.GetHeading().Radians() * (180 / M_PI)));
//  m_gyroSim.SetAngle(-m_driveSim.GetHeading().Degrees());
#endif
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif