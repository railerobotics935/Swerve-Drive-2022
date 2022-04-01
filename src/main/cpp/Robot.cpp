// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This is the main robot program for the frc 2022 robot competition


#include <iostream>
#include <ctre/phoenix/motorcontrol/InvertType.h>
#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "PixyFunctions/PixyStuff.h"
#include "Robot.h"

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

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// DEFINE AUTOS - position is based on the persepcive of the drive

#define RIGHT_AUTO
//#define CENTER_AUTO
//#define LEFT_AUTO
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

// Safty stop for right auto
#ifdef RIGHT_AUTO
#undef CENTER_AUTO
#undef LEFT_AUTO
#endif

// Safty stop for center auto
#ifdef CENTER_AUTO
#undef RIGHT_AUTO
#undef LEFT_AUTO
#endif

// Safty stop for left auto
#ifdef LEFT_AUTO
#undef CENTER_AUTO
#undef RIGHT_AUTO
#endif
//#define PRINT_BLOCK_DATA

void Robot::RobotInit()
{
  m_fieldRelative = false;

  // Initialize the 2D field widget
  frc::SmartDashboard::PutData("Field", &m_field);

  // Don't wait for serial data
  mxp_serial_port.SetTimeout(units::time::second_t(0));

  PixyStuffInit("datatable");

  // Initialize shuffleboard communications
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");
  nte_shooterPower = nt_table->GetEntry("Shooter/Power");

  // Setting the camera light off
  m_robotFunction.SetCameraLightOff();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  
  // Reset timer
  autoTimer.Reset();
  autoTimer.Start();
  shooterTimer.Reset();
  shooterTimer.Start();
}

void Robot::AutonomousPeriodic() {

  // Shoot ball
  if(autoTimer.Get() < (units::second_t) 3)
  {
    m_robotFunction.SetShooterTiltPos(390);
    m_robotFunction.SetShooter(1.0);
    if(shooterTimer.Get() > (units::second_t) 1.5)
    {
      m_robotFunction.SetBallStorageBelt(0.75);
      m_robotFunction.SetShooterFeeder(1.0);
    }
  }
  else
  {
    m_robotFunction.SetShooter(0.0);
    m_robotFunction.SetBallStorageBelt(0.0);
    m_robotFunction.SetShooterFeeder(0.0);
  }

  if(autoTimer.Get() > (units::second_t) 3.0 && autoTimer.Get() < (units::second_t) 6.0)
  {
    m_drive.Drive((units::velocity::meters_per_second_t) 0.8, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, m_fieldRelative);
    m_robotFunction.SetIntakeRoller(0.0);
  }
  else
  {
    m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
    m_robotFunction.SetIntakeRoller(0.0);
  }
  //TODO: Find out direction to go for each pos, and write code to go out of area, and maybe even pick up a ball. 
  // AUTO MOVEMENT
#ifdef RIGHT_AUTO

#endif
#ifdef CENTER_AUTO

#endif
#ifdef LEFT_AUTO

#endif
}


void Robot::TeleopInit()
{
  nte_shooterPower.SetDouble(8.0);
  // Resetting Pose2d and the odometry
  frc::Pose2d m_Pose{(units::meter_t)0.0, (units::meter_t)0.0, frc::Rotation2d((units::radian_t)0.0)};
  m_drive.ResetOdometry(m_Pose);
  CreateYawPID();
  
  m_robotFunction.SetBallStorageBelt(0.0);
  m_robotFunction.SetShooterFeeder(0.0);
  m_robotFunction.SetShooter(0.0);
  m_robotFunction.SetCameraLightOn();
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
  if (m_driveController.GetRawButtonPressed(3)){
    m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, "none", AutomatedFunctions::FunctionCmd::kStartFunction);
    printf("Locating ball\n\r"); 
  }

  // Drive the robot, keep Red Button pressed to run an automated function
  if (m_driveController.GetRawButton(3))
  {
    if (frc::DriverStation::GetAlliance	() == frc::DriverStation::Alliance::kRed){
      m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, "red ball", AutomatedFunctions::FunctionCmd::kRunFunction);
      printf("Searching for Red Ball\n\r"); 
    }
    else if (frc::DriverStation::GetAlliance	() == frc::DriverStation::Alliance::kBlue){
      m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, "blue ball", AutomatedFunctions::FunctionCmd::kRunFunction);
      printf("Searching for blue Ball\n\r");     
      }
    else
      std::cout << "Please specify the Alliance\n\r";
  }
  else
  {
    DriveWithJoystick(m_fieldRelative);
  }

  // Reset the sequence state of the automated function
  if (m_driveController.GetRawButtonReleased(3)){
    m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, "none", AutomatedFunctions::FunctionCmd::kStopFunction);
    printf("End of automation\n\r"); 
  }

  // Update nte
  m_robotFunction.UpdateNTE();

  //------------------------------------------------------------
  // PIXY STUFF
  //------------------------------------------------------------

  // Get Teensy/Pixy information from RoboRIO expansion port
  n_serial_bytes_read = 0;
  n_serial_bytes_read = mxp_serial_port.Read(nmea_serial_buf, 1000);
  PixyProcessData(n_serial_bytes_read, nmea_serial_buf);
#ifdef PRINT_BLOCK_DATA		
  if (n_serial_bytes_read > 0)
  {
    nmea_serial_buf[n_serial_bytes_read] = 0;
    printf("Chassis data: %s\n", nmea_serial_buf);
  }
#endif
}

void Robot::DisabledInit()
{
  // brake to 0 speed and rotation
  m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);

  m_robotFunction.SetCameraLightOff();
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

// Shooter controles
if(m_OpController.GetRawButton(4))
  {
    // upper hub shooter
    shooterPower = nte_shooterPower.GetDouble(0.8);
    m_robotFunction.SetShooter(nte_shooterPower.GetDouble(0.8));
     if(m_OpController.GetRawButton(3))
    {
      m_robotFunction.SetBallStorageBelt(0.75);
      m_robotFunction.SetShooterFeeder(1.0);
    }
    else{
      m_robotFunction.SetBallStorageBelt(0.0);
      m_robotFunction.SetShooterFeeder(0.0);
    }
  }
  else
  {
    m_robotFunction.SetShooter(0.0);

    // Controls for the intake (button 8 is reverse)
    if(m_OpController.GetRawButton(8)) {
      m_robotFunction.SetIntakeRoller(-1.0);
      m_robotFunction.SetBallStorageBelt(-0.75);
      m_robotFunction.SetShooterFeeder(-1.0);
    }
    else if(m_OpController.GetRawButton(7)) {
      m_robotFunction.SetIntakeRoller(1.0);
      m_robotFunction.SetBallStorageBelt(0.75);
    }
    else {
      m_robotFunction.SetIntakeRoller(0.0);
      m_robotFunction.SetBallStorageBelt(0.0);
      m_robotFunction.SetShooterFeeder(0.0);
    }
  }

  // Automatic intake lift movement
  if (m_OpController.GetRawButtonPressed(1))
    m_robotFunction.ToggleIntakeSolinoid();

  // Control to reset the Tilt encoder
  if(m_OpController.GetRawButton(10))
    m_robotFunction.ResetTiltEncoder();
  else
    m_robotFunction.SetShooterTiltMotor(frc::ApplyDeadband(m_OpController.GetRawAxis(1)*0.5, 0.08)); 

  // climb button
  if(m_driveController.GetRawButton(10))
    m_robotFunction.SetClimbMotorPower(-1.0);
  else
    m_robotFunction.SetClimbMotorPower(0.0);
  
  // Seting tilt to pos
  if(m_OpController.GetRawButton(5))
    m_robotFunction.SetShooterTiltPos(300);
  if(m_OpController.GetRawButton(6))
    m_robotFunction.SetShooterTiltPos(400);

  // Saftey stops
  m_robotFunction.SafetyShooterStop();
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