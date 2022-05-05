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

// first ball shooter power reduction
#define FIRST_BALL_SHOOTER_POWER_REDUCTION 0.89

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// comment out for 2 Ball autonomous:
#define AUTONOMOUS_3_BALL

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

// Target center is reported by Teensy as -0.080 radians
#define TARGET_ANGLE_OFFSET -0.02
#define TARGET_ANGLE_DEADBAND 0.04

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
  nte_shooterPower = nt_table->GetEntry("Shooter/Input Power");

  nte_shooterPower.SetDouble(0.7);

  // Setting the camera light off
  m_robotFunction.SetCameraLightOn();

  // Default intake up
  m_robotFunction.SetIntakeUp();
}

void Robot::RobotPeriodic() 
{
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

void Robot::AutonomousInit()
{
  // Reset timer
  autoTimer.Reset();
  autoTimer.Start();
  debugTimer.Reset();
  debugTimer.Start();

  // Get Alliance Color
  if (frc::DriverStation::GetAlliance	() == frc::DriverStation::Alliance::kRed)
    allianceColor = AutomatedFunctions::AllianceColor::kRed;
  else if (frc::DriverStation::GetAlliance	() == frc::DriverStation::Alliance::kBlue)
    allianceColor = AutomatedFunctions::AllianceColor::kBlue;
  else
  {
    allianceColor = AutomatedFunctions::AllianceColor::kNone;
    std::cout << "Please specify the Alliance\n\r";
  }

  // Initialize state for autonomous
  m_drive.ResetOdometry(m_drive.GetPose());
  currentState = AutoStates::kMoveBack;
}

void Robot::AutonomousPeriodic()
{
  switch (currentState)
  {
  case AutoStates::kMoveBack:
    // drive back about 1 meter in 1 second
    m_drive.Drive((units::velocity::meters_per_second_t)1.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, false);

    // spin up shooter right away
    m_robotFunction.SetShooter(targetShooterPower * FIRST_BALL_SHOOTER_POWER_REDUCTION);

    if ((m_drive.GetPose().X() >= (units::length::meter_t)0.8 ) ||
        (autoTimer.Get() > (units::time::second_t)1.5))
    {
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, false);
      currentState = AutoStates::kShootBall;
      shooterTimer.Reset();
      shooterTimer.Start();
      printf("AUTO %.2f Shoot 1st Ball\n\r", debugTimer.Get());
    }
    break;
  
  case AutoStates::kShootBall:
    m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
    
    // Automatic Shooting, set shooter angle and power
    m_robotFunction.SetShooterTiltPos(targetShooterAngle);
    m_robotFunction.SetShooter(targetShooterPower * FIRST_BALL_SHOOTER_POWER_REDUCTION);

    // Align robot with target
    // when the robot is to the right of the target the angle increases (+)
    // when the robot is to the left of the target the angle decreases (-)
    if (targetAngleOffset > (TARGET_ANGLE_OFFSET + TARGET_ANGLE_DEADBAND))
    { // Rotate robot Left
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, AutomatedFunctions::kTargetingRotation, false);
    }
    else if (targetAngleOffset < (TARGET_ANGLE_OFFSET - TARGET_ANGLE_DEADBAND))
    { // Rotate robot Right
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, -AutomatedFunctions::kTargetingRotation, false);
    }
    else
    { // The robot is aligned with the target
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
    }

    // give it 1 more second to power up the shooter, then feed ball
    if (shooterTimer.Get() > (units::time::second_t)1.0)
    {
      m_robotFunction.SetBallStorageBelt(0.75);
      m_robotFunction.SetShooterFeeder(1.0);
    }
    
    // and after another 2 seconds stop the shooter
    if (shooterTimer.Get() > (units::time::second_t)3.0)
    {
      m_robotFunction.SetShooter(0.0);
      m_robotFunction.SetBallStorageBelt(0.0);
      m_robotFunction.SetShooterFeeder(0.0);
      m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, AutomatedFunctions::kNone, AutomatedFunctions::FunctionCmd::kStartFunction);
      autoTimer.Reset();
      autoTimer.Start();
      currentState = AutoStates::kRotateForSecondBall;
//      currentState = AutoStates::kFindSecondBall;
//      currentState = AutoStates::kAutonomousFinished;
      printf("AUTO %.2f Start Rotate for 2nd Ball\n\r", debugTimer.Get());
    }   
    break;

  case AutoStates::kRotateForSecondBall:
    m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, -AutomatedFunctions::kFindBallRotation, false);
    if ((m_drive.GetPose().Rotation().Radians() <= (units::angle::radian_t)-0.8) ||
        (autoTimer.Get() > (units::time::second_t)2.0))
    {
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
      m_Tricks.LoadBall(m_drive, m_robotFunction, true);
      autoTimer.Reset();
      autoTimer.Start();
      currentState = AutoStates::kLoadSecondBall;
      printf("AUTO %.2f Load 2nd Ball\n\r", debugTimer.Get());
    }
    break;

  case AutoStates::kLoadSecondBall:
    if (m_Tricks.LoadBall(m_drive, m_robotFunction, false) ||
        (autoTimer.Get() > (units::time::second_t)3.5))
    {
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
      autoTimer.Reset();
      autoTimer.Start();
      currentState = AutoStates::kRotateBackFromSecondBall;
      printf("AUTO %.2f Rotate Back from 2nd Ball\n\r", debugTimer.Get());
    }
    break;

  case AutoStates::kRotateBackFromSecondBall:
    m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, AutomatedFunctions::kFindBallRotation, false);
    if ((m_drive.GetPose().Rotation().Radians() >= (units::angle::radian_t)0.0) ||
        (autoTimer.Get() > (units::time::second_t)2.0))
    {
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
      currentState = AutoStates::kShootSecondBall;
      shooterTimer.Reset();
      shooterTimer.Start();
      printf("AUTO %.2f Shoot 2nd Ball\n\r", debugTimer.Get());
    }
    break;

  case AutoStates::kFindSecondBall:
    // give it maximum 4 seconds to find another ball
    if ((m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, allianceColor, AutomatedFunctions::FunctionCmd::kRunFunction)) ||
        ((autoTimer.Get() > (units::time::second_t)4.0)))
    {
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
      currentState = AutoStates::kShootSecondBall;
    }
    break;

  case AutoStates::kShootSecondBall:
    // Automatic Shooting, set shooter angle and power
    m_robotFunction.SetShooterTiltPos(targetShooterAngle);
    m_robotFunction.SetShooter(targetShooterPower * FIRST_BALL_SHOOTER_POWER_REDUCTION);

    // Align robot with target
    // when the robot is to the right of the target the angle increases (+)
    // when the robot is to the left of the target the angle decreases (-)
    if (targetAngleOffset > (TARGET_ANGLE_OFFSET + TARGET_ANGLE_DEADBAND))
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, AutomatedFunctions::kTargetingRotation, false);
    else if (targetAngleOffset < (TARGET_ANGLE_OFFSET - TARGET_ANGLE_DEADBAND))
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, -AutomatedFunctions::kTargetingRotation, false);
    else
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);

    if (shooterTimer.Get() > (units::time::second_t)1.5)
    { // give it 1.5 seconds to power up the shooter, then feed ball
      m_robotFunction.SetBallStorageBelt(0.75);
      m_robotFunction.SetShooterFeeder(1.0);
    }
    
    if (shooterTimer.Get() > (units::time::second_t)3.5)
    { // and after another 2 seconds stop the shooter
#ifndef AUTONOMOUS_3_BALL
      currentState = AutoStates::kAutonomousFinished;
      printf("AUTO %.2f Finished\n\r", debugTimer.Get());
#else
      m_robotFunction.SetShooter(0.0);
      m_robotFunction.SetBallStorageBelt(0.0);
      m_robotFunction.SetShooterFeeder(0.0);
      m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, AutomatedFunctions::kNone, AutomatedFunctions::FunctionCmd::kStartFunction);
      autoTimer.Reset();
      autoTimer.Start();
      currentState = AutoStates::kFindThirdBall;
      printf("AUTO %.2f Locate 3rd Ball\n\r", debugTimer.Get());
#endif
    }
    break;

  case AutoStates::kFindThirdBall:
    // give it maximum 4 seconds to find another ball
    if ((m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, allianceColor, AutomatedFunctions::FunctionCmd::kRunFunction)) ||
        ((autoTimer.Get() > (units::time::second_t)4.0)))
    {
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
      currentState = AutoStates::kAutonomousFinished;
    }
    break;

  case AutoStates::kAutonomousFinished:
    // stop all movement
    m_robotFunction.SetShooter(0.0);
    m_robotFunction.SetBallStorageBelt(0.0);
    m_robotFunction.SetShooterFeeder(0.0);
    m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
    break;
  }
}

void Robot::TeleopInit()
{
  // Resetting Pose2d and the odometry
  frc::Pose2d m_Pose{(units::meter_t)0.0, (units::meter_t)0.0, frc::Rotation2d((units::radian_t)0.0)};
  m_drive.ResetOdometry(m_Pose);
  
  m_robotFunction.SetBallStorageBelt(0.0);
  m_robotFunction.SetShooterFeeder(0.0);
  m_robotFunction.SetShooter(0.0);
  m_robotFunction.SetCameraLightOn();
  m_fieldRelative = true;
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
    m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, AutomatedFunctions::AllianceColor::kNone, AutomatedFunctions::FunctionCmd::kStartFunction);
    printf("Locating ball\n\r"); 
  }

  // Drive the robot, keep Red Button pressed to run an automated function
  if (m_driveController.GetRawButton(3))
  {
    if (frc::DriverStation::GetAlliance	() == frc::DriverStation::Alliance::kRed){
      allianceColor = AutomatedFunctions::AllianceColor::kRed;
      printf("Searching for Red Ball\n\r"); 
    }
    else if (frc::DriverStation::GetAlliance	() == frc::DriverStation::Alliance::kBlue){
      allianceColor = AutomatedFunctions::AllianceColor::kBlue;
      printf("Searching for blue Ball\n\r");     
    }
    else
    {
//      DriveWithJoystick(m_fieldRelative);
      DriveWithJoystick(true);
      std::cout << "Please specify the Alliance\n\r";
    }

    m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, allianceColor, AutomatedFunctions::FunctionCmd::kRunFunction);
  }
  else
  {
//    DriveWithJoystick(m_fieldRelative);
    DriveWithJoystick(true);
  }

  // Reset the sequence state of the automated function
  if (m_driveController.GetRawButtonReleased(3)){
    m_Tricks.LocateAndLoadBall(m_drive, m_robotFunction, AutomatedFunctions::AllianceColor::kNone, AutomatedFunctions::FunctionCmd::kStopFunction);
    printf("End of automation\n\r"); 
  }

  // Update nte
  m_robotFunction.UpdateNTE();
}

void Robot::DisabledInit()
{
  // brake to 0 speed and rotation
  m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);

  m_robotFunction.SetCameraLightOn();

  // Default intake up
  m_robotFunction.SetIntakeUp();
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
  if( m_OpController.GetRawButtonPressed(4))
  {
    ball_shoot_count = 0;
  }

  if (m_OpController.GetRawButton(4))
  {
    // Automatic Shooting
    // Set shooter angle
    m_robotFunction.SetShooterTiltPos(targetShooterAngle);

    // upper hub shooter
    if(ball_shoot_count == 0)
      m_robotFunction.SetShooter(targetShooterPower * FIRST_BALL_SHOOTER_POWER_REDUCTION);
    else
      m_robotFunction.SetShooter(targetShooterPower);

    // Align robot with target
    // when the robot is to the right of the target the angle increases (+)
    // when the robot is to the left of the target the angle decreases (-)
    if (targetAngleOffset > (TARGET_ANGLE_OFFSET + TARGET_ANGLE_DEADBAND))
    {
      // Rotate robot Left
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, AutomatedFunctions::kTargetingRotation, false);
    }
    else if (targetAngleOffset < (TARGET_ANGLE_OFFSET - TARGET_ANGLE_DEADBAND))
    {
      // Rotate robot Right
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, -AutomatedFunctions::kTargetingRotation, false);
    }
    else
    {
      // The robot is aligned with the target
      m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);
    }

    if( m_OpController.GetRawButtonReleased(3))
    {
      ball_shoot_count++;
    }

    if(m_OpController.GetRawButton(3))
    {
      m_robotFunction.SetBallStorageBelt(0.75);
      m_robotFunction.SetShooterFeeder(1.0);
    }
    else
    {
      m_robotFunction.SetBallStorageBelt(0.0);
      m_robotFunction.SetShooterFeeder(0.0);
    }
  }
  else
  {
    ball_shoot_count = 0;

    // Reset power to zero if we aren't 
    m_robotFunction.SetShooter(0.0);
    m_robotFunction.SetShooterFeeder(0.0);

    // Manual Shooter control
    if(m_OpController.GetRawButton(1))
    {
      m_robotFunction.SetShooter(0.7);

      if(m_OpController.GetRawButton(2))
      {
        m_robotFunction.SetBallStorageBelt(0.75);
        m_robotFunction.SetShooterFeeder(1.0);
      }
      else
      {
        m_robotFunction.SetBallStorageBelt(0.0);
        m_robotFunction.SetShooterFeeder(0.0);
      }
    }
    else
    {
      // reseting again
      m_robotFunction.SetShooter(0.0);
      m_robotFunction.SetBallStorageBelt(0.0);
      m_robotFunction.SetShooterFeeder(0.0);

      // All the other manual control that we do if we aren't shooting
      if ((double)intakeTimer.Get() > 0.5)
      {
        m_robotFunction.SetIntakeMotorPower(0.0);
        m_robotFunction.SetBallStorageBelt(0.0);
      }

      // Automatic intake lift movement
          // Automatic intake lift movement
      if (m_OpController.GetRawButton(8))
      {
        m_robotFunction.SetIntakeDown();
        m_robotFunction.SetIntakeMotorPower(-1.0);
        m_robotFunction.SetBallStorageBelt(-0.75);
        m_robotFunction.SetShooterFeeder(-1.0);
      }
      if( m_OpController.GetRawButtonReleased(8))
      {
        m_robotFunction.SetIntakeUp();
        m_robotFunction.SetIntakeMotorPower(0.0);
        m_robotFunction.SetBallStorageBelt(0.0);
        m_robotFunction.SetShooterFeeder(0.0);
      }

      if (m_OpController.GetRawButton(7))
      {
        m_robotFunction.SetIntakeDown();
        m_robotFunction.SetIntakeMotorPower(1.0);
        m_robotFunction.SetBallStorageBelt(0.75);
      }
      if( m_OpController.GetRawButtonReleased(7))
      {
        m_robotFunction.SetIntakeUp();
        intakeTimer.Reset();
        intakeTimer.Start();
      }

      // Control to reset the Tilt encoder
      if(m_OpController.GetRawButton(10))
        m_robotFunction.ResetTiltEncoder();
      else
        m_robotFunction.SetShooterTiltMotor(frc::ApplyDeadband(m_OpController.GetRawAxis(1)*0.5, 0.08)); 
    }
 }

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