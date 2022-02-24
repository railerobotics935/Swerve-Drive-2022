// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Robot.h"

#include <iostream>

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include "Drivetrain.h"

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

  // Initialize the 2D field widget
  frc::SmartDashboard::PutData("Field", &m_field);

  //----------------------------------------------------
  // Initialize network table entries for ball tracking
  //
  // The FrontCam will put detected objects in SmartDashboard under:
  //
  //    "FrontCam/Object[x]/Label"
  //    "FrontCam/Object[x]/Status" 
  //    "FrontCam/Object[x]/Location"
  //
  // where x is a number from 0 to 15 (track maximum 16 objects)
  //
  // Possible values for Label are: "red ball", "blue ball", "person", "robot"
  // Possible values for Status are: "TRACKED", "LOST"
  // Location is defined as an array with x,y,z location relative to camera in mm: [x,y,z]
  //----------------------------------------------------
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("SmartDashboard");

  char s_FrontCamTableEntryPath[32];
  for (uint8_t i=0; i<FRONT_CAM_MAX_OBJECTS; i++)
  {
    sprintf( s_FrontCamTableEntryPath, "FrontCam/Object[%d]/Label", i);
    nte_front_cam_object_label[i] = nt_table->GetEntry(s_FrontCamTableEntryPath);
    sprintf( s_FrontCamTableEntryPath, "FrontCam/Object[%d]/Status", i);
    nte_front_cam_object_status[i] = nt_table->GetEntry(s_FrontCamTableEntryPath);
    sprintf( s_FrontCamTableEntryPath, "FrontCam/Object[%d]/Location", i);
    nte_front_cam_object_location[i] = nt_table->GetEntry(s_FrontCamTableEntryPath);
  }

  intakeMotor.ConfigFactoryDefault();
/*
            ssd=sd.getSubTable(f"tracked_object_{t.id}")
            ssd.putString("label", label)
            ssd.putString("status", t.status.name)
            ssd.putNumberArray("x,y,z", [int(t.spatialCoordinates.x), int(t.spatialCoordinates.y), int(t.spatialCoordinates.z)])
*/
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
//  frc::Pose2d m_Pose{(units::meter_t)0.0, (units::meter_t)0.0, frc::Rotation2d((units::radian_t)0.0)};
//  m_drive.ResetOdometry(m_Pose);

  //----------------------------------
  // Test trajectory based autonomous
  //----------------------------------
#if 0
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);

  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(DriveConstants::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing backward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass the config
      config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
#endif
}

void Robot::AutonomousPeriodic()
{
//  DriveWithJoystick(false);

  for (uint8_t i=0; i<FRONT_CAM_MAX_OBJECTS; i++)
  {
    if ((nte_front_cam_object_label[i].GetString("none") == "red ball") &&
        (nte_front_cam_object_status[i].GetString("none") == "TRACKED"))
    {
      std::vector<double> arr = nte_front_cam_object_location[i].GetDoubleArray(std::vector<double>());
      double m_x = arr[0];
      double m_y = arr[1];
      double m_z = arr[2];

      std::cout << "First Red Ball at: " << m_x << ", " << m_y << ", " << m_z << std::endl;

      i = FRONT_CAM_MAX_OBJECTS;
    }
  }
}

void Robot::TeleopInit()
{
  // Reseting Pose2d and the odometry
  frc::Pose2d m_Pose{(units::meter_t)0.0, (units::meter_t)0.0, frc::Rotation2d((units::radian_t)0.0)};
  m_drive.ResetOdometry(m_Pose);
}

void Robot::TeleopPeriodic()
{
  // Switching between robot relative and field relative (blue)
  if (m_controller.GetRawButtonPressed(1))
  {
    m_fieldRelative = !m_fieldRelative;
    if (m_fieldRelative)
      std::cout << "Drive set to FIELD Relative\n\r";
    else
      std::cout << "Drive set to ROBOT Relative\n\r";
  }

  // Reset the gyro by pressing a button (green)
  if (m_controller.GetRawButtonPressed(2))
    m_drive.ResetGyro();

  // Drive the robot
  DriveWithJoystick(m_fieldRelative);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::DriveWithJoystick(bool fieldRelative)
{
  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed = -m_xspeedLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetRawAxis(AXIS1_X), 0.05)) *
                Drivetrain::kMaxSpeed;

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed = -m_yspeedLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetRawAxis(AXIS1_Y), 0.05)) *
                Drivetrain::kMaxSpeed;

  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  const auto rot = -m_rotLimiter.Calculate(
                frc::ApplyDeadband(m_controller.GetRawAxis(AXIS2_X), 0.05)) *
                Drivetrain::kMaxAngularSpeed;

//  printf("JS x,y,r: %.1f, %.1f, %.2f\n\r", xSpeed, ySpeed, rot);

  m_drive.Drive(xSpeed, ySpeed, rot, fieldRelative);

//  const frc::Pose2d m_pose = m_drive.UpdateOdometry();
//  m_field.SetRobotPose(m_pose);
  m_field.SetRobotPose(m_drive.UpdateOdometry());

  // Controls for the intake
  if(m_controller.GetRawButton(5))
    intakeMotor.Set(0.5);
  else if(m_controller.GetRawButton(4))
    intakeMotor.Set(-0.5);
  else
    intakeMotor.Set(0);

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