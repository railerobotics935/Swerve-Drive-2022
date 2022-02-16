// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc/ADIS16470_IMU.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/numbers>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "SwerveModule.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain();


  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::numbers::pi};  // 1/2 rotation per second

 private:
  nt::NetworkTableEntry nte_fl_angle;
  nt::NetworkTableEntry nte_fr_angle;
  nt::NetworkTableEntry nte_bl_angle;
  nt::NetworkTableEntry nte_br_angle;
  nt::NetworkTableEntry nte_fl_speed;
  nt::NetworkTableEntry nte_fr_speed;
  nt::NetworkTableEntry nte_bl_speed;
  nt::NetworkTableEntry nte_br_speed;
  
  frc::Translation2d m_frontLeftLocation{+0.324_m, +0.2675_m};
  frc::Translation2d m_frontRightLocation{+0.324_m, -0.2675_m};
  frc::Translation2d m_backLeftLocation{-0.324_m, +0.2675_m};
  frc::Translation2d m_backRightLocation{-0.324_m, -0.2675_m};

  SwerveModule m_frontLeft{4, 0, 0, 1, 0};
  SwerveModule m_frontRight{5, 1, 2, 3, 1};
  SwerveModule m_backLeft{6, 2, 4, 5, 2};
  SwerveModule m_backRight{7, 3, 6, 7, 3};

  frc::ADIS16470_IMU m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetAngle()};
};
