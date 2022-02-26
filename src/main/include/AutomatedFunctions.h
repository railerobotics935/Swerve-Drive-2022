//------------------------------------------------------------
// Collection of Automated Functions for 2022 FRC Robot
//------------------------------------------------------------

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/MathUtil.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/numbers>

#include "Drivetrain.h"

#define FRONT_CAM_MAX_OBJECTS 16


class AutomatedFunctions
{
public:
  AutomatedFunctions();

  void DriveClockWiseSemiCircleAroundIntake(Drivetrain &m_drive);
  void LocateAndLoadBall(Drivetrain &m_drive);

  static constexpr units::meters_per_second_t kMaxAutoSpeed = 1.0_mps;  // 1 meters per second
  static constexpr units::radians_per_second_t kMaxAutoRotation{1 * wpi::numbers::pi};  // 1/2 rotations per second

private:
  // Declaring all of the network table entrys
  nt::NetworkTableEntry nte_front_cam_object_label[FRONT_CAM_MAX_OBJECTS];
  nt::NetworkTableEntry nte_front_cam_object_status[FRONT_CAM_MAX_OBJECTS];
  nt::NetworkTableEntry nte_front_cam_object_location[FRONT_CAM_MAX_OBJECTS];

  nt::NetworkTableEntry nte_front_cam_target_angle;
  nt::NetworkTableEntry nte_front_cam_target_distance;

  nt::NetworkTableEntry nte_front_cam_target_angular_v;
  nt::NetworkTableEntry nte_front_cam_target_linear_v;

  nt::NetworkTableEntry nte_front_cam_pose_angular_v;
  nt::NetworkTableEntry nte_front_cam_pose_linear_v;

  // State variables for tracking objects
  uint8_t prev_nearest_ball_id;
  frc::Rotation2d prev_front_cam_target_angle;
  double prev_front_cam_target_distance;
  frc::Rotation2d prev_front_cam_pose_angle;
  double prev_front_cam_pose_x;
  double prev_front_cam_pose_y;
};