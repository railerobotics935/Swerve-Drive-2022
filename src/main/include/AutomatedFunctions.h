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
#include <frc/Timer.h>

#include "Drivetrain.h"
#include "RobotFunction.h"

#define FRONT_CAM_MAX_OBJECTS 16


class AutomatedFunctions
{
public:
  enum FunctionCmd {kStartFunction, kRunFunction, kStopFunction};
  enum AllianceColor {kRed, kBlue, kNone};

  AutomatedFunctions();

  void DriveClockWiseSemiCircleAroundIntake(Drivetrain &m_drive);
  bool LocateAndLoadBall(Drivetrain &m_drive, RobotFunction &m_robotFunction, AllianceColor alliance, FunctionCmd command);
  bool LoadBall(Drivetrain &m_drive, RobotFunction &m_robotFunction, bool init_timer);

  static constexpr units::meters_per_second_t kMaxAutoSpeed = 1.0_mps;  // 1 meters per second
  static constexpr units::radians_per_second_t kMaxAutoRotation{1 * wpi::numbers::pi};  // 1/2 rotations per second
  static constexpr units::radians_per_second_t kFindBallRotation{0.25 * wpi::numbers::pi};  // 1/8 rotations per second
  static constexpr units::radians_per_second_t kTargetingRotation{0.125 * wpi::numbers::pi};  // 1/16 rotations per second

private:
  enum LocateAndLoadBallStep {kFindBall, kChaseBall, kLoadBall, kBallLoaded};
  LocateAndLoadBallStep m_LocateAndLoadBallStep;

  // Declare timer
  frc::Timer IntakeTimer{};

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

  void FindBall(Drivetrain &m_drive, std::string object_class);
  void ChaseBall(Drivetrain &m_drive, std::string object_class);
};
