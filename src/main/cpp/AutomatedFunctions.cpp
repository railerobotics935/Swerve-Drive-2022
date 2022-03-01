#include "AutomatedFunctions.h"

AutomatedFunctions::AutomatedFunctions()
{
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
  // Possible values for Status are: "TRACKED", "LOST", "REMOVED"
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

  nte_front_cam_target_angle = nt_table->GetEntry("FrontCam/Target/Angle");
  nte_front_cam_target_distance = nt_table->GetEntry("FrontCam/Target/Distance");
  nte_front_cam_target_angular_v = nt_table->GetEntry("FrontCam/Target/AngularV");
  nte_front_cam_target_linear_v = nt_table->GetEntry("FrontCam/Target/LinearV");
  nte_front_cam_pose_angular_v = nt_table->GetEntry("FrontCam/Pose/AngularV");
  nte_front_cam_pose_linear_v = nt_table->GetEntry("FrontCam/Pose/LinearV");

  // Reset Object Tracking state variables
  prev_nearest_ball_id = 0xFF;
  prev_front_cam_target_angle = frc::Rotation2d(0.0, 1.0);
  prev_front_cam_target_distance = 1.0E10;
  prev_front_cam_pose_angle = frc::Rotation2d(0.0, 1.0);
  prev_front_cam_pose_x = 0.0;
  prev_front_cam_pose_y = 0.0;
}

void AutomatedFunctions::DriveClockWiseSemiCircleAroundIntake(Drivetrain &m_drive)
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

//  const frc::Pose2d m_pose = m_drive.UpdateOdometry();
//  m_field.SetRobotPose(m_pose);
//    or use:
//  m_field.SetRobotPose(m_drive.GetPose());

}


void AutomatedFunctions::LocateAndLoadBall(Drivetrain &m_drive)
{
  uint8_t m_nearest_ball_id = 0xFF;
  double m_nearest_ball_x = 0.0;
  double m_nearest_ball_z = 1.0E10;
  uint8_t n_red_balls = 0;

  for (uint8_t i=0; i<FRONT_CAM_MAX_OBJECTS; i++)
  {
    if ((nte_front_cam_object_label[i].GetString("none") == "red ball") &&
        (nte_front_cam_object_status[i].GetString("none") == "TRACKED"))
    {
      n_red_balls++;

      std::vector<double> arr = nte_front_cam_object_location[i].GetDoubleArray(std::vector<double>());
      double m_x = arr[0];
//      double m_y = arr[1];
      double m_z = arr[2];

//      std::cout << "Red Ball " << n_red_balls << " at: " << m_x << ", " << m_y << ", " << m_z << std::endl;

      if (m_z < m_nearest_ball_z)
      {
        m_nearest_ball_id = i;
        m_nearest_ball_x = m_x;
        m_nearest_ball_z = m_z;
      }

      if (n_red_balls >= 3)
        i = FRONT_CAM_MAX_OBJECTS;
    }
  }

  if (n_red_balls >= 1)
  {
    frc::Rotation2d front_cam_target_angle = frc::Rotation2d(m_nearest_ball_z, m_nearest_ball_x);

    // update target location data on networktables
    nte_front_cam_target_angle.SetDouble((double)front_cam_target_angle.Radians());
    nte_front_cam_target_distance.SetDouble(m_nearest_ball_z);

    // try to predict where nearest ball is
    if (prev_nearest_ball_id == m_nearest_ball_id)
    {
      // check movement of ball in camera frame versus movement of the camera frame due to robot travel?
      // - robot travel can be obtained from m_drive.GetPose()
      // - object movement in camera frame from object tracking location
      // Note: object locations provided by the camera have a lot of noise

      // determine camera target angular velocity
      frc::Rotation2d target_delta_angle = front_cam_target_angle - prev_front_cam_target_angle;

      // determine camera pose angular velocity from odometry
      frc::Rotation2d pose_delta_angle = m_drive.GetPose().Rotation() - prev_front_cam_pose_angle;
      double pose_distance_squared = ((double)m_drive.GetPose().X() - prev_front_cam_pose_x) * ((double)m_drive.GetPose().X() - prev_front_cam_pose_x) +
                                      ((double)m_drive.GetPose().Y() - prev_front_cam_pose_y) * ((double)m_drive.GetPose().Y() - prev_front_cam_pose_y);
      double pose_distance = 0.0;
      if (pose_distance_squared > 0.0001)
        pose_distance = sqrt(pose_distance_squared);

      // update target velocity data on networktables
      nte_front_cam_target_angular_v.SetDouble((double)target_delta_angle.Radians() * 20.0);
      nte_front_cam_target_linear_v.SetDouble(((m_nearest_ball_z * 0.001) - prev_front_cam_target_distance) * 20.0);

      // update camera pose velocity data on networktables
      nte_front_cam_pose_angular_v.SetDouble((double)pose_delta_angle.Radians() * 20.0);
      nte_front_cam_pose_linear_v.SetDouble(pose_distance * 20.0);
    }

    // store current object ID and location for next control update
    prev_nearest_ball_id = m_nearest_ball_id;
    prev_front_cam_target_angle = front_cam_target_angle;
    prev_front_cam_target_distance = m_nearest_ball_z * 0.001;
    prev_front_cam_pose_angle = m_drive.GetPose().Rotation();
    prev_front_cam_pose_x = (double)m_drive.GetPose().X();
    prev_front_cam_pose_y = (double)m_drive.GetPose().Y();

    // turn and drive towards closest red_ball
    auto xSpeed = 0.0 * AutomatedFunctions::kMaxAutoSpeed;
    auto rot = 0.0 * AutomatedFunctions::kMaxAutoRotation;

    if (m_nearest_ball_z > 800.0)
      xSpeed = ((m_nearest_ball_z - 800.0) * 0.001) * AutomatedFunctions::kMaxAutoSpeed;

    if ((m_nearest_ball_x < -50.0) || (m_nearest_ball_x > 50.0))
      rot = -(m_nearest_ball_x * 0.0004) * AutomatedFunctions::kMaxAutoRotation;

    m_drive.Drive(xSpeed, (units::velocity::meters_per_second_t)0.0, rot, false);
  }
  else
  {
    m_drive.Drive((units::velocity::meters_per_second_t)0.0, (units::velocity::meters_per_second_t)0.0, (units::angular_velocity::radians_per_second_t)0.0, true);

    // reset previous object ID and location
    prev_nearest_ball_id = 0xFF;
    prev_front_cam_target_angle = frc::Rotation2d(0.0, 1.0);
    prev_front_cam_target_distance = 1.0E10;
    prev_front_cam_pose_angle = frc::Rotation2d(0.0, 1.0);
    prev_front_cam_pose_x = 0.0;
    prev_front_cam_pose_y = 0.0;
  }
}