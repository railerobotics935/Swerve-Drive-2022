#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include "Drivetrain.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationPeriodic() override;

private:
  frc::XboxController m_controller{0};
  Drivetrain m_drive;
  bool m_fieldRelative;

  // Declaring Motorcontrolers
  WPI_VictorSPX intakeMotor{8};

  // Slew rate limiters to make joystick inputs more gentle; 1/2 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
//  constexpr double kRamseteB = 2;
//  constexpr double kRamseteZeta = 0.7;

  // declare a 2D field widget object
  frc::Field2d m_field;

  nt::NetworkTableEntry nte_tracked_object_label[16];
  nt::NetworkTableEntry nte_tracked_object_status[16];
  nt::NetworkTableEntry nte_tracked_object_location[16];

  void DriveWithJoystick(bool fieldRelative);
};