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
#include "RobotFunction.h"
#include "AutomatedFunctions.h"

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
  frc::XboxController m_driveController{0};
  frc::XboxController m_OpController{1};
  Drivetrain m_drive;
  RobotFunction m_robotFunction;
  AutomatedFunctions m_Tricks;
  bool m_fieldRelative;
  bool shooterOn;

  bool intakeDown = true;
  int targetIntakePos = 0;

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

  void DriveWithJoystick(bool fieldRelative);
};