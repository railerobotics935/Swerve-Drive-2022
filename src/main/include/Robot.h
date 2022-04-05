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
#include <frc/SerialPort.h>

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

  double shooterPower = 0.0;

  bool intakeDown = false;
  bool firstTime = true;
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

  // Declare timers
  frc::Timer shooterTimer{};
  frc::Timer autoTimer{};
  frc::Timer intakeTimer{};

  // create a MXP serial Port and serial data buffer
  frc::SerialPort mxp_serial_port{115200, frc::SerialPort::kMXP, 8, frc::SerialPort::kParity_None, frc::SerialPort::kStopBits_One};
	char nmea_serial_buf[1000];
  int n_serial_bytes_read;

  // Networktable entries
  nt::NetworkTableEntry nte_shooterPower;

  void DriveWithJoystick(bool fieldRelative);
};