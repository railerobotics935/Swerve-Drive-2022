#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
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


class Robot : public frc::TimedRobot {
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

  private:
   frc::XboxController m_controller{0};
   Drivetrain m_swerve;
 
   // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
   // to 1.
   frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
   frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
 
   void DriveWithJoystick(bool fieldRelative);
};