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
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Encoder.h>

#include "Drivetrain.h"

#include "Drivetrain.h"
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
  bool m_fieldRelative;
  AutomatedFunctions m_Tricks;
  bool shooterOn;

  bool intakeDown = true;
  int targetIntakePos = 0;

  // Declaring Network Table Entrys
  nt::NetworkTableEntry nte_intakeLiftEncoderValue;

  // Declaring Motorcontrolers
  WPI_VictorSPX intakeRoller{8};
  WPI_VictorSPX intakeLiftMotorR{9};
  WPI_VictorSPX intakeLiftMotorL{10};
  rev::CANSparkMax ballStorageBelt{11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shooterFeeder{12, rev::CANSparkMax::MotorType::kBrushless};
  
  // Declaring spark max encoders
  rev::SparkMaxRelativeEncoder ballStorageEncoder{ballStorageBelt.GetEncoder()};
  rev::SparkMaxRelativeEncoder shooterFeederEncoder{shooterFeeder.GetEncoder()};

  // Declaring encoders for intake motor. 
  //Gear Ratio for the motor is 188:1 and encoder revoluiton is 7
  frc::Encoder intakeLiftEncoder{0, 1};

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