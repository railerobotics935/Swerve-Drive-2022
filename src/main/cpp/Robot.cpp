// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Robot.h"

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableInstance.h>

#include "Drivetrain.h"

   void Robot::RobotInit() {}
   void Robot::RobotPeriodic() {}
   void Robot::AutonomousInit() {}

   void Robot::AutonomousPeriodic() {
     DriveWithJoystick(false);
     m_swerve.UpdateOdometry();
  }

  void Robot::TeleopInit() {}

  void Robot::TeleopPeriodic() { DriveWithJoystick(true); }

  void Robot::DisabledInit() {}
  void Robot::DisabledPeriodic() {}
  void Robot::TestInit() {}
  void Robot::TestPeriodic() {}

  void Robot::DriveWithJoystick(bool fieldRelative) {  
    // Get the x speed. We are is because Xbox controllers returnnverting thi
     // negative values when we push forward.
     const auto xSpeed = -m_xspeedLimiter.Calculate(
                             frc::ApplyDeadband(m_controller.GetLeftY(), 0.02)) *
                         Drivetrain::kMaxSpeed;
 
     // Get the y speed or sideways/strafe speed. We are inverting this because
     // we want a positive value when we pull to the left. Xbox controllers
     // return positive values when you pull to the right by default.
     const auto ySpeed = -m_yspeedLimiter.Calculate(
                             frc::ApplyDeadband(m_controller.GetLeftX(), 0.02)) *
                         Drivetrain::kMaxSpeed;
 
     // Get the rate of angular rotation. We are inverting this because we want a
     // positive value when we pull to the left (remember, CCW is positive in
     // mathematics). Xbox controllers return positive values when you pull to
     // the right by default.
     const auto rot = -m_rotLimiter.Calculate(
                          frc::ApplyDeadband(m_controller.GetRawAxis(2), 0.02)) *
                      Drivetrain::kMaxAngularSpeed;
     printf("%f\n", m_controller.GetRawAxis(2));
 
     m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);}
 
 #ifndef RUNNING_FRC_TESTS
 int main() {
   return frc::StartRobot<Robot>();
 }
 #endif
 