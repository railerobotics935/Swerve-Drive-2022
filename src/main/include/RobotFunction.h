//------------------------------------------------------------
// Collection of Robot Functions for 2022 FRC Robot
//------------------------------------------------------------

#pragma once

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Encoder.h>


#include "ctre/phoenix.h"


class RobotFunction
{
public:
  RobotFunction();
  
  void SetIntakeRoller(double power);
  void SetBallStorageBelt(double power);
  void SetShooterFeeder(double power);
  void SetIntakeLift(bool intakeDown);
  void UpdateNTE();

private:

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

  // Declaring Network Table Entrys
  nt::NetworkTableEntry nte_intakeLiftEncoderValue;
};