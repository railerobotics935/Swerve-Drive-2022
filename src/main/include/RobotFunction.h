//------------------------------------------------------------
// Collection of Robot Functions for 2022 FRC Robot
//------------------------------------------------------------

#pragma once

#include <string>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Encoder.h>
#include <frc/I2C.h>
#include <rev/ColorSensorV3.h>

#include "ctre/phoenix.h"


class RobotFunction
{
public:
  RobotFunction();
  
  void SetIntakeRoller(double power);
  void SetBallStorageBelt(double power);
  void SetShooterFeeder(double power);
  void SetIntakeLift(bool intakeDown);
  void GetSensorColor();
  void UpdateNTE();

private:
  // Declaring Local variables
  frc::Color detectedColor;
  uint32_t proximity;

  // Declaring Motorcontrolers
  WPI_VictorSPX intakeRoller{8};
  WPI_VictorSPX intakeLiftMotorR{9};
  WPI_VictorSPX intakeLiftMotorL{10};
  rev::CANSparkMax ballStorageBelt{11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shooterFeeder{12, rev::CANSparkMax::MotorType::kBrushless};
  
  // Declaring spark max encoders
  rev::SparkMaxRelativeEncoder ballStorageEncoder{ballStorageBelt.GetEncoder()};
  rev::SparkMaxRelativeEncoder shooterFeederEncoder{shooterFeeder.GetEncoder()};

  // Color sensor
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorSensor{i2cPort};

  // Declaring encoders for intake motor. 
  //Gear Ratio for the motor is 188:1 and encoder revoluiton is 7
  frc::Encoder intakeLiftEncoder{0, 1};

  // Declaring Network Table Entrys
  nt::NetworkTableEntry nte_intakeLiftEncoderValue;
  nt::NetworkTableEntry nte_colorsensorRed;
  nt::NetworkTableEntry nte_colorsensorGreen;
  nt::NetworkTableEntry nte_colorsensorBlue;
  nt::NetworkTableEntry nte_colorsensorProximity;
};