//------------------------------------------------------------
// Collection of Robot Functions for 2022 FRC Robot
//------------------------------------------------------------

#pragma once

#include <iostream>
#include <string>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <frc/Encoder.h>
#include <frc/I2C.h>
#include <rev/ColorSensorV3.h>
#include <frc/DigitalInput.h>

#include "ctre/phoenix.h"


class RobotFunction
{
public:
  RobotFunction();
  
  void SetIntakeRoller(double power);
  void SetBallStorageBelt(double power);
  void SetShooterFeeder(double power);
  void SetIntakeLift(bool intakeDown);
  void SetShooter(double power);
  void SetShooterAngle(double power);
  void SetTargetShooterAngle(double target);
  std::string GetSensorColor();
  double GetSensorProximity();
  void UpdateNTE();
  void ResetTiltEncoder();
  void SafetyShooterStop();

private:
  // Declaring Local variables
  frc::Color detectedColor;
  uint32_t proximity;
  std::string returnColor;


  // Declaring Motorcontrolers
  WPI_VictorSPX intakeRollerMotor{8};
  WPI_VictorSPX intakeLiftMotorR{9};
  WPI_VictorSPX intakeLiftMotorL{10};
  rev::CANSparkMax ballStorageBeltMotor{11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shooterFeederMotor{12, rev::CANSparkMax::MotorType::kBrushless};
  WPI_VictorSPX shooterMotor1{14};
  WPI_VictorSPX shooterMotor2{15};
  WPI_VictorSPX shooterMotor3{16};
  WPI_VictorSPX shooterAngleMotor{13};
  
  // Declaring spark max encoders
  rev::SparkMaxRelativeEncoder ballStorageEncoder{ballStorageBeltMotor.GetEncoder()};
  rev::SparkMaxRelativeEncoder shooterFeederEncoder{shooterFeederMotor.GetEncoder()};

  // Color sensor
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorSensor{i2cPort};

  // Declaring encoders for intake lift and shooter angle. 
  //Gear Ratio for the motor is 188:1 and encoder revoluiton is 7
  frc::Encoder intakeLiftEncoder{0, 1};
  frc::Encoder shooterTiltEncoder{2, 3, true};
  
  // Tilt limit switch
  frc::DigitalInput tiltSwitch{4};

  // Declaring Network Table Entrys
  nt::NetworkTableEntry nte_intakeLiftEncoderValue;
  nt::NetworkTableEntry nte_shooterAngleEncoderValue;
  nt::NetworkTableEntry nte_colorSensorRed;
  nt::NetworkTableEntry nte_colorSensorGreen;
  nt::NetworkTableEntry nte_colorSensorBlue;
  nt::NetworkTableEntry nte_colorSensorProximity;
};