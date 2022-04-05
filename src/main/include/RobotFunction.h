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
#include <frc/Servo.h>
#include <frc/Relay.h>
#include <frc/Solenoid.h>

#include "ctre/phoenix.h"



class RobotFunction
{
public:
  RobotFunction();
  void SetIntakeUp();
  void SetIntakeDown();
  void SetIntakeMotorPower(double power);
  void SetBallStorageBelt(double power);
  void SetShooterFeeder(double power);
  void SetShooter(double power);
  void SetShooterTiltMotor(double power);
  void SetShooterTiltPos(double pos);
  std::string GetSensorColor();
  double GetSensorProximity();
  void UpdateNTE();
  void ResetTiltEncoder();
  void SafetyShooterStop(); 
  void TestServo(double angle);
  void SetClimbMotorPower(double power);
  void SetCameraLightOn();
  void SetCameraLightOff();


private:
  // Declaring Local variables
  frc::Color detectedColor;
  uint32_t proximity;
  std::string returnColor;
  double encoderStartingConfig = 0.0;

  // Declaring Motorcontrolers
  rev::CANSparkMax ballStorageBeltMotor{11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shooterFeederMotor{12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax climbMotor{17, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax intakeMotor{8, rev::CANSparkMax::MotorType::kBrushless};
  WPI_VictorSPX shooterMotor1{15};
  WPI_VictorSPX shooterMotor2{16};
  WPI_VictorSPX shooterTiltMotor{13};
  
  // Declaring spark max encoders
  rev::SparkMaxRelativeEncoder ballStorageEncoder{ballStorageBeltMotor.GetEncoder()};
  rev::SparkMaxRelativeEncoder shooterFeederEncoder{shooterFeederMotor.GetEncoder()};

  // Pneumatics Solenoids
  frc::Solenoid intakeSolenoid{frc::PneumaticsModuleType::CTREPCM, 0};

  // Color sensor
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 colorSensor{i2cPort};

  // Declaring encoders for intake lift and shooter angle. 
  //Gear Ratio for the motor is 188:1 and encoder revoluiton is 7
  frc::Encoder shooterTiltEncoder{2, 3, true};
  
  // Tilt limit switch
  frc::DigitalInput tiltSwitch{4};

  // Climber servo
  frc::Servo ClimbStopServo{0};

  // Spike for pixy light
  frc::Relay lightSpike{1, frc::Relay::Direction::kForwardOnly};

  // Declaring Network Table Entrys
  nt::NetworkTableEntry nte_shooterAngleEncoderValue;
  nt::NetworkTableEntry nte_colorSensorRed;
  nt::NetworkTableEntry nte_colorSensorGreen;
  nt::NetworkTableEntry nte_colorSensorBlue;
  nt::NetworkTableEntry nte_colorSensorProximity;
};