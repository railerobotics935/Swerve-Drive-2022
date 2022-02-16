// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

Drivetrain::Drivetrain(){
  m_gyro.Reset();

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");
  nte_fl_angle = nt_table->GetEntry("Serve Wheels/Front Left/Angle");
  nte_fr_angle = nt_table->GetEntry("Serve Wheels/Front Right/Angle");
  nte_bl_angle = nt_table->GetEntry("Serve Wheels/Back Left/Angle");
  nte_br_angle = nt_table->GetEntry("Serve Wheels/Back Right/Angle");
  nte_fl_speed = nt_table->GetEntry("Serve Wheels/Front Left/Speed");
  nte_fr_speed = nt_table->GetEntry("Serve Wheels/Front Right/Speed");
  nte_bl_speed = nt_table->GetEntry("Serve Wheels/Back Left/Speed");
  nte_br_speed = nt_table->GetEntry("Serve Wheels/Back Right/Speed");
  }

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetAngle())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  
  nte_fl_angle.SetDouble((double)fl.angle.Radians());
  nte_fr_angle.SetDouble((double)fr.angle.Radians());
  nte_bl_angle.SetDouble((double)bl.angle.Radians());
  nte_br_angle.SetDouble((double)br.angle.Radians());
  nte_fl_speed.SetDouble((double)fl.speed);
  nte_fr_speed.SetDouble((double)fr.speed);
  nte_bl_speed.SetDouble((double)bl.speed);
  nte_br_speed.SetDouble((double)br.speed);

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetAngle(), m_frontLeft.GetState(),
                    m_frontRight.GetState(), m_backLeft.GetState(),
                    m_backRight.GetState());
}
