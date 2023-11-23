// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/Encoder.h>
#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/drive/RobotDriveBase.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "Constants.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
                 int absoluteEncoderId);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

  void Stop();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;  

  rev::SparkMaxRelativeEncoder m_driveEncoder;
  rev::SparkMaxRelativeEncoder m_turningEncoder;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  units::radian_t GetAbsoluteEncoderRad();

  frc::AnalogInput absoluteEncoder;

  frc2::PIDController m_drivePIDController{
      ModuleConstants::kPDrive, 0, 0};
  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{ModuleConstants::kSDrive,
                                                                ModuleConstants::kVDrive * (1_s/1_m)};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPTurning,
      0.0,
      0.0,
      {DriveConstants::kMaxAngularSpeed, DriveConstants::kMaxAngularAcceleration}};
  frc::SimpleMotorFeedforward<units::radians> m_turningFeedforward{
      ModuleConstants::kSTurning,
      ModuleConstants::kVTurning * (1_s/1_rad)};
};
