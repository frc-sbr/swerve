// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/RobotDriveBase.h>
#include <frc/RobotController.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
    int absoluteEncoderId, radian_t absoluteEncoderOffset, bool absoluteEncoderReversed) :
        m_driveMotor{driveMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
        m_turningMotor{turningMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},

        m_driveEncoder{m_driveMotor.GetEncoder()},
        m_turningEncoder{m_turningMotor.GetEncoder()},

        absoluteEncoder{absoluteEncoderId},
        absoluteEncoderReversed{absoluteEncoderReversed},
        absoluteEncoderOffsetRad{absoluteEncoderOffset} {

  m_driveEncoder.SetPositionConversionFactor(ModuleConstants::kDriveEncoderRot2Meter.value());
  m_driveEncoder.SetVelocityConversionFactor(ModuleConstants::kDriveEncoderRPM2MeterPerSec.value());
  m_turningEncoder.SetPositionConversionFactor(ModuleConstants::kTurningEncoderRot2Rad.value());
  m_turningEncoder.SetVelocityConversionFactor(ModuleConstants::kTurningEncoderRPM2RadPerSec.value());

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{m_turningEncoder.GetPosition()}};
}

radian_t SwerveModule::GetAbsoluteEncoderRad() {
    radian_t angle = radian_t{absoluteEncoder.GetVoltage() / frc::RobotController::GetVoltage5V()};
    angle *= 2 * M_PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  if (units::math::abs(referenceState.speed) < 0.001_mps) {
        Stop();
        return;
    }
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t{m_turningEncoder.GetPosition()});

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetPosition()}, state.angle.Radians());

  // Set the motor outputs.
  m_driveMotor.Set(driveOutput);
  m_turningMotor.Set(turnOutput);
}

void SwerveModule::ResetEncoders() {
  m_driveEncoder.SetPosition(0);
  m_turningEncoder.SetPosition(GetAbsoluteEncoderRad().value());
}

void SwerveModule::Stop() {
    m_driveMotor.Set(0);
    m_turningMotor.Set(0);
}
