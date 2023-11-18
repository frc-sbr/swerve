// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"

using namespace DriveConstants;
using namespace ModuleConstants;

SwerveSubsystem::SwerveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftDriveEncoderReversed,
                  kFrontLeftTurningEncoderReversed,
                  kFrontLeftDriveAbsoluteEncoderPort},

      m_backLeft{
          kBackLeftDriveMotorPort,       kBackLeftTurningMotorPort,
          kBackLeftDriveEncoderReversed, kBackLeftTurningEncoderReversed,
          kBackLeftDriveAbsoluteEncoderPort},

      m_frontRight{
          kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
          kFrontRightDriveEncoderReversed, kFrontRightTurningEncoderReversed,
          kFrontRightDriveAbsoluteEncoderPort},

      m_backRight{
          kBackRightDriveMotorPort,       kBackRightTurningMotorPort,
          kBackRightDriveEncoderReversed, kBackRightTurningEncoderReversed,
          kBackRightDriveAbsoluteEncoderPort}, 
      
      m_gyro{frc::SPI::Port::kMXP}
  {
    std::thread{[&](){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ZeroHeading();
    }};
  }

void SwerveSubsystem::Periodic() {
  m_odometry.Update(
    GetRotation2d(), 
    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void SwerveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
    frc::ChassisSpeeds chassisSpeeds;
    if (fieldRelative) {
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetRotation2d());
    } else {
        chassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, rot};
    }

    wpi::array<frc::SwerveModuleState, 4> moduleStates = DriveConstants::kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);
    SetModuleStates(moduleStates);
}

void SwerveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, kPhysicalMaxSpeedMetersPerSecond);
  
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_backLeft.SetDesiredState(desiredStates[2]);
  m_backRight.SetDesiredState(desiredStates[3]);
}

void SwerveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_backLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_backRight.ResetEncoders();
}

units::degree_t SwerveSubsystem::GetHeading() const {
  return units::degree_t{fmod(m_gyro.GetAngle(), 360)}; 
}

void SwerveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

frc::Rotation2d SwerveSubsystem::GetRotation2d() {
    return frc::Rotation2d{GetHeading()};
}

frc::Pose2d SwerveSubsystem::GetPose(){
  return m_odometry.GetPose();
}

void SwerveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
    GetRotation2d(), 
    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()}, 
    pose);
}

void SwerveSubsystem::StopModules() {
    m_frontLeft.Stop();
    m_frontRight.Stop();
    m_backLeft.Stop();
    m_backRight.Stop();
}