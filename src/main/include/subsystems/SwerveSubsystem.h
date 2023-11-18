// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <frc/Encoder.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "SwerveModule.h"

class SwerveSubsystem : public frc2::SubsystemBase {
 public:
  SwerveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  void Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative);
  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading() const;

  frc::Rotation2d GetRotation2d();

  frc::Pose2d GetPose();

  void StopModules();

  void ResetOdometry(frc::Pose2d pose);

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  SwerveModule m_frontLeft;
  SwerveModule m_backLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backRight;

  frc::SwerveDriveOdometry<4> m_odometry{DriveConstants::kDriveKinematics, frc::Rotation2d{0_deg},
    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
    m_backLeft.GetPosition(), m_backRight.GetPosition()},
    frc::Pose2d{0_m, 0_m, 0_rad}};

  // The gyro sensor
  AHRS m_gyro;
};
