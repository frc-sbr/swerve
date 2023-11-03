#include <units/length.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/SPI.h>

#include "SwerveSubsystem.h"
#include "Constants.h"

// TODO: THIS IS AWFUL
SwerveSubsystem::SwerveSubsystem() : 
    frontLeft{DriveConstants::kFrontLeftDriveMotorPort,
              DriveConstants::kFrontLeftTurningMotorPort,
              DriveConstants::kFrontLeftDriveEncoderReversed,
              DriveConstants::kFrontLeftTurningEncoderReversed,
              DriveConstants::kFrontLeftDriveAbsoluteEncoderPort,
              DriveConstants::kFrontLeftDriveAbsoluteEncoderOffsetRad,
              DriveConstants::kFrontLeftDriveAbsoluteEncoderReversed},

    frontRight{DriveConstants::kFrontRightDriveMotorPort,
            DriveConstants::kFrontRightTurningMotorPort,
            DriveConstants::kFrontRightDriveEncoderReversed,
            DriveConstants::kFrontRightTurningEncoderReversed,
            DriveConstants::kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants::kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants::kFrontRightDriveAbsoluteEncoderReversed},

    backLeft{DriveConstants::kBackLeftDriveMotorPort,
            DriveConstants::kBackLeftTurningMotorPort,
            DriveConstants::kBackLeftDriveEncoderReversed,
            DriveConstants::kBackLeftTurningEncoderReversed,
            DriveConstants::kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants::kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants::kBackLeftDriveAbsoluteEncoderReversed},

    backRight{DriveConstants::kBackRightDriveMotorPort,
            DriveConstants::kBackRightTurningMotorPort,
            DriveConstants::kBackRightDriveEncoderReversed,
            DriveConstants::kBackRightTurningEncoderReversed,
            DriveConstants::kBackRightDriveAbsoluteEncoderPort,
            DriveConstants::kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants::kBackRightDriveAbsoluteEncoderReversed},

    gyro{frc::SPI::Port::kMXP}
{

}

void SwerveSubsystem::ZeroHeading() {
    gyro.Reset();
}
double SwerveSubsystem::GetHeading() {
    return fmod(gyro.GetAngle(), 360);
}

frc::Rotation2d SwerveSubsystem::GetRotation2d() {
    return frc::Rotation2d{degree_t{GetHeading()}};
}


void SwerveSubsystem::Periodic() {
    // empty
}


void SwerveSubsystem::StopModules() {
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}
// https://github.com/Jagwires7443/Swerve/blob/master/src/main/cpp/subsystems/DriveSubsystem.cpp
void SwerveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4>& desiredStates) {
    DriveConstants::kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, meters_per_second_t{DriveConstants::kPhysicalMaxSpeedMetersPerSecond});
}

const frc::SwerveDriveKinematics<4> DriveConstants::kDriveKinematics{
    frc::Translation2d{meter_t{kWheelBase / 2}, -meter_t{kTrackWidth / 2}},
    frc::Translation2d{meter_t{kWheelBase / 2}, meter_t{kTrackWidth / 2}},
    frc::Translation2d{-meter_t{kWheelBase / 2}, -meter_t{kTrackWidth / 2}},
    frc::Translation2d{-meter_t{kWheelBase / 2}, meter_t{kTrackWidth / 2}}};