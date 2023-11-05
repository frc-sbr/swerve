#include <units/length.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/SPI.h>
#include <thread>
#include <chrono>

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
    std::thread{[&](){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ZeroHeading();
    }};
}

void SwerveSubsystem::ZeroHeading() {
    gyro.Reset();
}
degree_t SwerveSubsystem::GetHeading() {
    return degree_t{fmod(gyro.GetAngle(), 360)}; //TODO: bad? idk no clue what fmod vs % distinction is
}

frc::Rotation2d SwerveSubsystem::GetRotation2d() {
    return frc::Rotation2d{GetHeading()};
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
    DriveConstants::kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kPhysicalMaxSpeedMetersPerSecond);

    frontLeft.SetDesiredState(desiredStates[0]);
    frontRight.SetDesiredState(desiredStates[1]);
    backLeft.SetDesiredState(desiredStates[2]);
    backRight.SetDesiredState(desiredStates[3]);
}

const frc::SwerveDriveKinematics<4> DriveConstants::kDriveKinematics{
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2}};