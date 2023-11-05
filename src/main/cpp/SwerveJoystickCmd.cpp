#include <units/length.h>
#include <frc2/command/CommandBase.h>
#include <frc/filter/SlewRateLimiter.h>
#include <functional>
#include <frc/kinematics/ChassisSpeeds.h>
#include <wpi/array.h>

#include "SwerveSubsystem.h"
#include "Constants.h"
#include "SwerveJoystickCmd.h"

SwerveJoystickCmd::SwerveJoystickCmd(SwerveSubsystem* swerveSubsystem,
    std::function<units::meters_per_second_t()> xSpdFunction, std::function<units::meters_per_second_t()> ySpdFunction,
    std::function<units::radians_per_second_t()>, std::function<bool()> fieldOrientedFunction) : 
        swerveSubsystem{swerveSubsystem},
        xSpdFunction{xSpdFunction},
        ySpdFunction{ySpdFunction},
        turningSpdFunction{turningSpdFunction},
        fieldOrientedFunction{fieldOrientedFunction},
        xLimiter{units::meters_per_second_t{DriveConstants::kTeleDriveMaxAccelerationUnitsPerSecond}},
        yLimiter{units::meters_per_second_t{DriveConstants::kTeleDriveMaxAccelerationUnitsPerSecond}},
        turningLimiter{units::radians_per_second_t{DriveConstants::kTeleDriveMaxAngularAccelerationUnitsPerSecond}}
{
    AddRequirements(swerveSubsystem);
}

void SwerveJoystickCmd::Initialize() {}

void SwerveJoystickCmd::Execute() {
    units::meters_per_second_t xSpeed  = xSpdFunction();
    units::meters_per_second_t ySpeed  = ySpdFunction();
    units::radians_per_second_t turningSpeed  = turningSpdFunction();

    // grrr add OIConstants
    xSpeed = units::math::abs(xSpeed) > units::meters_per_second_t{OIConstants::kDeadband} ? xSpeed : 0.0_mps;
    ySpeed = units::math::abs(xSpeed) > units::meters_per_second_t{OIConstants::kDeadband} ? ySpeed : 0.0_mps;
    turningSpeed = units::math::abs(turningSpeed) > units::radians_per_second_t{OIConstants::kDeadband} ? turningSpeed : 0.0_rad_per_s;

    xSpeed = xLimiter.Calculate(xSpeed) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.Calculate(ySpeed) * DriveConstants::kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.Calculate(turningSpeed) * DriveConstants::kTeleDriveMaxAngularSpeedRadiansPerSecond;

    frc::ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction()) {
        chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem->GetRotation2d());
    } else {
        chassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, turningSpeed};
    }

    wpi::array<frc::SwerveModuleState, 4> moduleStates = DriveConstants::kDriveKinematics.ToSwerveModuleStates(chassisSpeeds);
    swerveSubsystem->SetModuleStates(moduleStates);
}

void SwerveJoystickCmd::End(bool interrupted) {
    swerveSubsystem->StopModules();
}

bool SwerveJoystickCmd::IsFinished() {
    return false;
}