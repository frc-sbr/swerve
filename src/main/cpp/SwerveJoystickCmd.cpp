#include <units/length.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>
#include <functional>
#include <frc/kinematics/ChassisSpeeds.h>
#include <wpi/array.h>

#include "subsystems/SwerveSubsystem.h"
#include "SwerveJoystickCmd.h"
#include "Constants.h"

using namespace units;
using namespace units::length;

SwerveJoystickCmd::SwerveJoystickCmd(SwerveSubsystem* swerveSubsystem,
    std::function<meters_per_second_t()> xSpdFunction, std::function<meters_per_second_t()> ySpdFunction,
    std::function<radians_per_second_t()> turningSpdFunction, std::function<bool()> fieldOrientedFunction) : 
        swerveSubsystem{swerveSubsystem},

        xSpdFunction{xSpdFunction},
        ySpdFunction{ySpdFunction},
        turningSpdFunction{turningSpdFunction},
        fieldOrientedFunction{fieldOrientedFunction},

        xLimiter{DriveConstants::kTeleDriveMaxAccelerationUnitsPerSecond},
        yLimiter{DriveConstants::kTeleDriveMaxAccelerationUnitsPerSecond},
        turningLimiter{DriveConstants::kTeleDriveMaxAngularAccelerationUnitsPerSecond}
{
    AddRequirements(swerveSubsystem);
}

void SwerveJoystickCmd::Execute() {
    meters_per_second_t xSpeed  = xSpdFunction();
    meters_per_second_t ySpeed  = ySpdFunction();
    radians_per_second_t turningSpeed  = turningSpdFunction();

    xSpeed = (math::abs(xSpeed) > OIConstants::kDeadband)
        ? xSpeed : 0.0_mps;
    ySpeed = (math::abs(xSpeed) > OIConstants::kDeadband)
        ? ySpeed : 0.0_mps;
    turningSpeed = (math::abs(turningSpeed) > radians_per_second_t{OIConstants::kDeadband.value()})
        ? turningSpeed : 0.0_rad_per_s;

    xSpeed = xLimiter.Calculate(xSpeed) * (DriveConstants::kTeleDriveMaxSpeedMetersPerSecond.value());
    ySpeed = yLimiter.Calculate(ySpeed) * (DriveConstants::kTeleDriveMaxSpeedMetersPerSecond.value());
    turningSpeed = turningLimiter.Calculate(turningSpeed) * (DriveConstants::kTeleDriveMaxAngularSpeedRadiansPerSecond.value());

    swerveSubsystem->Drive(xSpeed, ySpeed, turningSpeed, fieldOrientedFunction());
}

void SwerveJoystickCmd::End(bool interrupted) {
    swerveSubsystem->StopModules();
}