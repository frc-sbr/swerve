#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/RobotDriveBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/RobotController.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "Constants.h"
#include "SwerveModule.h"


SwerveModule::SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
    int absoluteEncoderId, radian_t absoluteEncoderOffset, bool absoluteEncoderReversed) :
        driveMotor{driveMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
        turningMotor{turningMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
        driveEncoder{driveMotor.GetEncoder()},
        turningEncoder{turningMotor.GetEncoder()},
        absoluteEncoder{absoluteEncoderId},
        absoluteEncoderReversed{absoluteEncoderReversed},
        absoluteEncoderOffsetRad{absoluteEncoderOffset},
        turningPidController{Constants::kPTurning, 0, 0}
{
    driveMotor.SetInverted(driveMotorReversed);
    turningMotor.SetInverted(turningMotorReversed);

    driveEncoder.SetPositionConversionFactor(Constants::kDriveEncoderRot2Meter.value());
    driveEncoder.SetVelocityConversionFactor(Constants::kDriveEncoderRPM2MeterPerSec.value());
    turningEncoder.SetPositionConversionFactor(Constants::kTurningEncoderRot2Rad.value());
    turningEncoder.SetVelocityConversionFactor(Constants::kTurningEncoderRPM2RadPerSec.value());

    turningPidController.EnableContinuousInput(-M_PI, M_PI);
}


radian_t SwerveModule::GetDrivePosition() {
    return radian_t{driveEncoder.GetPosition()};
}
radian_t SwerveModule::GetTurningPosition() {
    return radian_t{turningEncoder.GetPosition()};
}


meters_per_second_t SwerveModule::GetDriveVelocity() {
    return meters_per_second_t{driveEncoder.GetVelocity()};
}
meters_per_second_t SwerveModule::GetTurningVelocity() {
    return meters_per_second_t{turningEncoder.GetVelocity()};
}


radian_t SwerveModule::GetAbsoluteEncoderRad() {
    radian_t angle = radian_t{absoluteEncoder.GetVoltage() / frc::RobotController::GetVoltage5V()};
    angle *= 2 * M_PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
}

void SwerveModule::ResetEncoders() {
    driveEncoder.SetPosition(0);
    turningEncoder.SetPosition(GetAbsoluteEncoderRad().value());
}


frc::SwerveModuleState SwerveModule::GetState() {
    return {GetDriveVelocity(), GetTurningPosition()};
}
void SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
    if (units::math::abs(state.speed) < 0.001_mps) {
        Stop();
        return;
    }
    state = frc::SwerveModuleState::Optimize(state, GetState().angle);
    driveMotor.Set(state.speed / DriveConstants::kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.Set(turningPidController.Calculate(GetTurningPosition().value(), state.angle.Radians().value()));
}


void SwerveModule::Stop() {
    driveMotor.Set(0);
    turningMotor.Set(0);
}