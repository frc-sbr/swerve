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
                           int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed) :
        absoluteEncoderOffsetRad{absoluteEncoderOffset},
        absoluteEncoderReversed{absoluteEncoderReversed},
        absoluteEncoder{absoluteEncoderId},

        driveMotor{driveMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
        turningMotor{turningMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},

        driveEncoder{driveMotor.GetEncoder()},
        turningEncoder{turningMotor.GetEncoder()},

        turningPidController{Constants::kPTurning, 0, 0} 
{
    driveMotor.SetInverted(driveMotorReversed);
    turningMotor.SetInverted(turningMotorReversed);

    driveEncoder.SetPositionConversionFactor(Constants::kDriveEncoderRot2Meter);
    driveEncoder.SetVelocityConversionFactor(Constants::kDriveEncoderRPM2MeterPerSec);
    turningEncoder.SetPositionConversionFactor(Constants::kTurningEncoderRot2Rad);
    turningEncoder.SetVelocityConversionFactor(Constants::kTurningEncoderRPM2RadPerSec);

    turningPidController.EnableContinuousInput(-M_PI, M_PI);

    ResetEncoders();
}


double SwerveModule::GetDrivePosition() {
    return driveEncoder.GetPosition();
}
double SwerveModule::GetTurningPosition() {
    return turningEncoder.GetPosition();
}


double SwerveModule::GetDriveVelocity() {
    return driveEncoder.GetVelocity();
}
double SwerveModule::GetTurningVelocity() {
    return turningEncoder.GetVelocity();
}


double SwerveModule::GetAbsoluteEncoderRad() {
    double angle = absoluteEncoder.GetVoltage() / frc::RobotController::GetVoltage5V();
    angle *= 2 * M_PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
}

void SwerveModule::ResetEncoders() {
    driveEncoder.SetPosition(0);
    turningEncoder.SetPosition(GetAbsoluteEncoderRad());
}


frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{GetDriveVelocity()},
          units::radian_t{GetTurningPosition()}};
}
void SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
    if (units::math::abs(state.speed).value() < 0.001) {
        Stop();
        return;
    }
    state = frc::SwerveModuleState::Optimize(state, GetState().angle);
    driveMotor.Set(1); // i have not gotten around to fixing the constants yet
    turningMotor.Set(turningPidController.Calculate(GetTurningPosition(), state.angle.Radians().value()));
}

void SwerveModule::Stop() {
    driveMotor.Set(0);
    turningMotor.Set(0);
}