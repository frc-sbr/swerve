#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/RobotDriveBase.h>

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
    // ?? no robot controller
}

void SwerveModule::ResetEncoders() {
    driveEncoder.SetPosition(0);
    turningEncoder.SetPosition(GetAbsoluteEncoderRad());
}