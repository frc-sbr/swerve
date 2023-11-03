#pragma once
#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/RobotDriveBase.h>

#include <frc/kinematics/SwerveModuleState.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "Constants.h"


class SwerveModule {
public:
    SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
                 int absoluteEncoderId, double absoluteEncoderOffset, bool absoluteEncoderReversed);

    double GetDrivePosition();
    double GetTurningPosition();

    double GetDriveVelocity();
    double GetTurningVelocity();

    double GetAbsoluteEncoderRad();

    void ResetEncoders();

    frc::SwerveModuleState GetState();
    void SetDesiredState(frc::SwerveModuleState state);

    void Stop();


private:
    rev::CANSparkMax driveMotor;
    rev::CANSparkMax turningMotor;

    rev::SparkMaxRelativeEncoder driveEncoder;
    rev::SparkMaxRelativeEncoder turningEncoder;

    frc::AnalogInput absoluteEncoder;
    bool absoluteEncoderReversed;
    double absoluteEncoderOffsetRad;

    frc2::PIDController turningPidController;
};