#pragma once
#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/RobotDriveBase.h>

#include <frc/kinematics/SwerveModuleState.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include <units/length.h>

#include "Constants.h"


using namespace units;
using namespace units::length;


class SwerveModule {
public:
    SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
                 int absoluteEncoderId, radian_t absoluteEncoderOffset, bool absoluteEncoderReversed);

    radian_t GetDrivePosition();
    radian_t GetTurningPosition();

    meters_per_second_t GetDriveVelocity();
    meters_per_second_t GetTurningVelocity();

    radian_t GetAbsoluteEncoderRad();

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
    radian_t absoluteEncoderOffsetRad;

    frc2::PIDController turningPidController;
};