#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/RobotDriveBase.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkMaxRelativeEncoder.h>

#include "Constants.h"


class SwerveModule {
public:
    SwerveModule(int driveMotorId, int turningMotorId, bool driveMotorReversed, bool turningMotorReversed,
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
    }

    double GetDrivePosition();
    double GetTurningPosition();

    double GetDriveVelocity();
    double GetTurningVelocity();

    double GetAbsoluteEncoderRad();

    void ResetEncoders();


private:
    rev::CANSparkMax driveMotor;
    rev::CANSparkMax turningMotor;

    rev::SparkMaxRelativeEncoder driveEncoder;
    rev::SparkMaxRelativeEncoder turningEncoder;

    frc2::PIDController turningPidController;

    frc::AnalogInput absoluteEncoder;
    bool absoluteEncoderReversed;
    double absoluteEncoderOffsetRad;
};