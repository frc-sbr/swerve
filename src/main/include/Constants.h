#pragma once
#include <units/length.h>
#include <math.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>

using namespace units;
using namespace units::length;

class Constants {
private:
public:
    static constexpr double kWheelDiameterMeters = convert<units::inches, units::meters>(4.0);
    static constexpr double kDriveMotorGearRatio = 1 / 6.75; // no idea where this number comes from (should be 1:6.75)
    static constexpr double kTurningMotorGearRatio = 1 / (150.0/7); // should be 1:150/7
    static constexpr double kDriveEncoderRot2Meter = kDriveMotorGearRatio * M_PI * kWheelDiameterMeters;
    static constexpr double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * M_PI;
    static constexpr double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    static constexpr double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    static constexpr double kPTurning = 0.5;
};