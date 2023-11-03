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

class DriveConstants {
private:
public:
    static constexpr double kTrackWidth = convert<units::inches, units::meters>(30); // TODO: check with mechanical
    // Distance between right and left wheels
    static constexpr double kWheelBase = convert<units::inches, units::meters>(30); // TODO: check with mechanical
    // Distance between front and back wheels
    static const frc::SwerveDriveKinematics<4> kDriveKinematics;

    // TODO: the following data is ALL WRONG. I COPY PASTED THIS FROM A YOUTUBE VIDEO. I AM NOT LOOKING FORWARD TO FIGURING ALL OF THIS OUT.
    static constexpr int kFrontLeftDriveMotorPort = 8;
    static constexpr int kBackLeftDriveMotorPort = 2;
    static constexpr int kFrontRightDriveMotorPort = 6;
    static constexpr int kBackRightDriveMotorPort = 4;

    static constexpr int kFrontLeftTurningMotorPort = 7;
    static constexpr int kBackLeftTurningMotorPort = 1;
    static constexpr int kFrontRightTurningMotorPort = 5;
    static constexpr int kBackRightTurningMotorPort = 3;

    static constexpr bool kFrontLeftTurningEncoderReversed = true;
    static constexpr bool kBackLeftTurningEncoderReversed = true;
    static constexpr bool kFrontRightTurningEncoderReversed = true;
    static constexpr bool kBackRightTurningEncoderReversed = true;

    static constexpr bool kFrontLeftDriveEncoderReversed = true;
    static constexpr bool kBackLeftDriveEncoderReversed = true;
    static constexpr bool kFrontRightDriveEncoderReversed = false;
    static constexpr bool kBackRightDriveEncoderReversed = false;

    static constexpr int kFrontLeftDriveAbsoluteEncoderPort = 0;
    static constexpr int kBackLeftDriveAbsoluteEncoderPort = 2;
    static constexpr int kFrontRightDriveAbsoluteEncoderPort = 1;
    static constexpr int kBackRightDriveAbsoluteEncoderPort = 3;

    static constexpr bool kFrontLeftDriveAbsoluteEncoderReversed = false;
    static constexpr bool kBackLeftDriveAbsoluteEncoderReversed = false;
    static constexpr bool kFrontRightDriveAbsoluteEncoderReversed = false;
    static constexpr bool kBackRightDriveAbsoluteEncoderReversed = false;

    static constexpr double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
    static constexpr double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
    static constexpr double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
    static constexpr double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;

    static constexpr double kPhysicalMaxSpeedMetersPerSecond = 5;
    static constexpr double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * M_PI;

    static constexpr double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    static constexpr double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    static constexpr double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    static constexpr double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
};