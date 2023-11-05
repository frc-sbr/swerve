#pragma once
#include <units/length.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <math.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>

using namespace units;
using namespace units::length;

class Constants {
public:
    static constexpr meter_t kWheelDiameterMeters = 4.0_m;
    static constexpr double kDriveMotorGearRatio = 1 / 6.75; // no idea where this number comes from (should be 1:6.75)
    static constexpr double kTurningMotorGearRatio = 1 / (150.0/7); // should be 1:150/7
    static constexpr meter_t kDriveEncoderRot2Meter = kDriveMotorGearRatio * M_PI * kWheelDiameterMeters;
    static constexpr radian_t kTurningEncoderRot2Rad = kTurningMotorGearRatio * radian_t{2 * M_PI};
    static constexpr meters_per_second_t kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60_s;
    static constexpr radians_per_second_t kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60_s;
    static constexpr double kPTurning = 0.5;
};

class DriveConstants {
public:
    static constexpr meter_t kTrackWidth = 30_in; // TODO: check with mechanical
    // Distance between right and left wheels
    static constexpr meter_t kWheelBase = 30_in; // TODO: check with mechanical
    // Distance between front and back wheels
    static const frc::SwerveDriveKinematics<4> kDriveKinematics; // TODO: add back stuff to subsystem.cpp

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

    static constexpr radian_t kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254_rad;
    static constexpr radian_t kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252_rad;
    static constexpr radian_t kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816_rad;
    static constexpr radian_t kBackRightDriveAbsoluteEncoderOffsetRad = -4.811_rad;

    static constexpr meters_per_second_t kPhysicalMaxSpeedMetersPerSecond = 5_mps;
    static constexpr radians_per_second_t kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * radian_t{2 * M_PI};

    static constexpr meters_per_second_squared_t kTeleDriveMaxAccelerationUnitsPerSecond = 3_mps_sq;
    static constexpr radians_per_second_squared_t kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3_rad_per_s_sq;
};

class OIConstants {
public:
    static constexpr int kDriverControllerPort = 0;

    static constexpr int kDriverYAxis = 1;
    static constexpr int kDriverXAxis = 0;
    static constexpr int kDriverRotAxis = 4;
    static constexpr int kDriverFieldOrientedButtonIdx = 1;

    static constexpr double kDeadband = 0.05;
};