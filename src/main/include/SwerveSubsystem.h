#pragma once
#include <AHRS.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include "SwerveModule.h"
#include "Constants.h"

class SwerveSubsystem : public frc2::SubsystemBase {
public:
    SwerveSubsystem();

    void ZeroHeading();
    double GetHeading();

    frc::Rotation2d GetRotation2d();

    void Periodic();

    void StopModules();
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4>& desiredStates);


private:
    // TODO: FILL IN THESE CONSTRUCTORS
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    AHRS gyro;
    // frc::SwerveDriveOdometry<4> odometer; // TODO: I CANNOT GET THIS TO WORK
};