#include <frc/Joystick.h>
#include <units/length.h>
#include <functional>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

#include "Constants.h"
#include "subsystems/SwerveSubsystem.h"
#include "SwerveJoystickCmd.h"
#include "RobotContainer.h"

RobotContainer::RobotContainer() {
    swerveSubsystem.SetDefaultCommand(SwerveJoystickCmd{
        &swerveSubsystem,
        [&]() -> meters_per_second_t {return meters_per_second_t{-driverJoystick.GetRawAxis(OIConstants::kDriverYAxis)};},
        [&]() -> meters_per_second_t {return meters_per_second_t{driverJoystick.GetRawAxis(OIConstants::kDriverXAxis)};},
        [&]() -> radians_per_second_t {return radians_per_second_t{driverJoystick.GetRawAxis(OIConstants::kDriverRotAxis)};},
        [&]() -> bool {return !driverJoystick.GetRawButton(OIConstants::kDriverFieldOrientedButtonIdx);}});

    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    frc::TrajectoryConfig config{
        AutoConstants::kAutoMaxSpeed,
        AutoConstants::kAutoMaxAcceleration
    };
    config.SetKinematics(DriveConstants::kDriveKinematics);

    frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d{0_m, 0_m, 0_rad},
        {frc::Translation2d{2.5_m, 0_m},
         frc::Translation2d{2.5_m, 2.5_m},
         frc::Translation2d{5_m, 2.5_m}},
        frc::Pose2d{5_m, 0_m, 0_rad},
        config
    ); // random trajectory just change it later

    frc2::PIDController xController{AutoConstants::kAutoPXController, 0, 0};
    frc2::PIDController yController{AutoConstants::kAutoPYController, 0, 0};
    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::kAutoPThetaController,
        0,
        0,
        AutoConstants::kAutoThetaControllerConstraints
    };
    thetaController.EnableContinuousInput(-units::radian_t(std::numbers::pi), units::radian_t(std::numbers::pi));

    frc2::SwerveControllerCommand<4> swerveControllerCommand{
        trajectory,
        [&]() {return swerveSubsystem.GetPose();},
        DriveConstants::kDriveKinematics,
        xController,
        yController,
        thetaController,
        [&] (auto desiredState) {return swerveSubsystem.SetModuleStates(desiredState);},
        {&swerveSubsystem}
    };

    return new frc2::SequentialCommandGroup(
        frc2::InstantCommand( [&]() {swerveSubsystem.ResetOdometry(trajectory.InitialPose());} ),
        std::move(swerveControllerCommand),
        frc2::InstantCommand( [&]() {swerveSubsystem.StopModules();} ) );
}