#include <frc/Joystick.h>
#include "SwerveSubsystem.h"
#include "SwerveJoystickCmd.h"
#include <units/length.h>
#include <functional>

#include "Constants.h"

using namespace units;

class RobotContainer {
private:
    SwerveSubsystem swerveSubsystem{};
    frc::Joystick driverJoystick{OIConstants::kDriverControllerPort};

public:
    RobotContainer() {
        swerveSubsystem.SetDefaultCommand(SwerveJoystickCmd{
            &swerveSubsystem,
            [&]() -> meters_per_second_t {return meters_per_second_t{-driverJoystick.GetRawAxis(OIConstants::kDriverYAxis)};},
            [&]() -> meters_per_second_t {return meters_per_second_t{driverJoystick.GetRawAxis(OIConstants::kDriverXAxis)};},
            [&]() -> radians_per_second_t {return radians_per_second_t{driverJoystick.GetRawAxis(OIConstants::kDriverRotAxis)};},
            [&]() -> bool {return !driverJoystick.GetRawButton(OIConstants::kDriverFieldOrientedButtonIdx);}});

        ConfigureButtonBindings();
    }

    void ConfigureButtonBindings() {
    }
};