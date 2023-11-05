#include <frc/Joystick.h>
#include "SwerveSubsystem.h"
#include "SwerveJoystickCmd.h"
#include <units/length.h>
#include <functional>

#include "Constants.h"

class RobotContainer {
private:
    SwerveSubsystem swerveSubsystem{};
    frc::Joystick driverJoystick{OIConstants::kDriverControllerPort};

public:
    RobotContainer() {
        swerveSubsystem.SetDefaultCommand(SwerveJoystickCmd{
            &swerveSubsystem,
            [&]() -> units::meters_per_second_t {-driverJoystick.GetRawAxis(OIConstants::kDriverYAxis);},
            [&]() -> units::meters_per_second_t {driverJoystick.GetRawAxis(OIConstants::kDriverXAxis);},
            [&]() -> units::radians_per_second_t {driverJoystick.GetRawAxis(OIConstants::kDriverRotAxis);},
            [&]() -> bool {!driverJoystick.GetRawAxis(OIConstants::kDriverFieldOrientedButtonIdx);}});

        ConfigureButtonBindings();
    }

    void ConfigureButtonBindings() {
    }
};

int main() {
    std::function<int()> func = []() -> int {return 1;};
}