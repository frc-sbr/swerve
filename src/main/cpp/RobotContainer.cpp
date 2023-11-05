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
            swerveSubsystem,
        });
    }

    void ConfigureButtonBindings() {
    }
};

int main() {
    std::function<int()> func = []() -> int {return 1;};
}