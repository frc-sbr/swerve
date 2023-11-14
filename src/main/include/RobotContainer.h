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

class RobotContainer {
private:
    SwerveSubsystem swerveSubsystem{};
    frc::Joystick driverJoystick{OIConstants::kDriverControllerPort};

public:
    RobotContainer();
    void ConfigureButtonBindings();
    frc2::Command* GetAutonomousCommand();
};