#include <units/length.h>
#include <frc2/command/CommandBase.h>
#include <frc/filter/SlewRateLimiter.h>
#include <functional>
#include <frc/kinematics/ChassisSpeeds.h>
#include <wpi/array.h>

#include "SwerveSubsystem.h"
#include "Constants.h"

// TODO: I am not **completely** sure that the function overloads are correct (I've been following a really weird style).
// If it seems like functions are NOT getting called, check...
class SwerveJoystickCmd : public frc2::CommandBase {
private:
    SwerveSubsystem* swerveSubsystem;

    std::function<units::meters_per_second_t()> xSpdFunction, ySpdFunction;
    std::function<units::radians_per_second_t()> turningSpdFunction;

    std::function<bool()> fieldOrientedFunction;

    frc::SlewRateLimiter<units::meters_per_second_t> xLimiter, yLimiter;
    frc::SlewRateLimiter<units::radians_per_second_t> turningLimiter;

public:
    SwerveJoystickCmd(SwerveSubsystem* swerveSubsystem,
        std::function<units::meters_per_second_t()> xSpdFunction, std::function<units::meters_per_second_t()> ySpdFunction,
        std::function<units::radians_per_second_t()>, std::function<bool()> fieldOrientedFunction);

    void Initialize();

    void Execute();

    void End(bool interrupted);

    bool IsFinished();
};