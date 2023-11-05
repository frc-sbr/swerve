#include <units/length.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>
#include <functional>
#include <frc/kinematics/ChassisSpeeds.h>
#include <wpi/array.h>

#include "SwerveSubsystem.h"
#include "Constants.h"

using namespace units;
using namespace units::length;

// TODO: I am not **completely** sure that the function overloads are correct (I've been following a really weird style).
// If it seems like functions are NOT getting called, check...
class SwerveJoystickCmd : public frc2::CommandHelper<frc2::CommandBase, SwerveJoystickCmd> {
private:
    SwerveSubsystem* swerveSubsystem;

    std::function<meters_per_second_t()> xSpdFunction, ySpdFunction;
    std::function<radians_per_second_t()> turningSpdFunction;

    std::function<bool()> fieldOrientedFunction;

    frc::SlewRateLimiter<meters_per_second> xLimiter, yLimiter;
    frc::SlewRateLimiter<radians_per_second> turningLimiter;

public:
    SwerveJoystickCmd(SwerveSubsystem* swerveSubsystem,
        std::function<meters_per_second_t()> xSpdFunction, std::function<meters_per_second_t()> ySpdFunction,
        std::function<radians_per_second_t()> turningSpdFunction, std::function<bool()> fieldOrientedFunction);

    void Execute();

    void End(bool interrupted);
};