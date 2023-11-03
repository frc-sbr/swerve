#include <units/length.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>

#include "Constants.h"

const frc::SwerveDriveKinematics<4> DriveConstants::kDriveKinematics{
    frc::Translation2d{meter_t{kWheelBase / 2}, -meter_t{kTrackWidth / 2}},
    frc::Translation2d{meter_t{kWheelBase / 2}, meter_t{kTrackWidth / 2}},
    frc::Translation2d{-meter_t{kWheelBase / 2}, -meter_t{kTrackWidth / 2}},
    frc::Translation2d{-meter_t{kWheelBase / 2}, meter_t{kTrackWidth / 2}}};