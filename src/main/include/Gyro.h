#pragma once

#include "Constants.h"

#include <units/angle.h>

#include <frc/geometry/Rotation2d.h>

#include <ctre/phoenix.h>

using namespace units;

class Gyro
{
public:
    Gyro();

    /// Returns the heading of the robot.
    /// \return the robot's heading in degrees, from -180 to 180
    double GetHeading();
    frc::Rotation2d GetHeadingAsRot2d() { return frc::Rotation2d(degree_t(GetHeading())); }
    double GetPitch() { 
        // auto pitch = m_pigeon.GetPitch();
        // if (pitch > 0.5)
        // {
        //     printf("Pitch %.3f degrees\n", pitch);
        // }

        // return pitch;
            return m_pigeon.GetPitch();
         }

    /// Zeroes the heading of the robot.
    void ZeroHeading();

    void SetHeading(double heading);

    /// Returns the turn rate of the robot.
    /// \return The turn rate of the robot, in degrees per second
    double GetTurnRate();
    
    ctre::phoenix::ErrorCode GetYawPitchRoll(double ypr[3]) const 
    {
        return m_pigeon.GetYawPitchRoll(ypr);
    }

protected:
    Pigeon2 m_pigeon;
};