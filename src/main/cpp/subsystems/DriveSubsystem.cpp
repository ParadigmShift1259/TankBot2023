#include "subsystems/DriveSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

DriveSubsystem::DriveSubsystem()
    : m_leftmcgroup(m_leftLeader, m_leftFollowerA, m_leftFollowerB)
    , m_rightmcgroup(m_rightLeader, m_rightFollowerA, m_rightFollowerB)
    , m_drive(m_leftmcgroup,m_rightmcgroup)
    , m_odometry(frc::Rotation2d(), 0_m, 0_m)
    , m_gyro()
{
    m_rightmcgroup.SetInverted(true);
}


void DriveSubsystem::Drive(double xSpeed, double zRotation, bool squareInputs /*= true*/)
{
    m_drive.ArcadeDrive(xSpeed, zRotation, squareInputs);
}

void DriveSubsystem::Periodic() {
    m_odometry.Update(m_gyro.GetHeadingAsRot2d()
                     , units::inch_t(m_leftLeader.GetSelectedSensorPosition(0) / m_ticksPerInch)
                     , units::inch_t(m_rightLeader.GetSelectedSensorPosition(0) / m_ticksPerInch));

    double ypr[3];
    ctre::phoenix::ErrorCode e = m_gyro.GetYawPitchRoll(ypr);
    if (e == ctre::phoenix::OK)
    {
        frc::SmartDashboard::PutNumber("Yaw 1", ypr[0]);
        frc::SmartDashboard::PutNumber("Pitch 1", ypr[1]);
        frc::SmartDashboard::PutNumber("Roll 1", ypr[2]);
    }
}

frc::Pose2d DriveSubsystem::GetPose() {
    return m_odometry.GetPose();
}

void DriveSubsystem::SetWheelSpeeds(units::meters_per_second_t leftSpeed, units::meters_per_second_t rightSpeed) {
    m_drive.TankDrive(leftSpeed.to<double>(), rightSpeed.to<double>());
}