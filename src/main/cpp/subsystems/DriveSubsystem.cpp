#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>

DriveSubsystem::DriveSubsystem()
    : m_leftmcgroup(m_leftLeader, m_leftFollowerA, m_leftFollowerB)
    , m_rightmcgroup(m_rightLeader, m_rightFollowerA, m_rightFollowerB)
    , m_drive(m_leftmcgroup,m_rightmcgroup)
    , m_odometry(frc::Rotation2d(), 0_m, 0_m)
    , m_gyro()
{
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_logRobotPoseX = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseX");
    m_logRobotPoseY = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseY");
    m_logRobotPoseTheta = wpi::log::DoubleLogEntry(log, "/odometry/robotPoseTheta");   
    //m_logRobotSpeed = wpi::log::DoubleLogEntry(log, "/odometry/robotSpeed");
    //m_logRobotAccel = wpi::log::DoubleLogEntry(log, "/odometry/robotAccel");
    m_logGyroPitch = wpi::log::DoubleLogEntry(log, "/gyro/pitch");

    m_rightmcgroup.SetInverted(true);
    m_leftLeader.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_leftFollowerA.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_leftFollowerB.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_rightLeader.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_rightFollowerA.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    m_rightFollowerB.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
}


void DriveSubsystem::Drive(double xSpeed, double zRotation, bool squareInputs /*= true*/)
{
    // if (xSpeed > 0.1)
    // {
    //     printf("Drive at %.3f\n", xSpeed);
    // }
    m_drive.ArcadeDrive(xSpeed, zRotation, squareInputs);
}

void DriveSubsystem::Periodic() {
    m_odometry.Update(m_gyro.GetHeadingAsRot2d()
                     , units::inch_t(m_leftLeader.GetSelectedSensorPosition(0) / m_ticksPerInch)
                     , units::inch_t(m_rightLeader.GetSelectedSensorPosition(0) / m_ticksPerInch));

  //Log Odometry Values
  frc::Pose2d pose = m_odometry.GetPose();
  m_logRobotPoseX.Append(pose.X().to<double>());
  m_logRobotPoseY.Append(pose.Y().to<double>());
  m_logRobotPoseTheta.Append(pose.Rotation().Degrees().to<double>());
  //m_logRobotSpeed.Append(m_velocity);
  //m_logRobotAccel.Append(m_acceleration);
  frc::SmartDashboard::PutNumber("GyroPitch", m_gyro.GetPitch());
  m_logGyroPitch.Append(m_gyro.GetPitch()); 


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