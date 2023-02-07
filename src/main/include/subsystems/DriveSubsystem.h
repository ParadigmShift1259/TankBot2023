#include <wpi/DataLog.h>
#include <units/length.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/Phoenix.h>

#include "Gyro.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class DriveSubsystem : public frc2::SubsystemBase

{
public:
    DriveSubsystem();
    void Drive(double xSpeed, double zRotation, bool squareInputs = true);
    void Periodic();
    frc::Pose2d GetPose();
    void SetWheelSpeeds(units::meters_per_second_t leftSpeed, units::meters_per_second_t rightSpeed);
    double GetPitch() { return m_gyro.GetPitch(); }

    const frc::DifferentialDriveKinematics& GetDriveKinematics() const { return kDriveKinematics; }

private:
    frc::DifferentialDriveKinematics kDriveKinematics{m_trackWidth};

    WPI_TalonFX m_leftLeader{1};
    WPI_TalonFX m_leftFollowerA{2};
    WPI_TalonFX m_leftFollowerB{3};

    WPI_TalonFX m_rightLeader{4};
    WPI_TalonFX m_rightFollowerA{5};
    WPI_TalonFX m_rightFollowerB{6};
    
    frc::MotorControllerGroup m_leftmcgroup;
    frc::MotorControllerGroup m_rightmcgroup;
    frc::DifferentialDrive m_drive;

    frc::DifferentialDriveOdometry m_odometry;
    Gyro m_gyro;
    double m_ticksPerInch = 286.479; //5400 ticks per rev divided by 6 pi (circumfrence of 6.0_in diameter wheels)
    units::inch_t m_trackWidth = 21.0_in; //track width from middle of wheels

  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
  //wpi::log::DoubleLogEntry m_logRobotSpeed;
  //wpi::log::DoubleLogEntry m_logRobotAccel;
  wpi::log::DoubleLogEntry m_logGyroPitch;
};