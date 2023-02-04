// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
//#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include <utility>  // For RunOnce

RobotContainer::RobotContainer() 
{
  // Initialize all of your commands and subsystems here
  std::vector<frc::Pose2d> testTrajectory
  {
    frc::Pose2d(0_in, 0_in, frc::Rotation2d(0_deg)),
    frc::Pose2d(12_in, 0_in, frc::Rotation2d(0_deg))
  };

  auto config = frc::TrajectoryConfig(units::meters_per_second_t{1.0}, units::meters_per_second_squared_t{1.0});
  config.SetKinematics(m_drive.GetDriveKinematics());
  m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(testTrajectory[0], {}, testTrajectory[1], config);
  m_autonomousCommand = GetCommandGroup();

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings()
{
  using namespace frc;
  using namespace frc2;
  using xbox = frc::XboxController::Button;

  m_drive.SetDefaultCommand(RunCommand(
    [this] {
      auto xInput = m_primaryController.GetLeftY() * -1.0;
      auto yInput = m_primaryController.GetLeftX() * -1.0;

      //if (!m_bRunningPark)
      {
        m_drive.Drive(xInput, yInput);
      }
    },
    {&m_drive}
  ));

  auto& primary = m_primaryController;
  JoystickButton(&primary, xbox::kB).OnTrue(GetParkAndBalanceCommand());
//  JoystickButton(&primary, xbox::kB).WhileTrue(&m_driveFwd);
//  JoystickButton(&primary, xbox::kB).WhileTrue(frc2::cmd::Run([this]() { m_drive.Drive(0.25, 0.0, false); }, {&m_drive}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_autonomousCommand;
}

frc2::SequentialCommandGroup* RobotContainer::GetCommandGroup() {
  return new frc2::SequentialCommandGroup
  (
      std::move(GetCommandPath())
    , frc2::InstantCommand([this]() { m_drive.Drive(0.0, 0.0); }, {&m_drive})
  );
}

frc2::RamseteCommand RobotContainer::GetCommandPath() {
  frc2::RamseteCommand ramseteCommand(
      m_trajectory                                                                                // frc::Trajectory
    , [this]() { return m_drive.GetPose(); }                                                      // std::function<frc::Pose2d ()>
    , m_ramseteController                                                                         // frc::RamseteController
    , m_drive.GetDriveKinematics()                                                                // frc::DifferentialDriveKinematics
    , [this](auto leftSpeed, auto rightSpeed) { m_drive.SetWheelSpeeds(leftSpeed, rightSpeed); }  // std::function<void (units::velocity::meters_per_second_t, units::velocity::meters_per_second_t)>
    , {&m_drive}                                                                                  // std::initializer_list
  );
  return ramseteCommand;
}

frc2::SequentialCommandGroup* RobotContainer::GetParkAndBalanceCommand()
{
    return new frc2::SequentialCommandGroup
    (
      //frc2::InstantCommand([this]() { m_bRunningPark = true; }, {})
//, frc2::RunCommand([this]() { m_drive.SetWheelSpeeds(0.25_mps, 0.25_mps); }, {&m_drive})
//, frc2::RunCommand([this]() { m_drive.Drive(0.25, 0.0, false); }, {&m_drive})
        frc2::ParallelDeadlineGroup
        (
              frc2::WaitUntilCommand([this]() { return m_drive.GetPitch() < -7.0; })
            , frc2::RunCommand([this]() { m_drive.Drive(0.2, 0.0, false); }, {&m_drive})
        )
        , frc2::ParallelDeadlineGroup
        (
            frc2::WaitCommand(1.600_s)
          , frc2::RunCommand([this]() { m_drive.Drive(0.19, 0.0, false); }, {&m_drive})
        )
        , frc2::InstantCommand([this]() { m_drive.Drive(0.0, 0.0, false); }, {&m_drive})
        //, frc2::InstantCommand([this]() { m_bRunningPark = false; }, {})
    );
}
