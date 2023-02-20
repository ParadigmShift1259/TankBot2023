// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/filter/SlewRateLimiter.h>

#include <frc2/command/Command.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/ConditionalCommand.h>

#include <vector>

#include "subsystems/DriveSubsystem.h"
#include <frc2/command/button/CommandXboxController.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
 public:
  RobotContainer();

  void Periodic();
  frc2::Command* GetAutonomousCommand();
  frc2::SequentialCommandGroup* GetCommandGroup();
  frc2::RamseteCommand GetCommandPath();
  frc2::SequentialCommandGroup* GetParkCommand();
  frc2::ConditionalCommand* GetParkAndBalanceCommand();

 private:
  void ConfigureButtonBindings();

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_drive;
  frc2::Command* m_autonomousCommand = nullptr;
  frc::Trajectory m_trajectory;
  frc::RamseteController m_ramseteController;

  // Changed to CommandXboxController to support binding D-Pad/POV buttons
  frc2::CommandXboxController m_primaryController{0};
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  void ConfigureButtonBindings();

//#define TEST_GP2040_BUTTON_BOX
#ifdef TEST_GP2040_BUTTON_BOX
  void PrintPOVLeft() { printf("POV Left button\n"); }
  void PrintPOVRight() { printf("POV Right button\n"); }
  void PrintPOVUp() { printf("POV Up button\n"); }
#endif
  // No weight
  //double m_pitchFactor = -0.0125;
  //double m_MaxAutoBalanceSpeed = 0.1;

  // About 108 lbs
  double m_pitchFactor = -0.0125;
  double m_maxAutoBalanceSpeed = 0.2;
};
