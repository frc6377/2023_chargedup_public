// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeploySubsystem;

public class RobotContainer {
  private final DeploySubsystem deploySubsystem = new DeploySubsystem();

  public RobotContainer() {
    deploySubsystem.Log();

    configureBindings();
  }

  private void configureBindings() {

    
  }

  public Command getAutonomousCommand() {
    // AutoRoutines should be used to add more auto routines that we'll execute.
    return AutoRoutines.Noop();
  }
}
