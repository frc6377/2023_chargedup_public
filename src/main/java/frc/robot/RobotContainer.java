// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.EndAffectorSubsystem;

public class RobotContainer {
  private final DeploySubsystem deploySubsystem = new DeploySubsystem();
  CommandXboxController controller = new CommandXboxController(0);
  ArmSubsystem arm = new ArmSubsystem(3);
  EndAffectorSubsystem endAffector = new EndAffectorSubsystem(6, 8);

  public RobotContainer() {
    deploySubsystem.Log();

    configureBindings();
  }

  private void configureBindings() {

    Trigger aButton = controller.a();
    Trigger bButton = controller.b();

    aButton.whileTrue(
        Commands.startEnd(() -> endAffector.intake(), () -> endAffector.halt(), endAffector));
    bButton.whileTrue(
        Commands.startEnd(() -> endAffector.outake(), () -> endAffector.halt(), endAffector));
  }

  public Command getAutonomousCommand() {
    // AutoRoutines should be used to add more auto routines that we'll execute.
    return AutoRoutines.Noop();
  }
}
