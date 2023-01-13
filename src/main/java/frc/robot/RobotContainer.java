// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.fieldPositioningSystem.FieldPositioningSystem;

public class RobotContainer {
  // Input controllers
  private final XboxController driveController = new XboxController(0);

  // Subsystems
  private final DeploySubsystem deploySubsystem = new DeploySubsystem();
  private final FieldPositioningSystem fieldPositioningSystem = new FieldPositioningSystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(null);

  public RobotContainer() {
    deploySubsystem.Log();
    drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivetrainSubsystem,
            () -> MathUtil.applyDeadband(-driveController.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(-driveController.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(driveController.getRightX(), 0.05),
            () -> fieldPositioningSystem.getCurrentRobotRotationXY()));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    // AutoRoutines should be used to add more auto routines that we'll execute.
    return AutoRoutines.Noop();
  }
}
