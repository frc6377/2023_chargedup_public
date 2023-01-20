// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.fieldPositioningSystem.FieldPositioningSystem;

public class RobotContainer {
  // Input controllers
  private final XboxController driveController = new XboxController(0);
  // Subsystems
  private final DeploySubsystem deploySubsystem = new DeploySubsystem();
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController gunner = new CommandXboxController(1);
  ArmSubsystem arm = new ArmSubsystem(11, 12);
  EndAffectorSubsystem endAffector = new EndAffectorSubsystem(9, 10);

  private final FieldPositioningSystem fieldPositioningSystem = new FieldPositioningSystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(null);

  public RobotContainer() {
    deploySubsystem.Log();

    fieldPositioningSystem.setDriveTrainSupplier(
        () -> drivetrainSubsystem.getOdometry(), drivetrainSubsystem.getKinematics());

    drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivetrainSubsystem,
            () -> MathUtil.applyDeadband(-driveController.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(-driveController.getLeftX(), 0.05),
            () -> MathUtil.applyDeadband(-driveController.getRightX(), 0.05),
            () -> fieldPositioningSystem.getCurrentRobotRotationXY()));
    configureBindings();
  }

  private void configureBindings() {

    Trigger intakeButton = driver.leftTrigger(0.3);
    Trigger shootButton = driver.rightTrigger(0.3);
    Trigger gunnerHighButton = gunner.a();
    Trigger gunnerMidButton = gunner.x();
    Trigger driverHighButton = driver.a();
    Trigger driverMidButton = driver.x();

    intakeButton.whileTrue(
        Commands.startEnd(() -> endAffector.intake(), () -> endAffector.idle(), endAffector));

    // This watches for the buttons to be pressed then released,
    // thereby making the arm extend quickly.
    shootButton
        .and(gunnerMidButton.or(driverMidButton).negate())
        .whileTrue(
            Commands.startEnd(
                () -> endAffector.fastOutake(), () -> endAffector.halt(), endAffector));
    // This watches for the buttons to be pressed and held, thereby making the arm extend slowly.
    shootButton
        .and(gunnerMidButton.or(driverMidButton))
        .whileTrue(
            Commands.startEnd(
                () -> endAffector.slowOutake(), () -> endAffector.halt(), endAffector));
    gunnerHighButton
        .or(driverHighButton)
        .whileTrue(Commands.startEnd(() -> arm.setHigh(), () -> arm.setLow(), arm));
    gunnerMidButton
        .or(driverMidButton)
        .whileTrue(Commands.startEnd(() -> arm.setMid(), () -> arm.setLow(), arm));
  }

  public Command getAutonomousCommand() {
    // AutoRoutines should be used to add more auto routines that we'll execute.
    return AutoRoutines.Noop();
  }
}
