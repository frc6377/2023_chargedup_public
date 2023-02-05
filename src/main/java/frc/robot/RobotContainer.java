// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SwerveAutoFactory;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.ColorSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.fieldPositioningSystem.FieldPositioningSystem;

public class RobotContainer {
  // Input controllers
  private final XboxController driveController =
      new XboxController(Constants.driverControllerID);
  private final StreamDeck streamDeck = new StreamDeck(2, 36);
  // Subsystems
  private final DeploySubsystem deploySubsystem = new DeploySubsystem();
  private final CommandXboxController driver =
      new CommandXboxController(Constants.driverControllerID);
  private final CommandXboxController gunner =
      new CommandXboxController(Constants.gunnerControllerID);
  private final ArmSubsystem arm =
      new ArmSubsystem();
      //TODO: Fix to make EndAffector Subsystem only use 1 motor because there will only BE 1 motor.
  private final EndAffectorSubsystem endAffector =
      new EndAffectorSubsystem(
          Constants.endAffectorID, Constants.endAffectorID);
  private final ColorSubsystem colorStrip = new ColorSubsystem(2);
  private final FieldPositioningSystem fieldPositioningSystem = new FieldPositioningSystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(null);
  private final SwerveAutoFactory autoCommand =
      new SwerveAutoFactory(fieldPositioningSystem::resetRobotPosition, drivetrainSubsystem);

  public RobotContainer() {
    deploySubsystem.Log();
    fieldPositioningSystem.setDriveTrainSupplier(
        () -> drivetrainSubsystem.getOdometry(), drivetrainSubsystem.getKinematics());

    drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivetrainSubsystem,
            () -> MathUtil.applyDeadband(-driveController.getLeftY(), 0.05),
            () -> MathUtil.applyDeadband(-driveController.getLeftX(), 0.05),
            () ->
                Math.pow(MathUtil.applyDeadband(driveController.getRightX(), 0.05), 2)
                    * Math.copySign(1, -driveController.getRightX()),
            () -> fieldPositioningSystem.getCurrentRobotRotationXY()));

    configureBindings();
  }

  private void configureBindings() {

    FieldPoses poses = new FieldPoses();

    Trigger intakeButton = driver.leftTrigger(0.3);
    Trigger shootButton = driver.rightTrigger(0.3);
    Trigger gunnerHighButton = gunner.a();
    Trigger gunnerMidButton = gunner.x();
    Trigger driverHighButton = driver.a();
    Trigger driverMidButton = driver.x();
    Trigger driverGoButton = driver.b();

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

    driver
        .povLeft()
        .debounce(0.05)
        .toggleOnTrue(Commands.runOnce(() -> colorStrip.positionColoring.Increment(), colorStrip));
    driver
        .povRight()
        .debounce(0.05)
        .toggleOnTrue(Commands.runOnce(() -> colorStrip.positionColoring.Decrement(), colorStrip));
    driver
        .povUp()
        .debounce(0.05)
        .toggleOnTrue(Commands.runOnce(() -> colorStrip.pieceColoring.toggleColor(), colorStrip));
    driver
        .povDown()
        .debounce(0.05)
        .toggleOnTrue(Commands.runOnce(() -> colorStrip.pieceColoring.toggleHeight(), colorStrip));

    gunnerHighButton
        .or(driverHighButton)
        .whileTrue(Commands.startEnd(() -> arm.setCubeHigh(), () -> arm.setLow(), arm));
    gunnerMidButton
        .or(driverMidButton)
        .whileTrue(Commands.startEnd(() -> arm.setCubeMid(), () -> arm.setLow(), arm));
    driverGoButton.whileTrue(
        Commands.runOnce(
            () -> CommandScheduler.getInstance().schedule(autoCommand.generateCommand(getBay()).until(this::isDriving))));
  }

  public Command getAutonomousCommand() {
    // AutoRoutines should be used to add more auto routines that we'll execute.

    return autoCommand.generateCommand("PickFirstElementRed", true);
  }

  public int getBay() {
    int selected = streamDeck.getSelected() - 1;
    int grid = (selected / 9) * 3; // if we are in the left right or middle grid
    return 8 - (grid + (selected % 9 % 3)); // if we are in the "1, 2, or 3" bays per grid
  }

  private boolean isDriving(){
    return 0.5 < Math.hypot(driveController.getLeftX(), driveController.getLeftY());
  }
}
