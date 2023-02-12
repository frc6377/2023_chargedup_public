// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private final XboxController driveController = new XboxController(Constants.DRIVER_CONTROLLER_ID);
  private final StreamDeck streamDeck = new StreamDeck(2, 36);
  // Subsystems
  private final DeploySubsystem deploySubsystem = new DeploySubsystem();
  private final CommandXboxController driver =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController gunner =
      new CommandXboxController(Constants.GUNNER_CONTROLLER_ID);
  private final ArmSubsystem arm = new ArmSubsystem();
  private final BooleanTopic isCubeTopic;
  private final EndAffectorSubsystem endAffector;
  private final ColorSubsystem colorStrip;
  private final FieldPositioningSystem fieldPositioningSystem = new FieldPositioningSystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(null);
  private final SwerveAutoFactory autoCommand =
      new SwerveAutoFactory(fieldPositioningSystem::resetRobotPosition, drivetrainSubsystem);

  public RobotContainer() {
    deploySubsystem.Log();

    isCubeTopic = NetworkTableInstance.getDefault().getBooleanTopic("isCube");
    endAffector = new EndAffectorSubsystem(Constants.END_AFFECTOR_ID, isCubeTopic);
    colorStrip =
        new ColorSubsystem(Constants.GAME_PIECE_CANDLE, Constants.GRID_SELECT_CANDLE, isCubeTopic);

    fieldPositioningSystem.setDriveTrainSupplier(
        () -> drivetrainSubsystem.getOdometry(), drivetrainSubsystem.getKinematics());

    drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivetrainSubsystem,
            () ->
                Math.pow(MathUtil.applyDeadband(-driveController.getLeftY(), 0.05), 2)
                    * Math.copySign(1, -driveController.getLeftY()),
            () ->
                Math.pow(MathUtil.applyDeadband(-driveController.getLeftX(), 0.05), 2)
                    * Math.copySign(1, -driveController.getLeftX()),
            () -> Math.pow(MathUtil.applyDeadband(driveController.getRightX(), 0.05), 3),
            () -> fieldPositioningSystem.getCurrentRobotRotationXY()));

    configureBindings();
  }

  private void configureBindings() {

    Trigger intakeButton = driver.leftTrigger(0.3);
    Trigger shootButton = driver.rightTrigger(0.3);
    Trigger gunnerHighButton = gunner.a();
    Trigger gunnerMidButton = gunner.x();
    Trigger gunnerLowButton = gunner.y();
    Trigger driverHighButton = driver.a();
    Trigger driverMidButton = driver.x();
    Trigger driverLowButton = driver.y();
    Trigger driverGoButton = driver.b();
    Trigger driverToggleGamePieceButton = driver.leftBumper();

    intakeButton.whileTrue(
        Commands.startEnd(() -> endAffector.intake(), () -> endAffector.idle(), endAffector));

    // This watches for the buttons to be pressed then released,
    // thereby making the arm extend quickly.
    driverToggleGamePieceButton.toggleOnTrue(
        Commands.runOnce(() -> endAffector.toggleGamePiece(), endAffector));

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

    driverGoButton.whileTrue(
        Commands.runOnce(
            () ->
                CommandScheduler.getInstance()
                    .schedule(autoCommand.generateGridCommand(getBay()).until(this::isDriving))));
  }

  public Command getAutonomousCommand() {
    // AutoRoutines should be used to add more auto routines that we'll execute.

    return autoCommand.generateCommandFromFile("PickFirstElementRed", true);
  }

  public int getBay() {
    int selected = streamDeck.getSelected() - 1;
    int grid = (selected / 9) * 3; // if we are in the left right or middle grid
    return 8 - (grid + (selected % 9 % 3)); // if we are in the "1, 2, or 3" bays per grid
  }

  private boolean isDriving() {
    return 0.5 < Math.hypot(driveController.getLeftX(), driveController.getLeftY());
  }
}
