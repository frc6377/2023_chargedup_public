// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveAutoFactory;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.ColorSubsystem;
import frc.robot.subsystems.drivetrain.DriveInput;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.config.DriverConfig;
import frc.robot.subsystems.fieldPositioningSystem.FieldPositioningSystem;
import java.util.function.DoubleSupplier;

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

    DriverConfig driverConfig = new DriverConfig();
    DoubleSupplier xSupplier =
        new DriveInput(
            driveController::getLeftY, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier ySupplier =
        new DriveInput(
            driveController::getLeftX, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier rotationSupplier =
        new DriveInput(
            driveController::getLeftX, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier pointingDriveInput =
        new DriveInput(driveController::getRightY, driveController::getRightX, driverConfig);

    isCubeTopic = NetworkTableInstance.getDefault().getBooleanTopic("isCube");
    endAffector = new EndAffectorSubsystem(Constants.END_AFFECTOR_ID, isCubeTopic);
    colorStrip =
        new ColorSubsystem(Constants.GAME_PIECE_CANDLE, Constants.GRID_SELECT_CANDLE, isCubeTopic);

    fieldPositioningSystem.setDriveTrainSupplier(
        () -> drivetrainSubsystem.getOdometry(), drivetrainSubsystem.getKinematics());

    drivetrainSubsystem.setDefaultCommand(
        new DriveCommand(
            drivetrainSubsystem,
            fieldPositioningSystem,
            xSupplier,
            ySupplier,
            rotationSupplier,
            pointingDriveInput));

    configureBindings();
  }

  private void configureBindings() {

    Trigger intakeButton = driver.leftTrigger(0.3);
    Trigger shootButton = driver.rightTrigger(0.3);
    Trigger highGearButton = driver.rightBumper();
    Trigger controlMethod = driver.back();

    Trigger gunnerMidButton = gunner.x();
    Trigger driverMidButton = driver.x();

    DriverConfig driverConfig = new DriverConfig();
    DoubleSupplier xSupplier =
        new DriveInput(
            driveController::getLeftY, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier ySupplier =
        new DriveInput(
            driveController::getLeftX, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier turnSupplier =
        new DriveInput(
            driveController::getRightX, DriveInput.InputType.ROTATION, driverConfig, false);
    DoubleSupplier pointingDriveInput =
        new DriveInput(driveController::getRightY, driveController::getRightX, driverConfig);

    DriveCommand driveCommand =
        new DriveCommand(
            drivetrainSubsystem,
            fieldPositioningSystem,
            xSupplier,
            ySupplier,
            turnSupplier,
            pointingDriveInput);

    drivetrainSubsystem.setDefaultCommand(driveCommand);

    controlMethod.onTrue(
        Commands.startEnd(driveCommand::toggleDriveType, () -> {}));
    Trigger driverGoButton = driver.b();
    Trigger driverResetFieldNorth = driver.start();
    Trigger driverToggleGamePieceButton = driver.leftBumper();

    intakeButton.whileTrue(
        Commands.startEnd(() -> endAffector.intake(), () -> endAffector.idle(), endAffector));

    driverResetFieldNorth.whileTrue(
        new InstantCommand(
            () ->
                fieldPositioningSystem.resetRobotPosition(
                    new Pose2d(
                        fieldPositioningSystem.getRobotXYPose().getTranslation(),
                        new Rotation2d(Math.PI)))));

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

    highGearButton.whileTrue(
        Commands.startEnd(
            () -> DriveInput.setToHighGear(true), () -> DriveInput.setToHighGear(false)));
    
    Trigger retract = gunner.leftTrigger(0.3);
    Trigger extend = gunner.a();


    retract.whileTrue(Commands.runOnce(() ->arm.setTarget(new ArmPosition(0, 1, -8475, "NAN")), arm));
    extend.whileTrue(new ArmCommand(new Translation2d(.5,.5), -8475, arm));

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
