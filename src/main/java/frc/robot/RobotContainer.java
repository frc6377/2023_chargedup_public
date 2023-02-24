// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmPowerCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveAutoFactory;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.EndAffectorSubsystem;
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
  private final BooleanSubscriber isCubeSubscriber;
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
    isCubeSubscriber = isCubeTopic.subscribe(true);
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
    Trigger driverMidButton = driver.x();

    Trigger gunnerHighButton = gunner.y();
    Trigger gunnerMidButton = gunner.b();
    Trigger gunnerLowButton = gunner.a();
    Trigger gunnerStowedButton = gunner.x();

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

    controlMethod.onTrue(Commands.startEnd(driveCommand::toggleDriveType, () -> {}));
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

    driverToggleGamePieceButton.toggleOnTrue(
        Commands.runOnce(() -> endAffector.toggleGamePiece(), endAffector));

    gunnerLowButton.onTrue(
        new ArmPowerCommand(
            () -> isCubeSubscriber.get() ? Constants.CUBE_LOW : Constants.CONE_LOW, arm, getBay()));

    gunnerMidButton.onTrue(
        new ArmPowerCommand(
            () -> isCubeSubscriber.get() ? Constants.CUBE_MID : Constants.CONE_MID, arm, getBay()));

    gunnerHighButton.onTrue(
        new ArmPowerCommand(
            () -> isCubeSubscriber.get() ? Constants.CUBE_HIGH : Constants.CONE_HIGH,
            arm,
            getBay()));

    gunnerStowedButton.onTrue(new ArmPowerCommand(() -> Constants.STOWED, arm, getBay()));

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

  public void onDisabled() {
    colorStrip.startRainbowAnimation();
  }

  public void onExitDisabled() {
    colorStrip.stopRainbowAnimation();
  }

  private boolean isDriving() {
    return 0.5 < Math.hypot(driveController.getLeftX(), driveController.getLeftY());
  }
}
