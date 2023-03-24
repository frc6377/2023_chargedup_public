// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.config.IdentifyRoborio;
import frc.config.RobotVersion;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmPowerCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RoutineFactory;
import frc.robot.commands.RoutineFactory.Routines;
import frc.robot.commands.SwerveAutoFactory;
import frc.robot.commands.SwitchTargetObject;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.ColorSubsystem;
import frc.robot.subsystems.drivetrain.DriveInput;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.config.DriverConfig;
import frc.robot.subsystems.fieldPositioningSystem.FieldPositioningSystem;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  // Find the configuration class to load for this robot
  RobotVersion robotVersion = IdentifyRoborio.identifyRobot();
  // Input controllers
  private final CommandXboxController driver =
      new CommandXboxController(Constants.DRIVER_CONTROLLER_ID);
  private final CommandXboxController gunner =
      new CommandXboxController(Constants.GUNNER_CONTROLLER_ID);
  private final StreamDeck streamDeck = new StreamDeck(2, 36);
  // Subsystems
  private final DeploySubsystem deploySubsystem = new DeploySubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final BooleanTopic isCubeTopic;
  private final BooleanSubscriber isCubeSubscriber;
  private final EndAffectorSubsystem endAffector;
  private final ColorSubsystem colorStrip;
  private final FieldPositioningSystem fieldPositioningSystem = new FieldPositioningSystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(robotVersion);
  private final SwerveAutoFactory autoCommand =
      new SwerveAutoFactory(fieldPositioningSystem::resetRobotPosition, drivetrainSubsystem);
  private final SendableChooser<Routines> autoChooser;
  private final RoutineFactory routineFactory;

  public RobotContainer() {
    deploySubsystem.Log();

    DriverConfig driverConfig = new DriverConfig();
    DoubleSupplier xSupplier =
        new DriveInput(driver::getLeftY, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier ySupplier =
        new DriveInput(driver::getLeftX, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier rotationSupplier =
        new DriveInput(driver::getLeftX, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier pointingDriveInput =
        new DriveInput(driver::getRightY, driver::getRightX, driverConfig);

    isCubeTopic = NetworkTableInstance.getDefault().getBooleanTopic("isCube");
    // Default to Cube.
    isCubeTopic.publish().set(true);
    isCubeSubscriber = isCubeTopic.subscribe(false);
    endAffector = new EndAffectorSubsystem(Constants.END_AFFECTOR_ID, isCubeSubscriber.get());
    colorStrip =
        new ColorSubsystem(
            Constants.GAME_PIECE_CANDLE, Constants.GRID_SELECT_CANDLE, isCubeSubscriber.get());

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

    arm.setIsCubeSupplier(isCubeSubscriber);

    autoChooser = new SendableChooser<>();
    addChooserOptions();
    routineFactory = new RoutineFactory(arm, endAffector, drivetrainSubsystem, autoCommand);

    configureBindings();
  }

  private void configureBindings() {
    Trigger highGearButton = driver.rightBumper();

    DriverConfig driverConfig = new DriverConfig();
    DoubleSupplier xSupplier =
        new DriveInput(driver::getLeftY, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier ySupplier =
        new DriveInput(driver::getLeftX, DriveInput.InputType.TRANSLATION, driverConfig, false);
    DoubleSupplier turnSupplier =
        new DriveInput(driver::getRightX, DriveInput.InputType.ROTATION, driverConfig, false);
    DoubleSupplier pointingDriveInput =
        new DriveInput(driver::getRightY, driver::getRightX, driverConfig);
    DoubleSupplier gunnerLeftYSupplier = gunner::getLeftY;
    DoubleSupplier gunnerRightYSupplier = gunner::getRightY;
    Trigger gunnerHybridButton = gunner.rightBumper();

    DriveCommand driveCommand =
        new DriveCommand(
            drivetrainSubsystem,
            fieldPositioningSystem,
            xSupplier,
            ySupplier,
            turnSupplier,
            pointingDriveInput);

    drivetrainSubsystem.setDefaultCommand(driveCommand);

    Trigger singleSubstation = driver.y();
    singleSubstation.onTrue(new ArmPowerCommand(Constants.SINGLE_SUBSTATION_CONE_POSITION, arm, 3));

    Trigger intakeButton = driver.leftTrigger(0.3);
    Trigger shootButton = driver.rightTrigger(0.3);

    intakeButton.whileTrue(
        Commands.startEnd(() -> endAffector.intake(), () -> endAffector.idle(), endAffector));

    shootButton.whileTrue(
        Commands.startEnd(() -> endAffector.fastOutake(), () -> endAffector.halt(), endAffector));

    Trigger driverGoButton = driver.a();
    Trigger driverResetFieldNorth = driver.start();

    driverResetFieldNorth.onTrue(
        new InstantCommand(
            () ->
                fieldPositioningSystem.resetRobotPosition(
                    new Pose2d(
                        fieldPositioningSystem.getRobotXYPose().getTranslation(),
                        new Rotation2d(0)))));

    highGearButton.whileTrue(
        Commands.startEnd(
            () -> DriveInput.setToHighGear(true), () -> DriveInput.setToHighGear(false)));

    // driverGoButton.onTrue(autoCommand.generateGridCommand(getBay()).until(this::isDriving));

    Trigger driverToggleGamePieceButton = driver.leftBumper();
    driverToggleGamePieceButton.onTrue(
        new SwitchTargetObject(endAffector, arm, colorStrip, isCubeTopic));

    Trigger driverStowed = driver.x();
    Trigger gunnerStowed = gunner.x();

    gunnerStowed.onTrue(new ArmPowerCommand(Constants.STOWED_ARM_POSITION, arm, 3));
    driverStowed.onTrue(
        Commands.runOnce(() -> driverStowBehavior().schedule(), new Subsystem[] {}));

    Trigger driverDefenseStowed = driver.b();
    driverDefenseStowed.onTrue(new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3));

    Trigger gunnerDefenseStowed = gunner.rightBumper();
    gunnerDefenseStowed.onTrue(new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3));

    Trigger zeroElevator = gunner.start();
    zeroElevator.onTrue(new ZeroElevator(arm));

    Trigger gunnerLowButton = gunner.a();
    Trigger gunnerMidButton = gunner.b();
    Trigger gunnerHighButton = gunner.y();
    Trigger gunnerLeftY =
        new Trigger(() -> Math.abs(gunner.getLeftY()) > Constants.ARM_MANUAL_OVERRIDE_DEADZONE);
    Trigger gunnerRightY =
        new Trigger(() -> Math.abs(gunner.getRightY()) > Constants.ARM_MANUAL_OVERRIDE_DEADZONE);

    gunnerLeftY
        .or(gunnerRightY)
        .onTrue(new ArmManualCommand(gunnerLeftYSupplier, gunnerRightYSupplier, arm));

    gunnerLowButton
        .and(() -> isCubeSubscriber.get())
        .onTrue(new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3));
    gunnerLowButton
        .and(() -> !isCubeSubscriber.get())
        .onTrue(new ArmPowerCommand(Constants.LOW_CONE_ARM_POSITION, arm, 3));
    gunnerMidButton
        .and(() -> isCubeSubscriber.get())
        .onTrue(new ArmPowerCommand(Constants.MID_CUBE_ARM_POSITION, arm, 3));
    gunnerMidButton
        .and(() -> !isCubeSubscriber.get())
        .onTrue(new ArmPowerCommand(Constants.MID_CONE_ARM_POSITION, arm, 3));
    gunnerHighButton
        .and(() -> isCubeSubscriber.get())
        .onTrue(new ArmPowerCommand(Constants.HIGH_CUBE_ARM_POSITION, arm, 3));
    gunnerHighButton
        .and(() -> !isCubeSubscriber.get())
        .onTrue(new ArmPowerCommand(Constants.HIGH_CONE_ARM_POSITION, arm, 3));

    gunnerHybridButton
        .and(() -> isCubeSubscriber.get())
        .onTrue(new ArmPowerCommand(Constants.HYBRID_CUBE_ARM_POSITION, arm, 3));
    gunnerHybridButton
        .and(() -> !isCubeSubscriber.get())
        .onTrue(new ArmPowerCommand(Constants.HYBRID_CONE_ARM_POSITION, arm, 3));

    Trigger gunnerLefTrigger = gunner.leftTrigger();
    gunnerLefTrigger.onTrue(new AutoBalanceCommand(drivetrainSubsystem));
  }

  private Command driverStowBehavior() {
    if (arm.getArmPosition().getHeight() == ArmHeight.STOWED
        || arm.getArmPosition().getHeight() == ArmHeight.STOWED) {
      ArmPosition targetPosition =
          ArmPosition.getArmPositionFromHeightAndType(ArmHeight.LOW, isCubeSubscriber.get());
      return new ArmPowerCommand(targetPosition, arm, 3);
    } else {
      return new ArmPowerCommand(Constants.STOWED_ARM_POSITION, arm, 3);
    }
  }

  public void updateLEDs() {
    colorStrip.forceUpdate();
  }

  public Command getAutonomousCommand() {
    // AutoRoutines should be used to add more auto routines that we'll execute.

    return routineFactory.getAuto(autoChooser.getSelected());
  }

  public int getBay() {
    int selected = streamDeck.getSelected() - 1;
    int grid = (selected / 9) * 3; // if we are in the left right or middle grid
    return 8 - (grid + (selected % 9 % 3)); // if we are in the "1, 2, or 3" bays per grid
  }

  public void onDisabled() {
    colorStrip.randomizePattern();
  }

  public void onExitDisabled() {
    colorStrip.stopRainbowAnimation();
  }

  private boolean isDriving() {
    return 0.5 < Math.hypot(driver.getLeftX(), driver.getLeftY());
  }

  public void unbindShoulder() {
    ArmPosition currentPos = arm.getArmPosition();
    ArmPosition unboundPos =
        new ArmPosition(
            currentPos.getArmRotation() + Math.toRadians(5),
            currentPos.getArmExtension(),
            currentPos.getWristRotation(),
            ArmHeight.NOT_SPECIFIED);
    new InstantCommand(() -> arm.setTarget(unboundPos)).schedule();
  }

  private void addChooserOptions() {
    autoChooser.setDefaultOption("no op", Routines.NO_OP);
    autoChooser.addOption("preload only", Routines.PRELOAD_ONLY);
    autoChooser.addOption("mid climb", Routines.MIDDLE_CLIMB);
    autoChooser.addOption("left 2 element climb", Routines.LEFT_2_ELEMENT_CLIMB);
    autoChooser.addOption("right 2 element climb", Routines.RIGHT_2_ELEMENT_CLIMB);
    autoChooser.addOption("left 2 element no climb", Routines.LEFT_2_ELEMENT_NOCLIMB);
    autoChooser.addOption("right 2 element no climb", Routines.RIGHT_2_ELEMENT_NOCLIMB);
    SmartDashboard.putData(autoChooser);
  }

  public void setElevator() {
    arm.setElevator();
  }
}
