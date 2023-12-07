// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.config.IdentifyRoborio;
import frc.config.RobotVersion;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmPowerCommand;
import frc.robot.commands.ArmPowerCommandWithZero;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.EndAffectorEjectCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RoutineFactory;
import frc.robot.commands.RoutineFactory.Routines;
import frc.robot.commands.SwerveAutoFactory;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.DeploySubsystem;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.GamePieceMode;
import frc.robot.subsystems.color.SignalingSubsystem;
import frc.robot.subsystems.drivetrain.DriveInput;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.config.DriverConfig;
import frc.robot.subsystems.fieldPositioningSystem.FieldPositioningSystem;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  // Find the configuration class to load for this robot
  RobotVersion robotVersion = IdentifyRoborio.identifyRobot();
  // Input controllers
  private final HowdyXboxController driver =
      new HowdyXboxController(Constants.DRIVER_CONTROLLER_ID);
  private final HowdyXboxController gunner =
      new HowdyXboxController(Constants.GUNNER_CONTROLLER_ID);
  // Subsystems
  private final DeploySubsystem deploySubsystem = new DeploySubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final RobotStateManager robotState;
  private final EndAffectorSubsystem endAffector;
  private final SignalingSubsystem colorStrip;
  private final FieldPositioningSystem fieldPositioningSystem = new FieldPositioningSystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(robotVersion);
  private final SwerveAutoFactory autoCommand =
      new SwerveAutoFactory(fieldPositioningSystem::resetRobotPosition, drivetrainSubsystem);
  private final SendableChooser<Routines> autoChooser;
  private final RoutineFactory routineFactory;
  // private GamePieceMode gamePieceMode;

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
        new DriveInput(
            driver::getRightY,
            driver::getRightX,
            driverConfig);

    robotState = new RobotStateManager(GamePieceMode.CUBE, ArmHeight.LOW);

    endAffector = new EndAffectorSubsystem(Constants.END_AFFECTOR_ID, Constants.END_AFFECTOR_KP);

    colorStrip = new SignalingSubsystem(Constants.GAME_PIECE_CANDLE, driver::setRumble);

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

    autoChooser = new SendableChooser<>();
    addChooserOptions();
    routineFactory =
        new RoutineFactory(arm, endAffector, drivetrainSubsystem, autoCommand, robotState);

    configureBindings();
  }

  private void configureBindings() {
    Trigger highGearButton = driver.rightBumper();
    Trigger driverPickup = driver.a();

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

    driverPickup.onTrue(new ArmPowerCommandWithZero(ArmHeight.LOW, arm, 3.0, robotState));

    drivetrainSubsystem.setDefaultCommand(driveCommand);

    Trigger intakeButton = driver.leftTrigger(0.3);
    Trigger shootButton = driver.rightTrigger(0.3);

    intakeButton.whileTrue(new IntakeCommand(endAffector, colorStrip, arm));

    DoubleSupplier shootSupplier = driver::getRightTriggerAxis;

    shootButton.whileTrue(new EndAffectorEjectCommand(shootSupplier, endAffector));

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

    Trigger driverToggleGamePieceButton = driver.leftBumper();

    driverToggleGamePieceButton.onTrue(
        new InstantCommand(() -> robotState.SwitchCubeCone())
            .andThen(new ArmPowerCommand(ArmHeight.LOW, arm, 3, robotState)));

    Trigger driverToggleSingleSubstation = driver.y();

    driverToggleSingleSubstation.onTrue(
        new InstantCommand(() -> robotState.ToggleSingleSubstationMode())
            .andThen(
                new ArmPowerCommand(ArmPosition.HYBRID_CUBE_ARM_POSITION, arm, 3, robotState)));

    Trigger driverStowed = driver.x();
    Trigger gunnerStowed = gunner.x();

    gunnerStowed.onTrue(
        new ArmPowerCommand(ArmPosition.HYBRID_CUBE_ARM_POSITION, arm, 3, robotState));

    Trigger driverDefenseStowed = driver.b();

    Trigger gunnerDefenseStowed = gunner.rightBumper();
    gunnerDefenseStowed.onTrue(
        new ArmPowerCommandWithZero(ArmHeight.HIGH_STOWED, arm, 3, robotState));

    Trigger zeroElevator = gunner.start();
    zeroElevator.onTrue(new ZeroElevator(arm));

    Trigger gunnerSelfRight1 = gunner.start();
    Trigger gunnerSelfRight2 = gunner.back();
    Trigger gunnerLowButton = gunner.a();
    Trigger gunnerMidButton = gunner.b();
    Trigger gunnerHighButton = gunner.y();
    Trigger gunnerLeftY =
        new Trigger(() -> Math.abs(gunner.getLeftY()) > Constants.ARM_MANUAL_OVERRIDE_DEADZONE);
    Trigger gunnerRightY =
        new Trigger(() -> Math.abs(gunner.getRightY()) > Constants.ARM_MANUAL_OVERRIDE_DEADZONE);

    driverStowed.onTrue(new ArmPowerCommand(ArmHeight.STOWED, arm, 3, robotState));
    driverDefenseStowed.onTrue(new ArmPowerCommand(ArmHeight.HIGH_STOWED, arm, 3, robotState));
    gunnerLeftY
        .or(gunnerRightY)
        .onTrue(new ArmManualCommand(gunnerLeftYSupplier, gunnerRightYSupplier, arm));

    gunnerSelfRight1
        .and(gunnerSelfRight2)
        .onTrue(new ArmPowerCommand(ArmPosition.SELF_RIGHT_POSITION, arm, 3, robotState));
    gunnerLowButton.onTrue(new ArmPowerCommandWithZero(ArmHeight.LOW, arm, 3, robotState));
    gunnerMidButton.onTrue(new ArmPowerCommand(ArmHeight.MID, arm, 3, robotState));
    gunnerHighButton.onTrue(new ArmPowerCommand(ArmHeight.HIGH, arm, 3, robotState));
    gunnerHybridButton.onTrue(new ArmPowerCommand(ArmHeight.HYBRID, arm, 3, robotState));
  }

  public Command getAutonomousCommand() {
    // AutoRoutines should be used to add more auto routines that we'll execute.
    return routineFactory.getAuto(autoChooser.getSelected());
  }

  public void onDisabled() {
    colorStrip.randomizePattern();
  }

  public void onExitDisabled() {
    colorStrip.stopRainbowAnimation();
  }

  public void updateLEDs() {
    colorStrip.updateLEDs();
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
    autoChooser.addOption("right 3 element no climb", Routines.RIGHT_3_ELEMENT_NOCLIMB);
    autoChooser.addOption("left volume", Routines.LEFT_VOLUME);
    autoChooser.addOption("left 3 element no climb", Routines.LEFT_3_ELEMENT_NOCLIMB);
    autoChooser.addOption("left bumpside 3 element", Routines.LEFT_BUMPSIDE_3_ELEMENT);
    autoChooser.addOption("mid 1.5 and mobility", Routines.MID_1_AND_GRAB);
    autoChooser.addOption("right bumpside 3 element", Routines.RIGHT_BUMPSIDE_3_ELEMENT);
    SmartDashboard.putData(autoChooser);
  }

  public void setElevator() {
    arm.setElevator();
  }

  public void onFatalError() {
    colorStrip.displayCriticalError();
  }
}
