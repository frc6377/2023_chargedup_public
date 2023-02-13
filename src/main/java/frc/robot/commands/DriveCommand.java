package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.fieldPositioningSystem.FieldPositioningSystem;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final FieldPositioningSystem fieldPositioningSystem;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final DoubleSupplier pointingSupplier;
  private final ProfiledPIDController turnController;
  private DriveType driveType;

  public DriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      FieldPositioningSystem fieldPositioningSystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier pointingSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.fieldPositioningSystem = fieldPositioningSystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.pointingSupplier = pointingSupplier;
    turnController =
        new ProfiledPIDController(
            6, 0.1, 0.1, new TrapezoidProfile.Constraints(Math.PI * 8, Math.PI * 4));
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrainSubsystem);
  }

  public void execute() {
    switch (driveType) {
      case ABSOLUTE_POINTING:
        drivetrainSubsystem.drive(absolutePointing());
        break;
      case CLASSIC:
        drivetrainSubsystem.drive(classicDriving());
        break;
      default:
        break;
    }
  }

  public void toggleDriveType() {
    if (driveType == DriveType.CLASSIC) {
      driveType = DriveType.ABSOLUTE_POINTING;
    } else {
      driveType = DriveType.CLASSIC;
    }
  }

  private ChassisSpeeds classicDriving() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        translationXSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        translationYSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        rotationSupplier.getAsDouble()
            * drivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        fieldPositioningSystem.getCurrentRobotRotationXY());
  }

  private ChassisSpeeds absolutePointing() {
    turnController.setGoal(pointingSupplier.getAsDouble());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        translationXSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        translationYSupplier.getAsDouble() * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        turnController.calculate(fieldPositioningSystem.getCurrentRobotRotationXY().getRadians()),
        fieldPositioningSystem.getCurrentRobotRotationXY());
  }

  private enum DriveType {
    ABSOLUTE_POINTING,
    CLASSIC
  }
}
