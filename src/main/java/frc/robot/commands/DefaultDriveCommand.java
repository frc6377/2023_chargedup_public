package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DefaultDriveCommand extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final Supplier<Rotation2d> robotRotationSupplier;

  public DefaultDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      Supplier<Rotation2d> robotRotationSupplier) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.robotRotationSupplier = robotRotationSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented
    // movement
    m_drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationXSupplier.getAsDouble()
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            m_translationYSupplier.getAsDouble()
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            m_rotationSupplier.getAsDouble()
                * m_drivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            robotRotationSupplier.get()));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
