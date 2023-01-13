package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class FieldPositioningSystem extends SubsystemBase {

  private Pose3d currentRobotPose;
  private Supplier<SwerveModulePosition[]> swerveOdemSupplier;
  private SwerveDriveOdometry swerveDriveOdometry;

  public FieldPositioningSystem() {}

  /**
   * returns the current robot rotation on the X-Y plane computed from the gyro and vision data
   *
   * @return the current robot rotation on the X-Y plane as a Rotation2d
   */
  public Rotation2d getCurrentRobotRotationXY() {
    return currentRobotPose.getRotation().toRotation2d();
  }

  public Pose2d getRobotXYPose() {
    return currentRobotPose.toPose2d();
  }

  public void setDriveTrainSupplier(
      Supplier<SwerveModulePosition[]> swerveOdemSupplier, SwerveDriveKinematics swerveKinematics) {
    if (swerveOdemSupplier == null) {
      return;
    }
    this.swerveOdemSupplier = swerveOdemSupplier;

    swerveDriveOdometry =
        new SwerveDriveOdometry(
            swerveKinematics, getCurrentRobotRotationXY(), swerveOdemSupplier.get());
  }

  /** Called every 20ms to provide update the robots position */
  @Override
  public void periodic() {
    updateSwerveDriveOdemetry();
  }

  private void updateSwerveDriveOdemetry() {
    if (swerveOdemSupplier == null) {
      return;
    }

    SwerveModulePosition[] moduleStates = swerveOdemSupplier.get();

    swerveDriveOdometry.update(getCurrentRobotRotationXY(), moduleStates);
  }
}
