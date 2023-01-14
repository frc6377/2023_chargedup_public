package frc.robot.subsystems.fieldPositioningSystem;

import com.ctre.phoenix.sensors.Pigeon2;
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
  private Pigeon2 IMU;
  private Supplier<SwerveModulePosition[]> swerveOdemSupplier;
  private SwerveDriveOdometry swerveDriveOdometry;

  public FieldPositioningSystem() {
    new FPSHardware().configure(this);
  }

  /*
   * TODO: have the IMU feed into robot pose
   * TODO: change getCurrentRobotRotationXY to read from robot pose MUST BE AFTER IMU FEEDS POSE
   */

  /**
   * returns the current robot rotation on the X-Y plane computed from the gyro and vision data
   *
   * @return the current robot rotation on the X-Y plane as a Rotation2d
   */
  public Rotation2d getCurrentRobotRotationXY() {
    return Rotation2d.fromDegrees(IMU.getYaw());
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

  public void setPigeon2(Pigeon2 pigeon) {
    IMU = pigeon;
  }

  private void updateSwerveDriveOdemetry() {
    if (swerveOdemSupplier == null) {
      return;
    }

    SwerveModulePosition[] moduleStates = swerveOdemSupplier.get();

    swerveDriveOdometry.update(getCurrentRobotRotationXY(), moduleStates);
  }

  private class FPSHardware {

    public void configure(FieldPositioningSystem FPS) {
      Pigeon2 pig = new Pigeon2(0);
      FPS.setPigeon2(pig);
    }
  }
}
