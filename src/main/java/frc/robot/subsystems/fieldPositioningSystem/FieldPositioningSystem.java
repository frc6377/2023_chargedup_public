package frc.robot.subsystems.fieldPositioningSystem;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.networktables.Pose2DPublisher;
import frc.robot.networktables.Topics;
import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

public class FieldPositioningSystem extends SubsystemBase {

  private final Field2d field = new Field2d();
  final AprilTagFieldLayout aprilTagFieldLayout;

  private Pose2d currentRobotPose;
  private Pigeon2 inertialMeasurementUnit;
  private Supplier<SwerveModuleState[]> swerveOdomSupplier;
  private SwerveDrivePoseEstimator swerveDriveOdometry;
  private CameraInterperter leftCamera;
  private CameraInterperter rightCamera;

  private final Pose2DPublisher pub = Topics.PoseTopic().publish();
  private final DoubleArrayPublisher yprPub =
      NetworkTableInstance.getDefault().getDoubleArrayTopic("pitch").publish();
  private final DoubleArrayPublisher yprOmegaPub =
      NetworkTableInstance.getDefault().getDoubleArrayTopic("omegas").publish();

  class FieldPositioningSystemError extends RuntimeException {
    public FieldPositioningSystemError(final String message, final Throwable throwable) {
      super(message, throwable);
    }
  }

  public FieldPositioningSystem() {

    SmartDashboard.putBoolean("Use Rejection", true);
    final String aprilTagFileLocation =
        Filesystem.getDeployDirectory().getAbsolutePath() + "/2023-Apriltaglocation.json";

    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(aprilTagFileLocation);
    } catch (IOException e) {
      throw new FieldPositioningSystemError(
          "ERROR: Apriltag location file (" + aprilTagFileLocation + ") not found", e);
    }

    leftCamera = new CameraInterperter(aprilTagFieldLayout, FPSConfiguration.leftCamera);
    rightCamera = new CameraInterperter(aprilTagFieldLayout, FPSConfiguration.rightCamera);

    currentSwervePodPosition =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    SmartDashboard.putData("Field", field);

    currentRobotPose = new Pose2d();
    inertialMeasurementUnit = new Pigeon2(0);
    inertialMeasurementUnit.setYaw(0);
  }

  public Rotation2d getCurrentRobotRotationXY() {
    return currentRobotPose.getRotation();
  }

  public Pose2d getRobotXYPose() {
    return currentRobotPose;
  }

  private Rotation2d getRotionFromIMU() {
    return Rotation2d.fromDegrees(inertialMeasurementUnit.getYaw());
  }

  public void setDriveTrainSupplier(
      Supplier<SwerveModuleState[]> swerveOdomSupplier, SwerveDriveKinematics swerveKinematics) {
    if (swerveOdomSupplier == null) {
      return;
    }
    this.swerveOdomSupplier = swerveOdomSupplier;
    lastSwerveModuleUpdate = Timer.getFPGATimestamp();

    swerveDriveOdometry =
        new SwerveDrivePoseEstimator(
            swerveKinematics, new Rotation2d(), currentSwervePodPosition, new Pose2d());
    swerveDriveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 1));
  }

  @Override
  public void periodic() {
    updateSwerveDriveOdometry();

    if (swerveDriveOdometry != null) {
      currentRobotPose = swerveDriveOdometry.getEstimatedPosition();
    }
    field.setRobotPose(currentRobotPose);
    pub.accept(currentRobotPose);
    double[] ypr = new double[3];
    double[] yprVelocity = new double[3];
    inertialMeasurementUnit.getYawPitchRoll(ypr);
    inertialMeasurementUnit.getRawGyro(yprVelocity);
    ypr[0] = getRobotXYPose().getRotation().getDegrees();
    yprPub.accept(ypr);
    yprOmegaPub.accept(yprVelocity);
    List<MXPlusBLine> allPotentialPositionLines = leftCamera.getPotentialPositionsLines(ypr[0]);
    allPotentialPositionLines.addAll(rightCamera.getPotentialPositionsLines(ypr[0]));

    if (allPotentialPositionLines.size() >= 2
        && allPotentialPositionLines.get(0) != null
        && allPotentialPositionLines.get(1) != null) {
      Translation2d aprilTagEstimatedPose =
          allPotentialPositionLines.get(0).getIntersection(allPotentialPositionLines.get(1));
      field
          .getObject("Camera Position")
          .setPose(new Pose2d(aprilTagEstimatedPose, getCurrentRobotRotationXY()));
      swerveDriveOdometry.addVisionMeasurement(
          new Pose2d(aprilTagEstimatedPose, getCurrentRobotRotationXY()), Timer.getFPGATimestamp());
    }
  }

  public void setPigeon2(Pigeon2 pigeon) {
    inertialMeasurementUnit = pigeon;
  }

  public void setCameras(CameraInterperter[] cameras) {}

  public void resetIMU() {
    switch (DriverStation.getAlliance()) {
      case Blue:
        inertialMeasurementUnit.setYaw(-90);
        break;
      case Red:
        inertialMeasurementUnit.setYaw(90);
        break;
      case Invalid:
      default:
        inertialMeasurementUnit.setYaw(0);
        break;
    }
  }

  public void resetRobotPosition(Pose2d robotPosition) {
    swerveDriveOdometry.resetPosition(getRotionFromIMU(), currentSwervePodPosition, robotPosition);
  }

  private SwerveModulePosition[] currentSwervePodPosition;
  private double lastSwerveModuleUpdate;

  private void updateSwerveDriveOdometry() {
    if (swerveOdomSupplier == null) {
      return;
    }

    SwerveModuleState[] moduleStates = swerveOdomSupplier.get();

    double dtSeconds = Timer.getFPGATimestamp() - lastSwerveModuleUpdate;

    double deltaDistancePod0 = moduleStates[0].speedMetersPerSecond * dtSeconds;
    double deltaDistancePod1 = moduleStates[1].speedMetersPerSecond * dtSeconds;
    double deltaDistancePod2 = moduleStates[2].speedMetersPerSecond * dtSeconds;
    double deltaDistancePod3 = moduleStates[3].speedMetersPerSecond * dtSeconds;

    double pod0Distance = currentSwervePodPosition[0].distanceMeters + deltaDistancePod0;
    double pod1Distance = currentSwervePodPosition[1].distanceMeters + deltaDistancePod1;
    double pod2Distance = currentSwervePodPosition[2].distanceMeters + deltaDistancePod2;
    double pod3Distance = currentSwervePodPosition[3].distanceMeters + deltaDistancePod3;

    currentSwervePodPosition[0] = new SwerveModulePosition(pod0Distance, moduleStates[0].angle);
    currentSwervePodPosition[1] = new SwerveModulePosition(pod1Distance, moduleStates[1].angle);
    currentSwervePodPosition[2] = new SwerveModulePosition(pod2Distance, moduleStates[2].angle);
    currentSwervePodPosition[3] = new SwerveModulePosition(pod3Distance, moduleStates[3].angle);

    lastSwerveModuleUpdate = Timer.getFPGATimestamp();

    swerveDriveOdometry.update(
        Rotation2d.fromDegrees(inertialMeasurementUnit.getYaw()), currentSwervePodPosition);
  }
}
