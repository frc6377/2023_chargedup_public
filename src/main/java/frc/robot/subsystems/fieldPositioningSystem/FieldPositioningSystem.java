package frc.robot.subsystems.fieldPositioningSystem;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.networktables.Pose2DPublisher;
import frc.robot.networktables.Topics;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

public class FieldPositioningSystem extends SubsystemBase {

  private final Field2d field = new Field2d();
  final AprilTagFieldLayout aprilTagFieldLayout;

  private Pose2d currentRobotPose;
  private Pigeon2 inertialMeasurementUnit;
  private Supplier<SwerveModuleState[]> swerveOdomSupplier;
  private SwerveDriveOdometry swerveDriveOdometry;
  private CameraInterperter[] cameras;
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
    final String aprilTagFileLocation =
        Filesystem.getDeployDirectory().getAbsolutePath() + "/2023-Apriltaglocation.json";
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(aprilTagFileLocation);
    } catch (IOException e) {
      throw new FieldPositioningSystemError(
          "ERROR: Apriltag location file (" + aprilTagFileLocation + ") not found", e);
    }

    currentSwervePodPosition =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    SmartDashboard.putData("Field", field);

    new FPSHardware(this).configure(this, new FPSConfiguration());
    currentRobotPose = new Pose2d();
    enableCameraDebug(0);
  }

  private void enableCameraDebug(int targetCameraIndex) {
    if (targetCameraIndex >= cameras.length) return;
    final FieldObject2d rawCameraPose = field.getObject("Camera Pose ID:" + targetCameraIndex);

    cameras[targetCameraIndex].setCameraFieldObject(rawCameraPose);

    System.out.println("Starting Camera Debug for camera ID:" + targetCameraIndex);
  }

  /**
   * returns the current robot rotation on the X-Y plane computed from the gyro and vision data
   *
   * @return the current robot rotation on the X-Y plane as a Rotation2d
   */
  public Rotation2d getCurrentRobotRotationXY() {
    return currentRobotPose.getRotation();
  }

  /**
   * Get the current robot pos in the XY plane
   *
   * @return the current robot position
   */
  public Pose2d getRobotXYPose() {
    return currentRobotPose;
  }

  private Rotation2d getRotionFromIMU() {
    return Rotation2d.fromDegrees(inertialMeasurementUnit.getYaw());
  }

  /**
   * Configure what drive train to update odometry with.
   *
   * @param swerveOdomSupplier - supplier of serve module state
   * @param swerveKinematics - the robots swerve drive kinematic module
   */
  public void setDriveTrainSupplier(
      Supplier<SwerveModuleState[]> swerveOdomSupplier, SwerveDriveKinematics swerveKinematics) {
    if (swerveOdomSupplier == null) {
      return;
    }
    this.swerveOdomSupplier = swerveOdomSupplier;
    lastSwerveModuleUpdate = Timer.getFPGATimestamp();

    swerveDriveOdometry =
        new SwerveDriveOdometry(
            swerveKinematics,
            new Rotation2d(), // TODO: this and gyro angle should come from auto
            currentSwervePodPosition,
            new Pose2d());
    //  Matrix<N3, N1> stdevMatrix = VecBuilder.fill(0.05, 0.05, 0.05);
    // swerveDriveOdometry.setVisionMeasurementStdDevs(stdevMatrix);
  }

  /** Called every 20ms to provide update the robots position */
  @Override
  public void periodic() {
    updateSwerveDriveOdometry();
    doVisionEstimation();
    currentRobotPose = swerveDriveOdometry.getPoseMeters();
    field.setRobotPose(currentRobotPose);
    pub.accept(currentRobotPose);
    double[] ypr = new double[3];
    double[] yprVelocity = new double[3];
    inertialMeasurementUnit.getYawPitchRoll(ypr);
    inertialMeasurementUnit.getRawGyro(yprVelocity);
    ypr[0] = getRobotXYPose().getRotation().getDegrees();
    yprPub.accept(ypr);
    yprOmegaPub.accept(yprVelocity);
    SmartDashboard.putString("pose", currentRobotPose.getX() + " " + currentRobotPose.getY());
  }

  /**
   * Set the pigeon object to be used for a IMU
   *
   * @param pigeon - the pigeon to use
   */
  public void setPigeon2(Pigeon2 pigeon) {
    inertialMeasurementUnit = pigeon;
  }

  public void setCameras(CameraInterperter[] cameras) {
    this.cameras = cameras;
  }

  /** Reset the IMU in accordance to driver perspictive. */
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
        // TODO: add debug statement
        break;
    }
  }

  /**
   * Sets the robot position to a given input.
   *
   * @param robotPosition the target location in meters
   */
  public void resetRobotPosition(Pose2d robotPosition) {
    swerveDriveOdometry.resetPosition(getRotionFromIMU(), currentSwervePodPosition, robotPosition);
  }

  public void discardOutlierRejectionForNextIteration() {}

  /** Run vision estimation on cameras. */
  private void doVisionEstimation() {
    for (CameraInterperter interperter : cameras) {
      Optional<VisionMeasurement> potentialMeasurement = interperter.measure();
      if (potentialMeasurement.isEmpty()) continue;
      VisionMeasurement measurement = potentialMeasurement.get();

      // Matrix<N3, N1> stdevMatrix =
      //     VecBuilder.fill(measurement.stdev, measurement.stdev, measurement.stdev);
      Translation2d measuredLocation = measurement.measurement.getTranslation();

      final double correctionDistance =
          measuredLocation.getDistance(swerveDriveOdometry.getPoseMeters().getTranslation());

      if (correctionDistance > FPSConfiguration.CAMER_OUTLIER_DISTANCE) {
        continue;
      }

      //   swerveDriveOdometry.addVisionMeasurement(
      //     new Pose2d(measuredLocation, getRotionFromIMU()), measurement.timeRecorded);
    }

    Pose2d robotPose = swerveDriveOdometry.getPoseMeters();
    Pose3d robotPose3d = new Pose3d(robotPose);
    for (CameraInterperter camera : cameras) {
      camera.setReferncePose(robotPose3d);
    }
  }

  private SwerveModulePosition[] currentSwervePodPosition;
  private double lastSwerveModuleUpdate;

  /** update the odometry model with current information. */
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
