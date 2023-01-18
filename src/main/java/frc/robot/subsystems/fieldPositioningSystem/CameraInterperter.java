package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

class CameraInterperter {
  private Pose3d[] lastMeasuredPoseAtTagID = new Pose3d[9];
  private final Pose3d cameraPosition;
  private final AprilTagFieldLayout aprilTagLayout;
  private final PhotonCamera photonCamera;
  private final Supplier<Rotation2d> yawSupplier;

  public CameraInterperter(
      Pose3d cameraPosition, AprilTagFieldLayout aprilTagLayout, String cameraName, Supplier<Rotation2d> yaw) {
    this.cameraPosition = cameraPosition;
    this.aprilTagLayout = aprilTagLayout;
    yawSupplier = yaw;
    photonCamera = new PhotonCamera(cameraName);
  }

  public VisionMeasurement[] measure() {
    AprilTagRead[] aprilTagRelativeLocations = readAprilTags();
    if (aprilTagRelativeLocations == null) return new VisionMeasurement[0];

    LinkedList<VisionMeasurement> visionMeasurements = new LinkedList<>();

    for (AprilTagRead apriltagRead : aprilTagRelativeLocations) {
      int tagID = apriltagRead.id;
      Pose3d aprilTagAbsoluteLocation = aprilTagLayout.getTagPose(tagID).get();

      Pose3d cameraPos = calculateCameraPoseOnField(aprilTagAbsoluteLocation, apriltagRead.tag);

      Pose3d robotPose = calculateRobotPositionFromCameraPosition(cameraPos);

      double stDev = calculateSTDevOfMeasure(robotPose, lastMeasuredPoseAtTagID[tagID - 1]);

      VisionMeasurement measurement =
          new VisionMeasurement(robotPose, stDev, apriltagRead.timeRecorded);
      visionMeasurements.add(measurement);

      lastMeasuredPoseAtTagID[tagID - 1] = robotPose;
    }

    return visionMeasurements.toArray(new VisionMeasurement[0]);
  }

  private double lastestTimeStamp = 0;

  private AprilTagRead[] readAprilTags() {
    PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
    if (!pipelineResult.hasTargets() || lastestTimeStamp == pipelineResult.getTimestampSeconds())
      return null;
    lastestTimeStamp = pipelineResult.getTimestampSeconds();

    List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
    LinkedList<AprilTagRead> output = new LinkedList<>();

    for (PhotonTrackedTarget target : targets) {
      output.add(
          new AprilTagRead(
              target.getBestCameraToTarget(),
              target.getFiducialId(),
              pipelineResult.getTimestampSeconds()));
    }

    return output.toArray(new AprilTagRead[0]);
  }

  private Pose3d calculateCameraPoseOnField(
      Pose3d tagAbsolutePositionOnField, final Transform3d tagRelativeLocationToCamera) {
    final Translation3d tagTranslation3d = tagRelativeLocationToCamera.getTranslation();
    final Translation3d rotatedTagLocation = tagTranslation3d.rotateBy(new Rotation3d(0, 0, yawSupplier.get().getRadians()));
    final Transform3d tagOffset = new Transform3d(rotatedTagLocation, tagRelativeLocationToCamera.getRotation());
    Pose3d cameraPose3d = tagAbsolutePositionOnField.plus(tagOffset);

    return cameraPose3d;
  }

  private Pose3d calculateRobotPositionFromCameraPosition(Pose3d cameraComputedLocation) {
    Transform3d subtraction = new Pose3d().minus(cameraPosition);
    Pose3d cameraPose3d = cameraComputedLocation.transformBy(subtraction);

    return cameraPose3d;
  }

  private double calculateSTDevOfMeasure(Pose3d robotPose, Pose3d previousMeasure) {
    return 0.05;
  }

  private class AprilTagRead {
    public final Transform3d tag;
    public final double timeRecorded;
    public final int id;

    public AprilTagRead(Transform3d tag, int id, double timeRecorded) {
      this.tag = tag;
      this.id = id;
      this.timeRecorded = timeRecorded;
    }
  }
}
