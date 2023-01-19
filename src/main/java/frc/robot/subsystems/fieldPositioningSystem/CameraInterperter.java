package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

class CameraInterperter {
  private Pose2d[] lastMeasuredPoseAtTagID = new Pose2d[9];
  private final Transform3d cameraPosition;
  private final AprilTagFieldLayout aprilTagLayout;
  private final PhotonCamera photonCamera;
  private final Supplier<Rotation2d> yawSupplier;
  ;

  private FieldObject2d cameraPositionFieldObject;
  private FieldObject2d robotPositionFieldObject;

  public CameraInterperter(
      Transform3d cameraPosition,
      AprilTagFieldLayout aprilTagLayout,
      String cameraName,
      Supplier<Rotation2d> yaw) {
    this.cameraPosition = cameraPosition;
    this.aprilTagLayout = aprilTagLayout;
    yawSupplier = yaw;
    photonCamera = new PhotonCamera(cameraName);
  }

  public void setCameraFieldObject(FieldObject2d cameraFieldObject2d) {
    cameraPositionFieldObject = cameraFieldObject2d;
  }

  public void setRobotFieldObject(FieldObject2d robotFieldObject2d) {
    robotPositionFieldObject = robotFieldObject2d;
  }

  public VisionMeasurement[] measure() {
    PhotonPipelineResult piplineResult = photonCamera.getLatestResult();
    List<PhotonTrackedTarget> targets = piplineResult.getTargets();
    LinkedList<VisionMeasurement> measurements = new LinkedList<>();

    for (PhotonTrackedTarget target : targets) {
      int tagID = target.getFiducialId();

      Pose3d targetFieldPosition;
      try {
        targetFieldPosition = aprilTagLayout.getTagPose(tagID).get();
      } catch (NullPointerException e) {
        continue;
      }

      Pose3d robotPosition =
          PhotonUtils.estimateFieldToRobotAprilTag(
              target.getBestCameraToTarget(), targetFieldPosition, cameraPosition);
      measurements.add(
          new VisionMeasurement(
              robotPosition.toPose2d(),
              target.getPoseAmbiguity(),
              piplineResult.getTimestampSeconds()));
    }

    return null;
  }

  // public VisionMeasurement[] measure() {
  //   AprilTagRead[] aprilTagRelativeLocations = readAprilTags();
  //   if (aprilTagRelativeLocations == null) return new VisionMeasurement[0];

  //   LinkedList<VisionMeasurement> visionMeasurements = new LinkedList<>();

  //   for (AprilTagRead apriltagRead : aprilTagRelativeLocations) {
  //     int tagID = apriltagRead.id;

  //     Pose3d aprilTagAbsoluteLocation;
  //     try{
  //       aprilTagAbsoluteLocation = aprilTagLayout.getTagPose(tagID).get();
  //     }catch(NullPointerException e){
  //       continue;
  //     }

  //     Pose2d cameraPos = calculateCameraPoseOnField(aprilTagAbsoluteLocation, apriltagRead.tag);

  //     if(cameraPositionFieldObject != null){
  //       cameraPositionFieldObject.setPose(cameraPos);
  //     }

  //     Pose2d robotPose = calculateRobotPositionFromCameraPosition(cameraPos);

  //     if(robotPositionFieldObject != null){
  //       robotPositionFieldObject.setPose(robotPose);
  //     }

  //     VisionMeasurement measurement =
  //         new VisionMeasurement(
  //           robotPose,
  //           apriltagRead.ambiguity,
  //           apriltagRead.timeRecorded);
  //     visionMeasurements.add(measurement);

  //     lastMeasuredPoseAtTagID[tagID - 1] = robotPose;
  //   }

  //   return visionMeasurements.toArray(new VisionMeasurement[0]);
  // }

  private double lastestTimeStamp = 0;

  /** Reads all the april tag this camera sees. */
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
              pipelineResult.getTimestampSeconds(),
              target.getPoseAmbiguity()));
    }

    return output.toArray(new AprilTagRead[0]);
  }

  private Pose2d calculateCameraPoseOnField(
      Pose3d tagAbsolutePositionOnField, final Transform3d tagRelativeLocationToCamera) {
    final Transform3d tagRelativeLocationToCamera2 =
        new Transform3d(
            tagRelativeLocationToCamera.getTranslation(), new Rotation3d(0, 0, Math.PI));
    Pose3d cameraPose3d = tagAbsolutePositionOnField.plus(tagRelativeLocationToCamera2.inverse());

    return cameraPose3d.toPose2d();
  }

  // private Pose2d calculateRobotPositionFromCameraPosition(Pose2d cameraComputedLocation) {
  //   Transform2d subtraction = new Pose2d().minus(cameraPosition.toPose2d());
  //   Pose2d cameraPose3d = cameraComputedLocation.transformBy(subtraction);

  //   return cameraPose3d;
  // }

  private class AprilTagRead {
    public final Transform3d tag;
    public final double timeRecorded;
    public final int id;
    public final double ambiguity;

    public AprilTagRead(Transform3d tag, int id, double timeRecorded, double ambiguity) {

      SmartDashboard.putNumber("rot", tag.getRotation().getAngle());
      this.tag = tag;
      this.id = id;
      this.timeRecorded = timeRecorded;
      this.ambiguity = ambiguity;
    }
  }
}
