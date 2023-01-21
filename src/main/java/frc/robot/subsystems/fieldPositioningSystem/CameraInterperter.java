package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

class CameraInterperter {
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator estimator;

  private FieldObject2d cameraPositionFieldObject;

  public CameraInterperter(
      Transform3d cameraPosition, AprilTagFieldLayout aprilTagLayout, String cameraName) {
    photonCamera = new PhotonCamera(cameraName);
    estimator =
        new PhotonPoseEstimator(
            aprilTagLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, photonCamera, cameraPosition);
  }

  public void setCameraFieldObject(FieldObject2d cameraFieldObject2d) {
    cameraPositionFieldObject = cameraFieldObject2d;
  }

  public Optional<VisionMeasurement> measure() {
    var potentialEstimatedPose = estimator.update();
    if (potentialEstimatedPose.isEmpty()) {
      return Optional.empty();
    }

    var estimatedPose = potentialEstimatedPose.get();
    VisionMeasurement output =
        new VisionMeasurement(
            estimatedPose.estimatedPose.toPose2d(), 0.1, estimatedPose.timestampSeconds);
    if (cameraPositionFieldObject != null) {
      cameraPositionFieldObject.setPose(output.measurement);
    }

    return Optional.of(output);
  }
}
