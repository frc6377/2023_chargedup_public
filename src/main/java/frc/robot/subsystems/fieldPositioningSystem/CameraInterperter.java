package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

class CameraInterperter {
  private AprilTagFieldLayout aprilTags;
  private final PhotonCamera camera;
  private final double angularPosition;
  private final double distanceFromCenter;
  private final double yaw;

  public CameraInterperter(AprilTagFieldLayout aprilTags, CameraInformation cameraInformation) {
    this.aprilTags = aprilTags;
    camera = new PhotonCamera(cameraInformation.getName());
    this.angularPosition = cameraInformation.getTheta();
    this.distanceFromCenter = cameraInformation.getDistanceFromCenter();
    this.yaw = cameraInformation.getYaw();
  }

  public boolean hasTarget() {
    return camera.getLatestResult().hasTargets();
  }

  public List<MXPlusBLine> getPotentialPositionsLines(double robotHeading) {
    List<PhotonTrackedTarget> targets = camera.getLatestResult().targets;
    ArrayList<MXPlusBLine> lines = new ArrayList<>();

    targets.forEach(
        (x) -> {
          lines.add(getPotentialPositionsLine(robotHeading, x));
        });
    return lines;
  }
  // robotHeading is in Radians
  public MXPlusBLine getPotentialPositionsLine(double robotHeading, PhotonTrackedTarget target) {
    if (target.getFiducialId() > 8 || target.getFiducialId() < 1) return null;
    double aprilTagAngle = target.getYaw();
    double cameraAngleFrom0 = robotHeading + angularPosition;
    double cameraYawFrom0 = robotHeading + yaw;
    double slope = Math.tan(cameraYawFrom0 + aprilTagAngle);
    double aprilTagX = aprilTags.getTagPose(target.getFiducialId()).get().getX();
    double aprilTagY = aprilTags.getTagPose(target.getFiducialId()).get().getY();
    double cameraXRelativeToRobot = Math.cos(cameraAngleFrom0) * distanceFromCenter;
    double cameraYRelativeToRobot = Math.sin(cameraAngleFrom0) * distanceFromCenter;
    double b = aprilTagY - cameraYRelativeToRobot - (aprilTagX - cameraXRelativeToRobot) * slope;

    return new MXPlusBLine(slope, b);
  }
}
