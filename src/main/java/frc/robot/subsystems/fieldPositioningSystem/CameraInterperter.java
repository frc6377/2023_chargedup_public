package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    double aprilTagAngle = Math.toRadians(target.getYaw());
    double cameraAngleFrom0 = angularPosition + Math.toRadians(robotHeading);
    double cameraYawFrom0 = yaw + Math.toRadians(robotHeading);
    double slope = Math.tan(aprilTagAngle - cameraYawFrom0);
    double aprilTagX = aprilTags.getTagPose(target.getFiducialId()).get().getX();
    double aprilTagY = aprilTags.getTagPose(target.getFiducialId()).get().getY();
    double cameraXRelativeToRobot = Math.cos(cameraAngleFrom0) * distanceFromCenter;
    double cameraYRelativeToRobot = Math.sin(cameraAngleFrom0) * distanceFromCenter;
    double b = aprilTagY - cameraYRelativeToRobot - (aprilTagX - cameraXRelativeToRobot) * slope;
    return new MXPlusBLine(slope, b, new Pose2d(aprilTagX+cameraXRelativeToRobot,aprilTagY+cameraYRelativeToRobot, new Rotation2d(cameraYawFrom0 + aprilTagAngle)));
  }
}
