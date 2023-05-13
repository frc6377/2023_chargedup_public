package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

class CameraInterperter {
  private final PhotonCamera camera;
  final AprilTagFieldLayout fieldTags;
  private Transform3d robotToCamera;
  private Pose3d estiamtedPose;
  private final PhotonPoseEstimator fallBack;
  private double lastAmbiguity;

  private FieldObject2d cameraPositionFieldObject;

  public CameraInterperter(
      Transform3d cameraPosition, AprilTagFieldLayout fieldTags, String cameraName) {
    camera = new PhotonCamera(cameraName);
    this.robotToCamera = cameraPosition;
    this.fieldTags = fieldTags;
    fallBack =
        new PhotonPoseEstimator(fieldTags, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCamera);
  }

  public void setCameraFieldObject(FieldObject2d cameraFieldObject2d) {
    cameraPositionFieldObject = cameraFieldObject2d;
    System.out.println("Enabled Debug on camera:" + camera.getName());
  }

  public void setReferncePose(Pose3d refPose) {
    estiamtedPose = refPose;
  }

  public Optional<EstimatedRobotPose> measure() {
    lastAmbiguity = 0;
    var rawData = camera.getLatestResult();
    Optional<EstimatedRobotPose> guessedPose;

    if (rawData.targets.size() == 0) {
      return Optional.empty();
    }

    if (SmartDashboard.getBoolean("Use Rejection", false)) {
      var filteredData = filterInput(rawData);

      if (filteredData == null || filteredData.targets.size() == 0) {
        return Optional.empty();
      }
      System.out.print(filteredData.targets.size());
      guessedPose = multiTagPNPStrategy(filteredData);
      for (PhotonTrackedTarget target : filteredData.targets) {
        lastAmbiguity += target.getPoseAmbiguity();
      }
      lastAmbiguity /= filteredData.targets.size();
      System.out.println(guessedPose.isPresent());
    } else {
      guessedPose = multiTagPNPStrategy(rawData);
      for (PhotonTrackedTarget target : rawData.targets) {
        lastAmbiguity += target.getPoseAmbiguity();
        lastAmbiguity /= rawData.targets.size();
      }
    }

    if (cameraPositionFieldObject != null) {
      if (guessedPose.isPresent())
        cameraPositionFieldObject.setPose(guessedPose.get().estimatedPose.toPose2d());
    }

    // if(filteredData.targets.size() != 0)
    // lastAmbiguity /= filteredData.targets.size();
    return scrubRotation(guessedPose);
  }

  private Optional<EstimatedRobotPose> scrubRotation(Optional<EstimatedRobotPose> in) {
    if (in.isEmpty()) return in;
    final EstimatedRobotPose initalPose = in.get();
    Pose3d position = initalPose.estimatedPose;
    Pose3d scrubed = new Pose3d(position.getTranslation(), estiamtedPose.getRotation());
    return Optional.of(
        new EstimatedRobotPose(scrubed, initalPose.timestampSeconds, initalPose.targetsUsed));
  }

  private PhotonPipelineResult filterInput(PhotonPipelineResult rawData) {
    PhotonPipelineResult filter = rawData;
    filter = ambiguityFilter(filter);
    filter = visibityFilter(filter);
    filter = minTargets(filter);
    if (filter != null) filter.setTimestampSeconds(rawData.getTimestampSeconds());
    return filter;
  }

  private PhotonPipelineResult minTargets(PhotonPipelineResult rawData) {
    if (rawData.targets.size() >= 2) return rawData;
    else {
      return null;
    }
  }

  private PhotonPipelineResult ambiguityFilter(PhotonPipelineResult rawData) {
    ArrayList<PhotonTrackedTarget> accepted = new ArrayList<>();

    for (PhotonTrackedTarget target : rawData.getTargets()) {
      if (target.getPoseAmbiguity() <= 0.2) {
        accepted.add(target);
      }
    }

    return new PhotonPipelineResult(rawData.getLatencyMillis(), accepted);
  }

  private PhotonPipelineResult visibityFilter(PhotonPipelineResult rawData) {
    double xMinLimit = fieldTags.getTagPose(1).get().getX();
    double xSubstationMinLimit = fieldTags.getTagPose(4).get().getX();
    double xMaxLimit = fieldTags.getTagPose(8).get().getX();
    double xSubstationMaxLimit = fieldTags.getTagPose(5).get().getX();

    ArrayList<PhotonTrackedTarget> accepted = new ArrayList<>();

    for (PhotonTrackedTarget target : rawData.getTargets()) {
      int targetID = target.getFiducialId();
      try {
        Pose3d targetPosition = fieldTags.getTagPose(targetID).get();

        Pose3d estimatedRobotPose =
            targetPosition
                .transformBy(target.getBestCameraToTarget().inverse())
                .transformBy(robotToCamera.inverse());

        if ((targetID == 4 || targetID == 5) && estimatedRobotPose.getX() < xSubstationMinLimit
            || estimatedRobotPose.getX() > xSubstationMaxLimit) {
          accepted.add(target);
        } else if (estimatedRobotPose.getX() < xMinLimit || estimatedRobotPose.getX() > xMaxLimit) {
          accepted.add(target);
        } else {
          System.out.println("REJECTED");
        }
      } catch (NoSuchElementException e) {
      }
    }
    return new PhotonPipelineResult(rawData.getLatencyMillis(), accepted);
  }

  private Optional<EstimatedRobotPose> multiTagPNPStrategy(PhotonPipelineResult result) {
    // Arrays we need declared up front
    var visCorners = new ArrayList<TargetCorner>();
    var knownVisTags = new ArrayList<AprilTag>();
    ArrayList<Pose3d> fieldToCams = new ArrayList<Pose3d>();
    ArrayList<Pose3d> fieldToCamsAlt = new ArrayList<Pose3d>();

    if (result.getTargets().size() < 2) {
      // Run fallback strategy instead
      return fallBack.update(result);
    }

    for (var target : result.getTargets()) {
      visCorners.addAll(target.getDetectedCorners());

      var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
      if (tagPoseOpt.isEmpty()) {
        // reportFiducialPoseError(target.getFiducialId());
        continue;
      }

      var tagPose = tagPoseOpt.get();

      // actual layout poses of visible tags -- not exposed, so have to recreate
      knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));

      fieldToCams.add(tagPose.transformBy(target.getBestCameraToTarget().inverse()));
      fieldToCamsAlt.add(tagPose.transformBy(target.getAlternateCameraToTarget().inverse()));
    }

    var cameraMatrixOpt = camera.getCameraMatrix();
    var distCoeffsOpt = camera.getDistCoeffs();
    boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();

    // multi-target solvePNP
    if (hasCalibData) {
      var cameraMatrix = cameraMatrixOpt.get();
      var distCoeffs = distCoeffsOpt.get();
      var pnpResults =
          VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
      var best =
          new Pose3d()
              .plus(pnpResults.best) // field-to-camera
              .plus(robotToCamera.inverse()); // field-to-robot

      return Optional.of(
          new EstimatedRobotPose(best, result.getTimestampSeconds(), result.getTargets()));
    } else {
      // TODO fallback strategy? Should we just always do solvePNP?
      return Optional.empty();
    }
  }

  public double getAmbiguity() {

    return lastAmbiguity * 5;
  }
}
