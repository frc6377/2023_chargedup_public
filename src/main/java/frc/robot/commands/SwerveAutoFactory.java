package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.FieldPoses;
import frc.robot.networktables.Pose2DSubscriber;
import frc.robot.networktables.Topics;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.ArrayList;
import java.util.function.Consumer;

// TODO
// reseting pose breaks field north. May not be an issue with this class
// general tuning
// move magic numbers to constants

public class SwerveAutoFactory {
  private final Pose2DSubscriber sub = Topics.PoseTopic().subscribe(new Pose2d());
  private FieldPoses fieldPoses =
      null; // bad values if built on startup, so instead the generate command method populates this
  public final double maxVelocity = Constants.autoMaxVelocity;
  public final double maxAcceleration = Constants.autoMaxAcceleration;
  private Consumer<Pose2d> poseReseter = null;
  private DrivetrainSubsystem drivetrainSubsystem;

  public SwerveAutoFactory(Consumer<Pose2d> poseReseter, DrivetrainSubsystem drivetrainSubsystem) {
    this.poseReseter = poseReseter;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  // probably unecessary but it can stay for now
  public SwerveAutoFactory(DrivetrainSubsystem drivetrainSubsystem) {
    this.poseReseter = null;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  // loads a trajectory from file and hands it to the command generator
  public Command generateCommand(String pathTofollow, boolean isFirstPath) {

    createFieldPoses(); // create a field poses object if we dont have one already

    PathPlannerTrajectory trajectory =
        PathPlanner.loadPath(pathTofollow, maxVelocity, maxAcceleration);
    return generateControllerCommand(isFirstPath, trajectory);
  }

  // this constructor just calls the target pose constructor, except it lets you directly reference
  // a delivery bay
  public Command generateCommand(int bay) {
    createFieldPoses(); // create a field poses object if we dont have one already
    return generateCommand(fieldPoses.getBay(bay));
  }

  // generates a trajectory on the fly from a given target pose
  public Command generateCommand(Pose2d targetPose) {

    createFieldPoses(); // create a field poses object if we dont have one already
    Pose2d currentPose = sub.get();
    Pose2d firstTarget =
        targetPose; // this variable tracks the pose of the second control point in the trajectory
    // so that our first control point's heading will face it
    ArrayList<PathPoint> points = new ArrayList<PathPoint>();

    // checks if we are not past the charge station because this means we are pretty free to move
    // along the Y axis and also it means we need to align ourself with one of the stations
    // "channels"

    if (!pastChargeStation(
        currentPose
            .getX())) { // because we can move in the Y axis we can choose the fastest path to the
      // target
      Pose2d safePose =
          (targetPose.getY() > FieldPoses.ChargeStationYCenter)
              ? fieldPoses.getUpperSafePoint()
              : fieldPoses.getLowerSafePoint();

      double stationOffset =
          (fieldPoses.isRed()
              ? -3
              : 3); // pretty gross, will fix in refactor. basically instead of having 2 safe points
      // we just offset the safe point we have by the length of the station with some
      // extra tolerance
      Pose2d firstSafePoint =
          new Pose2d(safePose.getX() + stationOffset, safePose.getY(), safePose.getRotation());
      points.add(poseToPathPoint(firstSafePoint, -1, safePose.getRotation()));
      points.add(poseToPathPoint(safePose, -1, headingBetweenPoses(safePose, targetPose)));
      firstTarget = firstSafePoint;
    }

    // checks if we are between the wall and the station because that means we dont have freedom
    // along the Y axis
    else if (!behindSafeLine(currentPose.getX())) {
      Pose2d safePose =
          (currentPose.getY() > FieldPoses.ChargeStationYCenter)
              ? fieldPoses.getUpperSafePoint()
              : fieldPoses.getLowerSafePoint();
      Pose2d firstSafePoint =
          new Pose2d(safePose.getX(), currentPose.getY(), safePose.getRotation());
      points.add(poseToPathPoint(firstSafePoint, -1, safePose.getRotation()));
      firstTarget = firstSafePoint;
    }

    // constructs the first point last using the current drivetrain velocity
    points.add(
        0,
        poseToPathPoint(
            currentPose,
            Math.hypot(
                drivetrainSubsystem.getChassisSpeeds().vxMetersPerSecond,
                drivetrainSubsystem.getChassisSpeeds().vyMetersPerSecond),
            headingBetweenPoses(currentPose, firstTarget)));

    SmartDashboard.putString(
        "target pose ",
        targetPose.getX() + " " + targetPose.getY() + " " + targetPose.getRotation().getDegrees());
    points.add(
        new PathPoint(
            targetPose.getTranslation(),
            targetPose.getRotation().rotateBy(new Rotation2d(Math.PI)),
            targetPose.getRotation().rotateBy(new Rotation2d(Math.PI))));

    var constraints = new PathConstraints(maxVelocity, maxAcceleration);
    var trajectory = PathPlanner.generatePath(constraints, points);

    return generateControllerCommand(false, trajectory);
  }

  public Command generateControllerCommand(boolean isFirstPath, PathPlannerTrajectory trajectory) {

    PIDController thetaController = new PIDController(2, 0.1, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand command =
        new PPSwerveControllerCommand(
            trajectory,
            sub::get,
            drivetrainSubsystem.getKinematics(),
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            thetaController,
            drivetrainSubsystem::updateAutoDemand,
            false,
            drivetrainSubsystem);

    if (poseReseter != null
        && isFirstPath) // checks if we have a pose reseter and if we want to reset our pose. If we
      // do we want to overwrite whatever the kalman filter has. Mostly for auton
      poseReseter.accept(trajectory.getInitialHolonomicPose());
    drivetrainSubsystem.sendTrajectoryToNT(trajectory); // posts trajectory to dashboard

    // run the command and than stop the drivetrain. Just to make sure we arent moving at the end
    return command.andThen(
        new InstantCommand(() -> drivetrainSubsystem.drive(new ChassisSpeeds())));
  }

  // converts a Pose2d to a PathPoint
  private PathPoint poseToPathPoint(Pose2d pose, double velocityOverride, Rotation2d heading) {

    // translation, rotation (direction of the path), holonomic rotation (where the robot is
    // facing), velocity override (initial velo)
    return new PathPoint(pose.getTranslation(), heading, pose.getRotation(), velocityOverride);
  }

  // computes the angle between two poses. Used so points have headings that point to the next point
  private Rotation2d headingBetweenPoses(Pose2d pose1, Pose2d pose2) {
    double theta = Math.atan2(pose1.getY() - pose2.getY(), pose1.getX() - pose2.getX());
    return new Rotation2d(theta + Math.PI);
  }

  private boolean behindSafeLine(
      double xPosition) { // checks if we are behind the "safe line" which is defined as a line
    // perpendicular to the driver such that when we are behind said line we
    // cannot hit the charging station
    if (fieldPoses.isRed()) {
      return xPosition > FieldPoses.RedSafeLineX;
    }

    return xPosition < FieldPoses.BlueSafeLineX;
  }

  private boolean pastChargeStation(
      double xPosition) { // checks if we are behind the charge station
    if (fieldPoses.isRed()) {
      return xPosition > FieldPoses.RedSafeLineX - 2.7;
    }

    return xPosition < FieldPoses.BlueSafeLineX + 2.7;
  }

  // constructs a field poses object if we dont have already. This is done because if this object is
  // created on startup it is garbage
  private void createFieldPoses() {
    if (fieldPoses == null) {
      fieldPoses = new FieldPoses();
    }
  }
}
