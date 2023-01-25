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

public class SwerveAutoFactory {
  private final Pose2DSubscriber sub = Topics.PoseTopic().subscribe(new Pose2d());
  private final FieldPoses fieldPoses = new FieldPoses();
  public final double maxVelocity = Constants.autoMaxVelocity;
  public final double maxAcceleration = Constants.autoMaxAcceleration;
  private Consumer<Pose2d> poseReseter = null;
  private DrivetrainSubsystem drivetrainSubsystem;

  // loads trajectory from a file. Because this is usually invoked in auto we have the option to
  // reset our pose.
  public SwerveAutoFactory(Consumer<Pose2d> poseReseter, DrivetrainSubsystem drivetrainSubsystem) {
    this.poseReseter = poseReseter;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  public SwerveAutoFactory(DrivetrainSubsystem drivetrainSubsystem) {
    this.poseReseter = null;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  public Command generateCommand(String pathTofollow, boolean isFirstPath) {

    PathPlannerTrajectory trajectory =
        PathPlanner.loadPath(pathTofollow, maxVelocity, maxAcceleration);
    return generateControllerCommand(isFirstPath, trajectory);
  }

  // generates a trajectory on the fly from a given target pose
  public Command generateCommand(Pose2d targetPose) {
    Pose2d currentPose = sub.get();

    ArrayList<PathPoint> points = new ArrayList<PathPoint>();
    points.add(
        poseToPathPoint(
            currentPose,
            Math.hypot(
                drivetrainSubsystem.getChassisSpeeds().vxMetersPerSecond,
                drivetrainSubsystem.getChassisSpeeds()
                    .vyMetersPerSecond), headingBetweenPoses(currentPose, targetPose))); // current drivetrain velocity

    // Check the safe line. If we are not behind it, then choose to go
    // high or low based on where we are relative to the center of the
    // charge station.
    // if (false) {
    //   Pose2d safePose =
    //       (currentPose.getY() > FieldPoses.ChargeStationYCenter)
    //           ? fieldPoses.getUpperSafePoint()
    //           : fieldPoses.getLowerSafePoint();
    //   points.add(poseToPathPoint(safePose));
    // }

    SmartDashboard.putString(
        "target pose ",
        targetPose.getX() + " " + targetPose.getY() + " " + targetPose.getRotation().getDegrees());
    points.add(
        new PathPoint(
            targetPose.getTranslation(), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));

    var constraints = new PathConstraints(maxVelocity, maxAcceleration);
    var trajectory = PathPlanner.generatePath(constraints, points);
    System.out.println(trajectory.getEndState().poseMeters);

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
            drivetrainSubsystem);

    if (poseReseter != null && isFirstPath)
      poseReseter.accept(trajectory.getInitialHolonomicPose());
    drivetrainSubsystem.sendTrajectoryToNT(trajectory);
    System.out.println("total time " + trajectory.getTotalTimeSeconds());
    return command.andThen(
        new InstantCommand(() -> drivetrainSubsystem.drive(new ChassisSpeeds())));
  }

  private PathPoint poseToPathPoint(Pose2d pose, double velocityOverride, Rotation2d heading) {

    // translation, rotation (direction of the path), holonomic rotation (where the robot is
    // facing), velocity override (initial velo)
    return new PathPoint(
        pose.getTranslation(), heading, pose.getRotation(), velocityOverride);
  }

  private Rotation2d headingBetweenPoses(Pose2d pose1, Pose2d pose2){
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
}
