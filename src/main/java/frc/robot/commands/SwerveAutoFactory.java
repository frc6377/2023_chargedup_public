package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldPoses;
import frc.robot.networktables.Pose2DSubscriber;
import frc.robot.networktables.Topics;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.ArrayList;
import java.util.function.Consumer;

public class SwerveAutoFactory {
  private final Pose2DSubscriber sub = Topics.PoseTopic().subscribe(new Pose2d());
  private FieldPoses fieldPoses = null;
  public final double maxVelocity = Constants.AUTO_MAX_VELOCITY;
  public final double maxAcceleration = Constants.AUTO_MAX_ACCELERATION;
  private Consumer<Pose2d> poseReseter = null;
  private DrivetrainSubsystem drivetrainSubsystem;

  public SwerveAutoFactory(Consumer<Pose2d> poseReseter, DrivetrainSubsystem drivetrainSubsystem) {
    this.poseReseter = poseReseter;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  // Loads a trajectory from file and hands it to the command generator.
  public SequentialCommandGroup generateCommandFromFile(String pathTofollow, boolean isFirstPath) {
    return generateCommandFromFile(pathTofollow, isFirstPath, maxVelocity, maxAcceleration);
  }

  public SequentialCommandGroup generateCommandFromFile(
      String pathTofollow, boolean isFirstPath, double Velo, double accel) {
    createFieldPoses();
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathTofollow, Velo, accel);
    return generateControllerCommand(isFirstPath, trajectory);
  }

  // Generates a trajectory on the fly from a given target pose.
  public Command generateGridCommand(int targetBay) {

    createFieldPoses();
    Translation2d targetPose = fieldPoses.getBay(targetBay);

    Rotation2d deliveryRotation = fieldPoses.getDeliveryRotation();
    Pose2d currentPose = sub.get();

    Translation2d firstTarget = targetPose;
    ArrayList<PathPoint> points = new ArrayList<PathPoint>();

    // Checks if the robot ois not past the charge station.

    Zone currentZone = getZone(currentPose.getY());
    Proximity currentProximity = getProx(currentPose.getX());

    // Build midpoints.
    switch (currentProximity) {
      case CLOSE:
        break;
      case MID:
        Translation2d inflection =
            new Translation2d(
                fieldPoses.closeProximityBoundary,
                currentPose.getY()); // Cannot move in the y-axis at mid.
        points.add(
            poseToPathPoint(
                new Pose2d(
                    inflection, headingBetweenPoints(currentPose.getTranslation(), inflection)),
                -1,
                deliveryRotation));
        firstTarget = inflection;
        break;

      case FAR:
        Translation2d inflection1;
        Translation2d inflection2;

        if (targetBay <= 1) {
          inflection1 = fieldPoses.rightFarInflectionPoint;
          inflection2 = fieldPoses.rightCloseInflectionPoint;
        } else if (targetBay >= 7) {
          inflection1 = fieldPoses.leftFarInflectionPoint;
          inflection2 = fieldPoses.leftCloseInflectionPoint;
        } else {
          switch (currentZone) {
            case LEFT:
              inflection1 = fieldPoses.leftFarInflectionPoint;
              inflection2 = fieldPoses.leftCloseInflectionPoint;
              break;

            case LEFT_STATION:
              inflection1 = fieldPoses.leftStationFarInflectionPoint;
              inflection2 = fieldPoses.leftStationCloseInflectionPoint;
              break;

            case RIGHT:
              inflection1 = fieldPoses.rightFarInflectionPoint;
              inflection2 = fieldPoses.rightCloseInflectionPoint;
              break;

            case RIGHT_STATION:
              inflection1 = fieldPoses.rightStationFarInflectionPoint;
              inflection2 = fieldPoses.rightStationCloseInflectionPoint;
              break;

            default:
              System.out.println("NO ZONE FOUND");
              inflection1 = null;
              inflection2 = null;
          }
        }
        firstTarget = inflection1;
        points.add(
            poseToPathPoint(
                new Pose2d(
                    inflection1, headingBetweenPoints(currentPose.getTranslation(), inflection1)),
                -1,
                deliveryRotation));
        points.add(
            poseToPathPoint(
                new Pose2d(
                    inflection2, headingBetweenPoints(currentPose.getTranslation(), inflection2)),
                -1,
                deliveryRotation));
        break;
    }

    // Constructs the first point last using the current drivetrain velocity.
    points.add(
        0,
        poseToPathPoint(
            currentPose,
            Math.hypot(
                drivetrainSubsystem.getChassisSpeeds().vxMetersPerSecond,
                drivetrainSubsystem.getChassisSpeeds().vyMetersPerSecond),
            headingBetweenPoints(currentPose.getTranslation(), firstTarget)));

    points.add(
        new PathPoint(targetPose, deliveryRotation, deliveryRotation).withPrevControlLength(0.2));

    PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration);
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(constraints, points);

    return generateControllerCommand(false, trajectory);
  }

  private Command generateCommandFromPoint(Pose2d endPoint) {

    ArrayList<PathPoint> points = new ArrayList<PathPoint>();
    Pose2d currentPose = sub.get();

    points.add(
        poseToPathPoint(
            currentPose,
            Math.hypot(
                drivetrainSubsystem.getChassisSpeeds().vxMetersPerSecond,
                drivetrainSubsystem.getChassisSpeeds().vyMetersPerSecond),
            headingBetweenPoints(currentPose.getTranslation(), endPoint.getTranslation())));
    points.add(
        poseToPathPoint(
            endPoint,
            -1,
            headingBetweenPoints(currentPose.getTranslation(), endPoint.getTranslation())
                .rotateBy(new Rotation2d(Math.PI))));

    PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration);
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(constraints, points);

    return generateControllerCommand(false, trajectory);
  }

  public Command generateSingleSubCommand() {
    createFieldPoses();
    return generateCommandFromPoint(
        new Pose2d(fieldPoses.getSingleSubstation(), fieldPoses.getSingleSubRotation()));
  }

  public Command generateDoubleSubCommand() {
    createFieldPoses();
    return generateCommandFromPoint(
        new Pose2d(fieldPoses.getDoubleSubstation(), fieldPoses.getDeliveryRotation()));
  }

  public Command generateStrafeCommand() {
    createFieldPoses();

    Pose2d endPoint = new Pose2d(fieldPoses.getBay(0).getX(), 0, fieldPoses.getDeliveryRotation());

    ArrayList<PathPoint> points = new ArrayList<PathPoint>();
    Pose2d currentPose =
        new Pose2d(fieldPoses.getDoubleSubstation().getX(), 0, fieldPoses.getSingleSubRotation());

    points.add(
        poseToPathPoint(
            currentPose,
            Math.hypot(
                drivetrainSubsystem.getChassisSpeeds().vxMetersPerSecond,
                drivetrainSubsystem.getChassisSpeeds().vyMetersPerSecond),
            headingBetweenPoints(currentPose.getTranslation(), endPoint.getTranslation())));

    points.add(
        poseToPathPoint(
            endPoint,
            -1,
            headingBetweenPoints(currentPose.getTranslation(), endPoint.getTranslation())
                .rotateBy(new Rotation2d(Math.PI))));

    PathConstraints constraints = new PathConstraints(5000, 1000);
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(constraints, points);

    return generateControllerCommand(false, trajectory);
  }

  public SequentialCommandGroup generateControllerCommand(
      boolean isFirstPath, PathPlannerTrajectory trajectory) {

    PIDController thetaController = new PIDController(2, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command command =
        new PPSwerveControllerCommand(
            trajectory,
            sub::get,
            drivetrainSubsystem.getKinematics(),
            new PIDController(5, 0, 0),
            new PIDController(5, 0, 0),
            thetaController,
            drivetrainSubsystem::updateAutoDemand,
            false,
            drivetrainSubsystem);

    if (poseReseter != null
        && isFirstPath) { // Checks if we have a pose reseter and if we want to reset our pose.
      // If so, we want to overwrite whatever the Kalman filter has.
      command =
          new InstantCommand(
                  () -> {
                    System.out.println("Reset POSE!!");
                    poseReseter.accept(trajectory.getInitialHolonomicPose());
                  })
              .andThen(command);
    }
    command =
        new InstantCommand(() -> drivetrainSubsystem.sendTrajectoryToNT(trajectory))
            .andThen(command);
    command =
        new InstantCommand(
                () -> System.out.println("starting path----------------------------------"))
            .andThen(command);

    // Run the command and then stop the drivetrain.
    return command.andThen(
        new InstantCommand(() -> drivetrainSubsystem.drive(new ChassisSpeeds())));
  }

  private PathPoint poseToPathPoint(Pose2d pose, double velocityOverride, Rotation2d heading) {
    return new PathPoint(pose.getTranslation(), heading, pose.getRotation(), velocityOverride);
  }

  // Computes the angle between two poses.
  private Rotation2d headingBetweenPoints(Translation2d translation1, Translation2d translation2) {
    double theta =
        Math.atan2(
            translation1.getY() - translation2.getY(), translation1.getX() - translation2.getX());
    return new Rotation2d(theta + Math.PI);
  }

  private void createFieldPoses() {
    if (fieldPoses == null) {
      fieldPoses = new FieldPoses();
    }
  }

  private Proximity getProx(double x) {

    int mult = (fieldPoses.isRed()) ? -1 : 1;
    x *= mult;

    if (x < fieldPoses.closeProximityBoundary * mult) {
      return Proximity.CLOSE;
    }

    if (x < fieldPoses.midProximityBoundary * mult) {
      return Proximity.MID;
    }

    return Proximity.FAR;
  }

  private Zone getZone(double y) {

    int mult = (fieldPoses.isRed()) ? -1 : 1;
    y *= mult;

    if (y < fieldPoses.rightZoneBoundary * mult) {
      return Zone.RIGHT;
    }

    if (y < fieldPoses.rightStationZoneBoundary * mult) {
      return Zone.RIGHT_STATION;
    }

    if (y < fieldPoses.leftStationZoneBoundary * mult) {
      return Zone.LEFT_STATION;
    }

    return Zone.LEFT;
  }

  private enum Proximity {
    CLOSE,
    MID,
    FAR
  }

  private enum Zone {
    LEFT,
    LEFT_STATION,
    RIGHT,
    RIGHT_STATION
  }
}
