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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldPoses;
import frc.robot.networktables.Pose2DSubscriber;
import frc.robot.networktables.Topics;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.ArrayList;

public class SwerveAutoCommand extends SequentialCommandGroup {
  Pose2DSubscriber sub = Topics.PoseTopic().subscribe(new Pose2d());
  private final FieldPoses fieldPoses = new FieldPoses();

  public SwerveAutoCommand(
      String pathTofollow, DrivetrainSubsystem drivetrainSubsystem, Boolean isFirstPath) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathTofollow, 4.5, 2.5);

    fredrick(trajectory, drivetrainSubsystem, isFirstPath);
  }

  private boolean behindSafeLine(double xPosition) {

    if (fieldPoses.isRed()) {
      return xPosition > FieldPoses.RedSafeLineX;
    }

    return xPosition < FieldPoses.BlueSafeLineX;
  }

  public final double MaxVelocity = 4.5;
  public final double MaxAcceleration = 2.5;

  public SwerveAutoCommand(Pose2d targetPose, DrivetrainSubsystem drivetrainSubsystem) {
    var currentPose = sub.get();

    var points = new ArrayList<PathPoint>();
    points.add(
        new PathPoint(
            currentPose.getTranslation(),
            currentPose.getRotation(),
            currentPose.getRotation(),
            Math.hypot(
                drivetrainSubsystem.getChassisSpeeds().vxMetersPerSecond,
                drivetrainSubsystem.getChassisSpeeds().vyMetersPerSecond)));

    // Check the safe line. If we are not behind it, then choose to go
    // high or low based on where we are relative to the center of the
    // charge station.
    if (false) {
      if (currentPose.getY() > FieldPoses.ChargeStationYCenter) {
        points.add(
            new PathPoint(
                fieldPoses.getUpperSafePoint().getTranslation(),
                fieldPoses.getUpperSafePoint().getRotation()));
      } else {
        points.add(
            new PathPoint(
                fieldPoses.getLowerSafePoint().getTranslation(),
                fieldPoses.getLowerSafePoint().getRotation()));
      }
    }
    SmartDashboard.putString("target pose ", targetPose.getX() + " " + targetPose.getY() + " " + targetPose.getRotation().getDegrees());
    points.add(
        new PathPoint(
            targetPose.getTranslation(), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));

    var constraints = new PathConstraints(MaxVelocity, MaxAcceleration);
    var trajectory = PathPlanner.generatePath(constraints, points);
    System.out.println(trajectory.getEndState().poseMeters);

    fredrick(trajectory, drivetrainSubsystem, false);
  }

  private  void fredrick(
      PathPlannerTrajectory trajectory,
      DrivetrainSubsystem drivetrainSubsystem,
      Boolean isFirstPath) {

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

    // TODO: write to FPS
    // if (isFirstPath) drivetrainSubsystem.resetPose(trajectory.getInitialPose());
    drivetrainSubsystem.sendTrajectoryToNT(trajectory);
    System.out.println("total time " + trajectory.getTotalTimeSeconds());
    addCommands(
        command.andThen(new InstantCommand(() -> drivetrainSubsystem.drive(new ChassisSpeeds()))));
  }
}
