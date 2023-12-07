package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.networktables.Pose2DSubscriber;
import frc.robot.networktables.Topics;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.function.Consumer;

public class SwerveAutoFactoryPP24 {
  private final Pose2DSubscriber pose2dSubscriber = Topics.PoseTopic().subscribe(new Pose2d());
  private final Consumer<Pose2d> poseResetter;

  public SwerveAutoFactoryPP24(
      final Consumer<Pose2d> poseResetter, final DrivetrainSubsystem drivetrainSubsystem) {
    this.poseResetter = poseResetter;
  }
}
