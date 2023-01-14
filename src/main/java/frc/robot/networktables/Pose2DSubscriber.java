package frc.robot.networktables;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.Subscriber;
import java.util.function.Supplier;

public interface Pose2DSubscriber extends Subscriber, Supplier<Pose2d> {}
