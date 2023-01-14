package frc.robot.networktables;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.Publisher;
import java.util.function.Consumer;

public interface Pose2DPublisher extends Publisher, Consumer<Pose2d> {}
