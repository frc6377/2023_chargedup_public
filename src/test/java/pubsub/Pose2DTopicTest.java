package pubsub;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.networktables.Topics;
import org.junit.jupiter.api.Test;

public class Pose2DTopicTest {
  @Test // marks this method as a test
  void publish_subscribe() {
    var expected = new Pose2d(6.0, 7.0, new Rotation2d(8.0));

    var topic = Topics.PoseTopic();
    var pub = topic.publish();
    var sub = topic.subscribe(new Pose2d());

    pub.accept(expected);
    var actual = sub.get();

    assertEquals(expected, actual);
  }
}
