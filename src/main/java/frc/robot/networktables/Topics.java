package frc.robot.networktables;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Topics {
  private static final String datatable = "datatable";
  private static final String poseTopic = "/datatable/pose";

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private Topics() {
    inst.getTable(datatable);
  }

  public static Pose2DTopic PoseTopic() {
    var topics = new Topics();
    return Pose2DTopic.get(topics.inst, poseTopic);
  }

  public static Pose2DTopic PoseTopic(String name) {
    var topics = new Topics();
    return Pose2DTopic.get(topics.inst, name);
  }
}
