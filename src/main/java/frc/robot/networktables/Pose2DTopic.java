package frc.robot.networktables;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.EntryBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;

public class Pose2DTopic {
  private final DoubleArrayTopic internalTopic;

  private Pose2DTopic(DoubleArrayTopic topic) {
    internalTopic = topic;
  }

  public static Pose2DTopic get(NetworkTableInstance inst, String name) {
    var topic = inst.getDoubleArrayTopic(name);
    return new Pose2DTopic(topic);
  }

  public Pose2DSubscriber subscribe(Pose2d defaultValue, PubSubOption... options) {
    var entry = internalTopic.getEntry(toDoubleArray(defaultValue), options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  public Pose2DSubscriber subscribeEx(
      String typeString, Pose2d defaultValue, PubSubOption... options) {
    var entry = internalTopic.getEntryEx(typeString, toDoubleArray(defaultValue), options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  public Pose2DPublisher publish(PubSubOption... options) {
    var entry = internalTopic.getEntry(new double[] {}, options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  public Pose2DEntry getEntry(Pose2d defaultValue, PubSubOption... options) {
    var entry = internalTopic.getEntry(toDoubleArray(defaultValue), options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  public Pose2DEntry getEntryEx(String typeString, Pose2d defaultValue, PubSubOption... options) {
    var entry = internalTopic.getEntryEx(typeString, toDoubleArray(defaultValue), options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  private double[] toDoubleArray(Pose2d point) {
    return new double[] {point.getX(), point.getY(), point.getRotation().getRadians()};
  }

  private class Pose2DEntryImpl extends EntryBase implements Pose2DEntry {
    private final Topic internalTopic;
    private final DoubleArrayEntry internalEntry;

    public Pose2DEntryImpl(Topic topic, DoubleArrayEntry entry) {
      super(entry.getHandle());
      internalTopic = topic;
      internalEntry = entry;
    }

    /*
     * WARNING: This topic will not work as expected. It will
     * return a DoubleArrayTopic instead of a Pose2DTopic.
     *
     * Currently, there is no way around this issue.
     */
    @Override
    public Topic getTopic() {
      return internalTopic;
    }

    @Override
    public Pose2d get() {
      var values = internalEntry.get();
      return new Pose2d(values[0], values[1], new Rotation2d(values[2]));
    }

    @Override
    public void accept(Pose2d t) {
      var values = new double[] {t.getX(), t.getY(), t.getRotation().getRadians()};
      internalEntry.accept(values);
    }
  }
}
