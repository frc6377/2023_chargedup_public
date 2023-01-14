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

  /**
   * Create a new subscriber to the topic.
   *
   * <p>The subscriber is only active as long as the returned object is not closed.
   *
   * <p>Subscribers that do not match the published data type do not return any values. To determine
   * if the data type matches, use the appropriate Topic functions.
   *
   * @param defaultValue default value used when a default is not provided to a getter function
   * @param options subscribe options
   * @return subscriber
   */
  public Pose2DSubscriber subscribe(Pose2d defaultValue, PubSubOption... options) {
    var entry = internalTopic.getEntry(toDoubleArray(defaultValue), options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  /**
   * Create a new subscriber to the topic, with specified type string.
   *
   * <p>The subscriber is only active as long as the returned object is not closed.
   *
   * <p>Subscribers that do not match the published data type do not return any values. To determine
   * if the data type matches, use the appropriate Topic functions.
   *
   * @param typeString type string
   * @param defaultValue default value used when a default is not provided to a getter function
   * @param options subscribe options
   * @return subscriber
   */
  public Pose2DSubscriber subscribeEx(
      String typeString, Pose2d defaultValue, PubSubOption... options) {
    var entry = internalTopic.getEntryEx(typeString, toDoubleArray(defaultValue), options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  /**
   * Create a new publisher to the topic.
   *
   * <p>The publisher is only active as long as the returned object is not closed.
   *
   * <p>It is not possible to publish two different data types to the same topic. Conflicts between
   * publishers are typically resolved by the server on a first-come, first-served basis. Any
   * published values that do not match the topic's data type are dropped (ignored). To determine if
   * the data type matches, use the appropriate Topic functions.
   *
   * @param options publish options
   * @return publisher
   */
  public Pose2DPublisher publish(PubSubOption... options) {
    var entry = internalTopic.getEntry(new double[] {}, options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  /**
   * Create a new entry for the topic.
   *
   * <p>Entries act as a combination of a subscriber and a weak publisher. The subscriber is active
   * as long as the entry is not closed. The publisher is created when the entry is first written
   * to, and remains active until either unpublish() is called or the entry is closed.
   *
   * <p>It is not possible to use two different data types with the same topic. Conflicts between
   * publishers are typically resolved by the server on a first-come, first-served basis. Any
   * published values that do not match the topic's data type are dropped (ignored), and the entry
   * will show no new values if the data type does not match. To determine if the data type matches,
   * use the appropriate Topic functions.
   *
   * @param defaultValue default value used when a default is not provided to a getter function
   * @param options publish and/or subscribe options
   * @return entry
   */
  public Pose2DEntry getEntry(Pose2d defaultValue, PubSubOption... options) {
    var entry = internalTopic.getEntry(toDoubleArray(defaultValue), options);
    return new Pose2DEntryImpl(internalTopic, entry);
  }

  /**
   * Create a new entry for the topic, with specified type string.
   *
   * <p>Entries act as a combination of a subscriber and a weak publisher. The subscriber is active
   * as long as the entry is not closed. The publisher is created when the entry is first written
   * to, and remains active until either unpublish() is called or the entry is closed.
   *
   * <p>It is not possible to use two different data types with the same topic. Conflicts between
   * publishers are typically resolved by the server on a first-come, first-served basis. Any
   * published values that do not match the topic's data type are dropped (ignored), and the entry
   * will show no new values if the data type does not match. To determine if the data type matches,
   * use the appropriate Topic functions.
   *
   * @param typeString type string
   * @param defaultValue default value used when a default is not provided to a getter function
   * @param options publish and/or subscribe options
   * @return entry
   */
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
