package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Collections;
import java.util.List;

/**
 * Team membership is unknown until network tables are available. This class allows the system to
 * query for team membership with lazy initialization via the isRed() method. Additionally, this
 * class sets the expected relative position of field elements as calculated based on team
 * membership and known positions of known field elements.
 */
public class FieldPoses {

  private boolean isRed;

  public final double closeZoneBoundary;
  public final double midZoneBoundary;
  public final double farZoneBoundary;

  public final double rightZoneBoundary = Constants.rightZoneBoundary;
  public final double rightStationZoneBoundary = Constants.rightStationZoneBoundary;
  public final double leftZoneBoundary = Constants.leftZoneBoundary;
  public final double leftStationZoneBoundary = Constants.leftStationZoneBoundary;

  public final Translation2d rightFarInflectionPoint;
  public final Translation2d rightCloseInflectionPoint;

  public final Translation2d rightStationFarInflectionPoint;
  public final Translation2d rightStationCloseInflectionPoint;

  public final Translation2d leftFarInflectionPoint;
  public final Translation2d leftCloseInflectionPoint;

  public final Translation2d leftStationFarInflectionPoint;
  public final Translation2d leftStationCloseInflectionPoint;

  private static final Translation2d[] bay = new Translation2d[9];

  private final Translation2d singleSubstation;
  private final Translation2d doubleSubstation;

  class NullAllianceException extends RuntimeException {}

  // getters
  public final Translation2d GetBay(int idx) {
    return bay[idx];
  }

  public final Translation2d getRightFarInflectionPoint() {
    return rightFarInflectionPoint;
  }

  public final Translation2d getRightCloseInflectionPoint() {
    return rightCloseInflectionPoint;
  }

  public final Translation2d getRightStationFarInflectionPoint() {
    return rightStationFarInflectionPoint;
  }

  public final Translation2d getRightStationCloseInflectionPoint() {
    return rightStationCloseInflectionPoint;
  }

  public final Translation2d getLeftFarInflectionPoint() {
    return leftFarInflectionPoint;
  }

  public final Translation2d getLeftCloseInflectionPoint() {
    return leftCloseInflectionPoint;
  }

  public final Translation2d getLeftStationFarInflectionPoint() {
    return leftStationFarInflectionPoint;
  }

  public final double getRightZoneBoundary() {
    return rightZoneBoundary;
  }

  public final double getRightStationZoneBoundary() {
    return rightStationZoneBoundary;
  }

  public final double getLeftZoneBoundary() {
    return leftZoneBoundary;
  }

  public final double getCloseZoneBoundary() {
    return closeZoneBoundary;
  }

  public final double getMidZoneBoundary() {
    return midZoneBoundary;
  }

  public final double getFarZoneBoundary() {
    return farZoneBoundary;
  }

  public final Translation2d getSingleSubstation() {
    return singleSubstation;
  }

  public final Translation2d getDoubleSubstation() {
    return doubleSubstation;
  }

  public FieldPoses() {
    this.isRed = isRed();

    // The following values are coordinate points describing the y-locations of the individual
    // grids. They are ordered assuming we are on the Blue team.
    List<Translation2d> relativeBays = Constants.deliveryBays;

    // initialize all attributes such that they are absolute
    createBays(relativeBays);

    singleSubstation = relativeToAbsolute(Constants.singleSubstation);
    doubleSubstation = relativeToAbsolute(Constants.doubleSubstation);

    rightFarInflectionPoint = relativeToAbsolute(Constants.rightFarInflectionPoint);
    rightCloseInflectionPoint = relativeToAbsolute(Constants.rightCloseInflectionPoint);

    rightStationFarInflectionPoint = relativeToAbsolute(Constants.rightStationFarInflectionPoint);
    rightStationCloseInflectionPoint =
        relativeToAbsolute(Constants.rightStationCloseInflectionPoint);

    leftFarInflectionPoint = relativeToAbsolute(Constants.leftFarInflectionPoint);
    leftCloseInflectionPoint = relativeToAbsolute(Constants.leftCloseInflectionPoint);

    leftStationFarInflectionPoint = relativeToAbsolute(Constants.leftStationFarInflectionPoint);
    leftStationCloseInflectionPoint = relativeToAbsolute(Constants.leftStationCloseInflectionPoint);

    closeZoneBoundary = relativeToAbsolute(Constants.closeZoneBoundary);
    midZoneBoundary = relativeToAbsolute(Constants.midZoneBoundary);
    farZoneBoundary = relativeToAbsolute(Constants.farZoneBoundary);
  }

  /*
   * Because networks tables are required to determine which side you are on. It is
   * important that we use lazy initialization.
   */
  public final boolean isRed() {
    // TODO: consider whether to also watch for DriverStation.isDSAttached() to be true
    // TODO: cache this?
    final Alliance alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Invalid) {
      throw new NullAllianceException();
    }
    return alliance == Alliance.Red;
  }

  public Translation2d getBay(final int bayNumber) {
    return bay[bayNumber];
  }

  private void createBays(List<Translation2d> relativeBays) {
    // because bays are ordered left to right they must be flipped based on alliance color
    if (isRed) {
      Collections.reverse(relativeBays);
    }
    for (int i = 0; i < relativeBays.size(); i++) {
      bay[i] = relativeToAbsolute(relativeBays.get(i));
    }
  }

  // mirrors a translation 2d object
  private Translation2d relativeToAbsolute(Translation2d translation) {

    if (isRed) {
      translation = new Translation2d(Constants.fieldX - translation.getX(), translation.getY());
    }

    return translation;
  }

  // mirrors a distance from the alliance wall
  private double relativeToAbsolute(double x) {
    if (isRed) {
      x = Constants.fieldX - x;
    }

    return x;
  }
}
