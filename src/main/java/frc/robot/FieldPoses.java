package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
import java.util.Collections;

/**
 * Team membership is unknown until network tables are available. This class allows the system to
 * query for team membership with lazy initialization via the isRed() method. Additionally, this
 * class sets the expected relative position of field elements as calculated based on team
 * membership and known positions of known field elements.
 */
public class FieldPoses {
  public static final double BlueSafeLineX = 2.3;
  public static final double RedSafeLineX = 14.2;
  public static final double ChargeStationYCenter = 2.8;

  private static final Pose2d[] bay = new Pose2d[9];
  private final Pose2d upperSafePoint; // point to avoid switch
  private final Pose2d lowerSafePoint; // other point to avoid switch

  class NullAllianceException extends RuntimeException {}

  public final Pose2d GetPose(int idx) {
    return bay[idx];
  }

  public final Pose2d UpperSafe() {
    return upperSafePoint;
  }

  public final Pose2d LowerSafe() {
    return lowerSafePoint;
  }

  public FieldPoses() {
    boolean isRed = isRed();

    var xPosition = isRed ? 2.9 : 14.6;

    // The following values are coordinate points describing the y-locations of the individual
    // grids. They are ordered assuming we are on the Blue team.
    var yPosition = Arrays.asList(0.55, 1.05, 1.65, 2.15, 2.75, 3.3, 3.85, 4.45, 4.98);

    // We reverse the grid locations if we are on the Red team.
    if (isRed) {
      Collections.reverse(yPosition);
    }
    var rotationValue = isRed ? 0 : Math.PI;
    var rotation = new Rotation2d(rotationValue);

    for (var i = 0; i < bay.length; i++) {
      bay[i] = new Pose2d(xPosition, yPosition.get(i), rotation);
    }

    var safeXPose = isRed ? 14.3 : 2.6;
    upperSafePoint = new Pose2d(safeXPose, 4.65, rotation);
    lowerSafePoint = new Pose2d(safeXPose, 0.9, rotation);
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

  public Pose2d getBay(final int bayNumber) {
    return bay[bayNumber];
  }

  public Pose2d getUpperSafePoint() {
    return upperSafePoint;
  }

  public Pose2d getLowerSafePoint() {
    return lowerSafePoint;
  }
}
