package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
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
  public static final double BlueSafeLineX = 2.8;
  public static final double RedSafeLineX = 14.2;
  public static final double ChargeStationYCenter = 2.8;

  public static final Translation2d rightFarInflectionPoint;
  public static final Translation2d rightCloseInflectionPoint;

  public static final Translation2d rightStationFarInflectionPoint;
  public static final Translation2d rightStationCloseInflectionPoint;

  public static final Translation2d leftFarInflectionPoint;
  public static final Translation2d leftCloseInflectionPoint;

  public static final Translation2d leftStationFarInflectionPoint;
  public static final Translation2d leftStationCloseInflectionPoint;


  private static final Translation2d[] bay = new Translation2d[9];
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
     this.isRed = isRed();

    // The following values are coordinate points describing the y-locations of the individual
    // grids. They are ordered assuming we are on the Blue team.
    List<Translation2d> relativeBays = Constants.deliveryBays;

    // We reverse the grid locations if we are on the Red team.
    Rotation2d rotation;
    if (isRed) {
      Collections.reverse(relativeBays);
    }

    createBays(relativeBays);

 
    var safeXPose = isRed ? 14.24 : 2.4;
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

  public Translation2d getBay(final int bayNumber) {
    return bay[bayNumber];
  }

  public Pose2d getUpperSafePoint() {
    return upperSafePoint;
  }

  public Pose2d getLowerSafePoint() {
    return lowerSafePoint;
  }

  private void createBays (List<Translation2d> relativeBays) {
    if (isRed){
      for (int i = 0; i < relativeBays.size(); i++){
        bay[i] = relativeToAbsolute(relativeBays.get(i));
      }
    }
  }
  private Translation2d relativeToAbsolute (Translation2d translation){
    Translation2d absTranslation = translation;

    if (isRed) {
      translation = new Translation2d(Constants.fieldX - translation.getX(), translation.getY());
    }

    return translation;
  }

  public void createInflectionPoints (){

    
  }
}
