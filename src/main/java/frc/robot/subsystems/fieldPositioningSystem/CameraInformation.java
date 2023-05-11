package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraInformation {
  private String name;
  private double theta;
  private double distanceFromCenter;
  private double yaw;
  /**
   * Coordinates are in meters relative to robot center.
   *
   * @param name Name of the camera in the PhotonUI
   * @param x x of the camera relative to robot center in meters
   * @param y y of the camera relative to robot center in meters
   * @param yaw yaw of the camera relative to robot center in radians
   */

  public CameraInformation(
      final String name,
      final double x,
      final double y,
      final double yaw) {
    this.name = name;
    this.theta = Math.atan2(y,x);
    this.distanceFromCenter = Math.sqrt(x*x+y*y);
    this.yaw = yaw;
  }

  public String getName() {
    return name;
  }

  public double getTheta () {
    return theta;
  }

  public double getDistanceFromCenter () {
    return distanceFromCenter;
  }

  public double getYaw () {
    return yaw;
  }
}
