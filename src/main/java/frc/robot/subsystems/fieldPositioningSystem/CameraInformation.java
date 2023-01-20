package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

class CameraInformation {
  private String name;
  private Transform3d cameraLocation;
  /**
   * Coordinates are in meters relative to robot center.
   *
   * @param name Name of the camera in the PhotonUI
   * @param x x of the camera relative to robot center in meters
   * @param y y of the camera relative to robot center in meters
   * @param z z of the camera relative to robot center in meters
   * @param yaw yaw of the camera relative to robot center in radians
   * @param pitch pitch of the camera relative to robot center in radians
   */
  public CameraInformation(
      final String name,
      final double x,
      final double y,
      final double z,
      final double yaw,
      final double pitch,
      final double roll) {
    this.name = name;
    cameraLocation = new Transform3d(new Translation3d(x, y, z), new Rotation3d(Units.degreesToRadians(roll), pitch, yaw));
  }

  public String getName() {
    return name;
  }

  public Transform3d getCameraLocation() {
    return cameraLocation;
  }
}
