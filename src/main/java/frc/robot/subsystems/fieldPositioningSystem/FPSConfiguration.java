package frc.robot.subsystems.fieldPositioningSystem;

public class FPSConfiguration {
  public static final double CAMER_OUTLIER_DISTANCE = 3;
  public static final CameraInformation leftCamera =
    new CameraInformation(
        "robot-left",
        -0.195,
        0.193,
        Math.toRadians(32.87));
  public static final CameraInformation rightCamera =
    new CameraInformation(
        "robot-right",
        0.195,
        0.193,
        -Math.toRadians(32.87));
}
