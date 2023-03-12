package frc.robot.subsystems.fieldPositioningSystem;

class FPSConfiguration {
  public static final double CAMER_OUTLIER_DISTANCE = 3;
  public CameraInformation[] cameraInformations = {
    // new CameraInformation("Test1", 0, 0.33, 0.457, 0, 0, -2.7)
    new CameraInformation(
        "robot-left",
        -0.195,
        0.193,
        .24,
        Math.toRadians(32.87),
        Math.toRadians(58.39),
        Math.toRadians(0))
  };
}
