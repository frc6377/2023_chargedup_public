package frc.robot.subsystems.fieldPositioningSystem;

import com.ctre.phoenix.sensors.Pigeon2;

class FPSHardware {
  private final FieldPositioningSystem fieldPositioningSystem;

  /**
   * @param fieldPositioningSystem
   */
  FPSHardware(FieldPositioningSystem fieldPositioningSystem) {
    this.fieldPositioningSystem = fieldPositioningSystem;
  }

  public void configure(FieldPositioningSystem FPS, FPSConfiguration config) {
    configCameras(FPS, config);

    Pigeon2 pig = new Pigeon2(0);
    FPS.setPigeon2(pig);
  }

  private void configCameras(FieldPositioningSystem FPS, FPSConfiguration config) {
    CameraInterperter[] interperters = new CameraInterperter[config.cameraInformations.length];
    for (int i = 0; i < config.cameraInformations.length; i++) {
      CameraInformation camInfo = config.cameraInformations[i];
      interperters[i] =
          new CameraInterperter(
              camInfo.getCameraLocation(),
              this.fieldPositioningSystem.aprilTagFieldLayout,
              camInfo.getName(),
              () -> FPS.getCurrentRobotRotationXY());
    }
    FPS.setCameras(interperters);
  }
}
