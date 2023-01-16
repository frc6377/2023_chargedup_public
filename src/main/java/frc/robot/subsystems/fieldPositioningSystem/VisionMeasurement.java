package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.math.geometry.Pose3d;

class VisionMeasurement {
  Pose3d measurement;
  double stdev;
  double timeRecorded;

  public VisionMeasurement(Pose3d measurement, double stdev, double timeRecorded) {
    this.measurement = measurement;
    this.timeRecorded = timeRecorded;
    this.stdev = stdev;
  }
}
