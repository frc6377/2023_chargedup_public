package frc.robot.subsystems.fieldPositioningSystem;

import edu.wpi.first.math.geometry.Pose2d;

class VisionMeasurement {
  Pose2d measurement;
  double stdev;
  double timeRecorded;

  public VisionMeasurement(Pose2d measurement, double stdev, double timeRecorded) {
    this.measurement = measurement;
    this.timeRecorded = timeRecorded;
    this.stdev = stdev;
  }
}
