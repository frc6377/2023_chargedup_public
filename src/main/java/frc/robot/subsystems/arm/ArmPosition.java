package frc.robot.subsystems.arm;

public class ArmPosition {
  protected final double armRotation;
  protected final double armExtension;
  protected final double wristRotation;
  protected final String name;
  protected final String description;

  public ArmPosition(double armRotation, double armExtension, double wristRotation, String name) {
    this.armRotation = armRotation;
    this.armExtension = armExtension;
    this.wristRotation = wristRotation;
    this.name = name;
    this.description =
        String.format(
            "%s - armRotation: %.3f, armExtension: %.3f, wristRotation: %.3f",
            name, armRotation, armExtension, wristRotation);
  }

  public String toString() {
    return description;
  }

  public double getArmRotation() {
    return armRotation;
  }

  public double getArmExtension() {
    return armExtension;
  }

  public double getWristRotation() {
    return wristRotation;
  }

  public String getName() {
    return name;
  }

  public String getDescription() {
    return description;
  }
}
