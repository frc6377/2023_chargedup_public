package frc.robot.subsystems.drivetrain.config;

public enum PodLocation {
  FRONT_RIGHT(0),
  FRONT_LEFT(1),
  BACK_RIGHT(2),
  BACK_LEFT(3);
  private int id;

  private PodLocation(int id) {
    this.id = id;
  }

  public int getDriveMotorID() {
    switch (id) {
      case 0: // Front Right
        return 5;
      case 1: // Front Left
        return 3;
      case 2: // Back Right
        return 7;
      case 3: // Back Left
        return 1;
      default:
        return 0;
    }
  }

  public int getSteerMotorID() {
    switch (id) {
      case 0: // Front Right
        return 6;
      case 1: // Front Left
        return 4;
      case 2: // Back Right Turn
        return 8;
      case 3: // Back Left
        return 2;
      default:
        return 0;
    }
  }

  public double getOffSet() {
    switch (id) {
      case 0:
        return -Math.toRadians(0);
      case 1:
        return -Math.toRadians(90);
      case 2:
        return -Math.toRadians(180);
      case 3:
        return -Math.toRadians(270);
      default:
        return 0;
    }
  }
}
