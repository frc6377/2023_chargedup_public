package frc.robot.subsystems.drivetrain.config;

public class PodConfig {
  protected final int driveID;
  protected final int turnID;

  protected final int canCoderID;
  protected final double canCoderOffset;
  protected final String name;

  public PodConfig(int driveID, int turnID, int canCoderID, double canCoderOffset, String name) {

    this.driveID = driveID;
    this.turnID = turnID;
    this.canCoderID = canCoderID;
    this.canCoderOffset = canCoderOffset;
    this.name = name;
  }

  public String toString() {
    return name;
  }

  public int getDriveID() {
    return driveID;
  }

  public int getTurnID() {
    return turnID;
  }

  public int getCanCoderID() {
    return canCoderID;
  }

  public double getCanCoderOffset() {
    return canCoderOffset;
  }

  public String getName() {
    return name;
  }
}
