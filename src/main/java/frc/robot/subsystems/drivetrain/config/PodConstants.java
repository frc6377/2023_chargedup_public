package frc.robot.subsystems.drivetrain.config;

public class PodConstants {
  public final int DRIVE_MOTOR;
  public final int STEER_MOTOR;
  public final int STEER_ENCODER;
  public final double STEER_OFFSET;

  private PodConstants(int DRIVE_MOTOR, int STEER_MOTOR, int STEER_ENCODER, double STEER_OFFSET) {
    this.DRIVE_MOTOR = DRIVE_MOTOR;
    this.STEER_MOTOR = STEER_MOTOR;
    this.STEER_ENCODER = STEER_ENCODER;
    this.STEER_OFFSET = STEER_OFFSET;
  }

  public static final class PodConstantsBuilder {
    private Integer driveMotorID = null;
    private Integer steerMotorID = null;
    private Integer encoderID = null;
    private Double encoderOffset = null;
    private PodLocation location;

    public PodConstants build() {
      boolean error = false;
      if (encoderID == null) {
        System.out.println("Encoder created without encoderID");
        error = true;
      }
      if (encoderOffset == null) {
        System.out.println("Pod created without encoderOffset");
        error = true;
      }
      if (location == null) {
        System.out.println("Pod created without location");
        error = true;
      }

      if (error) return null;

      //encoderOffset += location.getOffSet();
      driveMotorID = location.getDriveMotorID();
      steerMotorID = location.getSteerMotorID();

      return new PodConstants(driveMotorID, steerMotorID, encoderID, encoderOffset);
    }

    public PodConstantsBuilder setEncoderID(Integer encoderID) {
      this.encoderID = encoderID;
      return this;
    }

    public PodConstantsBuilder setEncoderOffset(Double encoderOffset) {
      this.encoderOffset = encoderOffset;
      return this;
    }

    public PodConstantsBuilder setLocation(PodLocation location) {
      this.location = location;
      return this;
    }
  }
}
