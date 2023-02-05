package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  // Hardware
  private final CANSparkMax armMotorLead;
  private final CANSparkMax armMotorFollow;
  private final RelativeEncoder encoder;
  private final ProfiledPIDController ppc;
  private final ArmExtender extender;
  private final Wrist wrist;

  public ArmSubsystem() {
    System.out.println("Starting Construct ArmSubsystem");
    // TODO: add current limits for arm rotation, extension, wrist.
    ppc =
        new ProfiledPIDController(
            0.099902, 0, 0, new TrapezoidProfile.Constraints(15, 17.64705882));
    armMotorLead = new CANSparkMax(Constants.armRotationID1, MotorType.kBrushless);
    armMotorLead.restoreFactoryDefaults();
    armMotorFollow = new CANSparkMax(Constants.armRotationID2, MotorType.kBrushless);
    armMotorFollow.restoreFactoryDefaults();
    armMotorFollow.follow(armMotorLead);
    // TODO: determine the correct value for countsPerRev for this encoder
    encoder = armMotorLead.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    armMotorLead.setSmartCurrentLimit(40);

    // TODO: when we have a real extension arm, change this to `ActiveArmExtender`
    extender = new ArmExtender(Constants.armLengthID1, Constants.armLengthID2);
    wrist = new Wrist(Constants.wristID);

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {
    armMotorLead.set(ppc.calculate(encoder.getPosition()) - computeArbitraryFeetForward());
  }

  public void setStowed() {
    setPosition(Constants.armRotationStowed);
    extender.setLength(Constants.armLengthStowed);
    wrist.setPositionDegrees(Constants.wristRotationStowed);
  }

  public void setLow() {
    setPosition(Constants.armRotationLow);
    extender.setLength(Constants.armLengthLow);
    wrist.setPositionDegrees(Constants.wristRotationLow);
  }

  public void setCubeMid() {
    setPosition(Constants.armRotationCubeMid);
    extender.setLength(Constants.armLengthCubeMid);
    wrist.setPositionDegrees(Constants.wristRotationCubeMid);
  }

  public void setCubeHigh() {
    setPosition(Constants.armRotationCubeHigh);
    extender.setLength(Constants.armLengthCubeHigh);
    wrist.setPositionDegrees(Constants.wristRotationCubeHigh);
  }

  public void setConeMid() {
    setPosition(Constants.armRotationConeMid);
    extender.setLength(Constants.armLengthConeMid);
    wrist.setPositionDegrees(Constants.wristRotationConeMid);
  }

  public void setConeHigh() {
    setPosition(Constants.armRotationConeHigh);
    extender.setLength(Constants.armLengthConeHigh);
    wrist.setPositionDegrees(Constants.wristRotationConeHigh);
  }

  /**
   * Calculates the amount of power needed to counteract the force of gravity to keep the arm at a
   * constant angle.
   *
   * @return The power needed to keep the arme stable, in ?electrical output units?.
   */
  private double computeArbitraryFeetForward() {
    double theta = encoder.getPosition() * Constants.armRotationalTicksToRadians;
    double armLength = extender.getLength();
    return (Math.cos(theta - Math.toRadians(Constants.armAngleAtRest))
            * armLength
            * Constants.armWeight)
        / (Constants.stalledTorque * Constants.rotationArmGearRatio);
  }

  private void setPosition(double position) {
    ppc.setGoal(position);
  }

  public void setPositionDegrees(double degrees) {
    setPosition(Math.toRadians(degrees) / Constants.armRotationalTicksToRadians);
  }

  private double getPosition() {
    return encoder.getPosition();
  }
}
