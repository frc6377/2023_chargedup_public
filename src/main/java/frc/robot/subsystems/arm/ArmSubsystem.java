package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  // Hardware
  private final CANSparkMax armMotorLead;
  private final CANSparkMax armMotorFollow;
  private final ProfiledPIDController armPPC;
  private final ArmExtender extender;
  private final Wrist wrist;

  public ArmSubsystem() {
    System.out.println("Starting Construct ArmSubsystem");
    // TODO: add current limits for arm rotation, extension, wrist.
    armPPC =
        new ProfiledPIDController(
            Constants.armRotationKp,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.armRotationMaxVelo, Constants.armRotationMaxAccel));
    armMotorLead = new CANSparkMax(Constants.armRotationID1, MotorType.kBrushless);
    armMotorLead.restoreFactoryDefaults();
    armMotorFollow = new CANSparkMax(Constants.armRotationID2, MotorType.kBrushless);
    armMotorFollow.restoreFactoryDefaults();
    armMotorFollow.follow(armMotorLead);

    armMotorLead.setSmartCurrentLimit(40);

    extender = new ArmExtender(Constants.armLengthID1, Constants.armLengthID2);
    wrist = new Wrist(Constants.wristID);

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {
    armMotorLead.set(
        armPPC.calculate(armMotorLead.getEncoder().getPosition())
            - computeRotationArbitraryFeetForward());
  }

  public void setStowed() {
    armPPC.setGoal(Constants.armRotationStowed);
    extender.setLength(Constants.armLengthStowed);
    // extenderPPC.setGoal
    wrist.setPositionDegrees(Constants.wristRotationStowed);
    // wristPPC.setGoal(degresToTicks(Constants.writstRotationStowed)
  }

  public void setLow() {
    armPPC.setGoal(Constants.armRotationLow);
    extender.setLength(Constants.armLengthLow);
    wrist.setPositionDegrees(Constants.wristRotationLow);
  }

  public void setCubeMid() {
    armPPC.setGoal(Constants.armRotationCubeMid);
    extender.setLength(Constants.armLengthCubeMid);
    wrist.setPositionDegrees(Constants.wristRotationCubeMid);
  }

  public void setCubeHigh() {
    armPPC.setGoal(Constants.armRotationCubeHigh);
    extender.setLength(Constants.armLengthCubeHigh);
    wrist.setPositionDegrees(Constants.wristRotationCubeHigh);
  }

  public void setConeMid() {
    armPPC.setGoal(Constants.armRotationConeMid);
    extender.setLength(Constants.armLengthConeMid);
    wrist.setPositionDegrees(Constants.wristRotationConeMid);
  }

  public void setConeHigh() {
    armPPC.setGoal(Constants.armRotationConeHigh);
    extender.setLength(Constants.armLengthConeHigh);
    wrist.setPositionDegrees(Constants.wristRotationConeHigh);
  }

  /**
   * Calculates the amount of power needed to counteract the force of gravity to keep the arm at a
   * constant angle.
   *
   * @return The power needed to keep the arme stable, in ?electrical output units?.
   */
  private double computeRotationArbitraryFeetForward() {
    double theta = armMotorLead.getEncoder().getPosition() * Constants.armRotationalTicksToRadians;
    double armLength = extender.getLength();
    return (Math.cos(theta - Math.toRadians(Constants.armAngleAtRest))
            * armLength
            * Constants.armWeight)
        / (Constants.stalledTorque * Constants.rotationArmGearRatio);
  }
  // TODO: Change this from setting a single motor to
  private void setRotation(double position) {
    armPPC.setGoal(position);
  }

  public void setRotationDegrees(double degrees) {
    setRotation(Math.toRadians(degrees) / Constants.armRotationalTicksToRadians);
  }
}
