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

  private final CANSparkMax extendMotor;
  private final ProfiledPIDController extendPPC;

  private final CANSparkMax wristMotor;
  private final ProfiledPIDController wristPPC;

  public ArmSubsystem() {
    System.out.println("Starting Construct ArmSubsystem");
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
    armMotorLead.setSmartCurrentLimit(Constants.armRotationCurrentLimit);

    extendPPC =
        new ProfiledPIDController(
            Constants.armExtensionKp,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.armExtensionMaxVelo, Constants.armExtensionMaxAccel));
    extendMotor = new CANSparkMax(Constants.armExtenderID, MotorType.kBrushless);
    extendMotor.restoreFactoryDefaults();
    extendMotor.setSmartCurrentLimit(Constants.armExtensionCurrentLimit);

    wristPPC =
        new ProfiledPIDController(
            Constants.wristKp,
            0,
            0,
            new TrapezoidProfile.Constraints(Constants.wristMaxVelo, Constants.wristMaxAccel));
    wristMotor = new CANSparkMax(Constants.wristID, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristMotor.setSmartCurrentLimit(Constants.wristCurrentLimit);

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {
    armMotorLead.set(
        armPPC.calculate(armMotorLead.getEncoder().getPosition())
            - computeRotationArbitraryFeetForward());
    extendMotor.set(extendPPC.calculate(extendMotor.getEncoder().getPosition()));
    wristMotor.set(
        wristPPC.calculate(wristMotor.getEncoder().getPosition())
            + computeWristArbitraryFeetForward());
  }

  public void setStowed() {
    armPPC.setGoal(Constants.armRotationStowed);
    extendPPC.setGoal(Constants.armLengthStowed);
    wristPPC.setGoal(Constants.wristRotationStowed);
  }

  public void setLow() {
    armPPC.setGoal(Constants.armRotationLow);
    extendPPC.setGoal(Constants.armLengthLow);
    wristPPC.setGoal(Constants.wristRotationLow);
  }

  public void setCubeMid() {
    armPPC.setGoal(Constants.armRotationCubeMid);
    extendPPC.setGoal(Constants.armLengthCubeMid);
    wristPPC.setGoal(Constants.wristRotationCubeMid);
  }

  public void setCubeHigh() {
    armPPC.setGoal(Constants.armRotationCubeHigh);
    extendPPC.setGoal(Constants.armLengthCubeHigh);
    wristPPC.setGoal(Constants.wristRotationCubeHigh);
  }

  public void setConeMid() {
    armPPC.setGoal(Constants.armRotationConeMid);
    extendPPC.setGoal(Constants.armLengthConeMid);
    wristPPC.setGoal(Constants.wristRotationConeMid);
  }

  public void setConeHigh() {
    armPPC.setGoal(Constants.armRotationConeHigh);
    extendPPC.setGoal(Constants.armLengthConeHigh);
    wristPPC.setGoal(Constants.wristRotationConeHigh);
  }

  /**
   * Calculates the amount of power needed to counteract the force of gravity to keep the arm at a
   * constant angle.
   *
   * @return The power needed to keep the arme stable, in ?electrical output units?.
   */
  private double computeRotationArbitraryFeetForward() {
    double theta = armMotorLead.getEncoder().getPosition() * Constants.armRotationalTicksToRadians;
    double armLength =
        extendMotor.getEncoder().getPosition() * Constants.armLengthTicksToMeters
            + Constants.armLengthAtZeroTicks;
    return (Math.cos(theta - Math.toRadians(Constants.armAngleAtRest))
            * armLength
            * Constants.armWeight)
        / (Constants.stalledTorque * Constants.rotationArmGearRatio);
  }

  private double computeWristArbitraryFeetForward() {
    double theta =
        wristMotor.getEncoder().getPosition() * Constants.wristTicksToRadians
            + armMotorLead.getEncoder().getPosition() * Constants.armRotationalTicksToRadians;
    return (Math.cos(theta) * Constants.wristMomentOfInertia * 9.8)
        / (Constants.stalledTorque * Constants.wristGearRatio);
  }
}
