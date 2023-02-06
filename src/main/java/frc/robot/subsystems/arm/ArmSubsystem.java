package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  // Hardware
  private final CANSparkMax armMotor1;
  private final CANSparkMax armMotor2;
  private final ProfiledPIDController armPPC1;

  private final CANSparkMax extendMotor;
  private final ProfiledPIDController extendPPC;

  private final WPI_TalonFX wristMotor;
  private double wristMotorGoal;

  public ArmSubsystem() {
    System.out.println("Starting Construct ArmSubsystem");
    armPPC1 =
        new ProfiledPIDController(
            Constants.armRotationKp,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.armRotationMaxVelo, Constants.armRotationMaxAccel));
    armMotor1 = new CANSparkMax(Constants.armRotationID1, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(Constants.armRotationID2, MotorType.kBrushless);

    armMotor1.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor1.setSmartCurrentLimit(Constants.armRotationCurrentLimit);

    armMotor2.follow(armMotor1);
    armMotor2.setSmartCurrentLimit(Constants.armRotationCurrentLimit);

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

    wristMotor = new WPI_TalonFX(Constants.wristID);
    wristMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            true, Constants.wristStatorLimit, Constants.wristStatorLimit, 1));
    wristMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, Constants.wristCurrentLimit, Constants.wristCurrentLimit, 1));
    wristMotor.config_kP(0, Constants.wristKp);
    wristMotor.configMotionAcceleration(Constants.wristMaxAccel);
    wristMotor.configMotionCruiseVelocity(Constants.wristMaxVelo);
    wristMotorGoal = Constants.wristRotationStowed;

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {
    armMotor1.set(
        armPPC1.calculate(armMotor1.getEncoder().getPosition())
            - computeRotationArbitraryFeetForward());
    extendMotor.set(extendPPC.calculate(extendMotor.getEncoder().getPosition()));
    wristMotor.set(
        ControlMode.MotionMagic,
        wristMotorGoal,
        DemandType.ArbitraryFeedForward,
        computeWristArbitraryFeetForward());
  }

  public void setStowed() {
    armPPC1.setGoal(Constants.armRotationStowed);
    extendPPC.setGoal(Constants.armLengthStowed);
    wristMotorGoal = Constants.wristRotationStowed;
    System.out.println("Stowed");
  }

  public void setLow() {
    armPPC1.setGoal(Constants.armRotationLow);
    extendPPC.setGoal(Constants.armLengthLow);
    wristMotorGoal = Constants.wristRotationLow;
    System.out.println("Low");
  }

  public void setCubeMid() {
    armPPC1.setGoal(Constants.armRotationCubeMid);
    extendPPC.setGoal(Constants.armLengthCubeMid);
    wristMotorGoal = Constants.wristRotationCubeMid;
    System.out.println("Cube Mid");
  }

  public void setCubeHigh() {
    armPPC1.setGoal(Constants.armRotationCubeHigh);
    extendPPC.setGoal(Constants.armLengthCubeHigh);
    wristMotorGoal = Constants.wristRotationCubeHigh;
    System.out.println("Cube High");
  }

  public void setConeMid() {
    armPPC1.setGoal(Constants.armRotationConeMid);
    extendPPC.setGoal(Constants.armLengthConeMid);
    wristMotorGoal = Constants.wristRotationConeMid;
  }

  public void setConeHigh() {
    armPPC1.setGoal(Constants.armRotationConeHigh);
    extendPPC.setGoal(Constants.armLengthConeHigh);
    wristMotorGoal = Constants.wristRotationConeHigh;
  }

  /**
   * Calculates the amount of power needed to counteract the force of gravity to keep the arm at a
   * constant angle.
   *
   * @return The power needed to keep the arme stable, in ?electrical output units?.
   */
  private double computeRotationArbitraryFeetForward() {
    double theta = armMotor1.getEncoder().getPosition() * Constants.armRotationalTicksToRadians;
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
        wristMotor.getSelectedSensorPosition() * Constants.wristTicksToRadians
            + armMotor1.getEncoder().getPosition() * Constants.armRotationalTicksToRadians;
            
    return (Math.cos(theta) * Constants.wristMomentOfInertia * 9.8)
        / (Constants.stalledTorque * Constants.wristGearRatio);
  }
}
