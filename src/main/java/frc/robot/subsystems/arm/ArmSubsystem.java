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
            Constants.ARM_ROTATION_KP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.ARM_ROTATION_MAX_VELOCITY, Constants.ARM_ROTATION_MAX_ACCELLERATION));
    armMotor1 = new CANSparkMax(Constants.ARM_ROTATION_ID_1, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(Constants.ARM_ROTATION_ID2, MotorType.kBrushless);

    armMotor1.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor1.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

    // We haven't been able to gett following to work.
    // armMotor2.follow(armMotor1);
    armMotor2.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

    extendPPC =
        new ProfiledPIDController(
            Constants.ARM_EXTENSION_KP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.ARM_EXTENSION_MAX_VELOCITY, Constants.ARM_EXTENSION_MAX_ACCELLERATION));
    extendMotor = new CANSparkMax(Constants.ARM_EXTENDER_ID, MotorType.kBrushless);
    extendMotor.restoreFactoryDefaults();
    extendMotor.setSmartCurrentLimit(Constants.ARM_EXTENSION_CURRENT_LIMIT);

    wristMotor = new WPI_TalonFX(Constants.WRIST_ID);
    wristMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            true, Constants.WRIST_STATOR_LIMIT, Constants.WRIST_STATOR_LIMIT, 1));
    wristMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true, Constants.WRIST_CURRENT_LIMIT, Constants.WRIST_CURRENT_LIMIT, 1));
    wristMotor.config_kP(0, Constants.WRIST_KP);
    wristMotor.configMotionAcceleration(Constants.WRIST_MAX_ACCELLERATION);
    wristMotor.configMotionCruiseVelocity(Constants.WRIST_MAX_VELOCITY);
    wristMotorGoal = Constants.WRIST_ROTATION_STOWED;

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {
    double armMotorOutput = armPPC1.calculate(armMotor1.getEncoder().getPosition()) + computeRotationArbitraryFeetForward();
    armMotor1.set(armMotorOutput);
    armMotor2.set(-armMotorOutput);
    extendMotor.set(extendPPC.calculate(extendMotor.getEncoder().getPosition()));
    wristMotor.set(
        ControlMode.MotionMagic,
        wristMotorGoal,
        DemandType.ArbitraryFeedForward,
        computeWristArbitraryFeetForward());
  }

  public void setTarget(double armDegrees, double extendMeters, double wristDegrees) {
    armPPC1.setGoal(Math.toRadians(armDegrees)/Constants.ARM_ROTATION_TICKS_TO_RADIANS);
    extendPPC.setGoal(extendMeters/Constants.ARM_EXTENSIONS_TICKS_TO_METERS);
    wristMotorGoal = Math.toRadians(wristDegrees)/Constants.WRIST_TICKS_TO_RADIANS;
  }

  /**
   * Calculates the amount of power needed to counteract the force of gravity to keep the arm at a
   * constant angle.
   *
   * @return The power needed to keep the arme stable, in ?electrical output units?.
   */
  //TODO: move to I alpha instead of torque
  private double computeRotationArbitraryFeetForward() {
    double theta = armMotor1.getEncoder().getPosition() * Constants.ARM_ROTATION_TICKS_TO_RADIANS;
    double armLength =
        extendMotor.getEncoder().getPosition() * Constants.ARM_EXTENSIONS_TICKS_TO_METERS
            + Constants.ARM_LENGTH_AT_ZERO_TICKS;
    return (Math.cos(theta - Math.toRadians(Constants.ARM_ANGLE_AT_REST))
            * armLength
            * Constants.ARM_WEIGHT)
        / (Constants.STALLED_TORQUE * 2 * Constants.ROTATION_ARM_GEAR_RATIO);
  }

  private double computeWristArbitraryFeetForward() {
    double theta =
        wristMotor.getSelectedSensorPosition() * Constants.WRIST_TICKS_TO_RADIANS
            + armMotor1.getEncoder().getPosition() * Constants.ARM_ROTATION_TICKS_TO_RADIANS;
            
    return (Math.cos(theta) * Constants.WRIST_MOMENT_OF_INERTIA * 9.8)
        / (Constants.STALLED_TORQUE * Constants.WRIST_GEAR_RATIO);
  }
}
