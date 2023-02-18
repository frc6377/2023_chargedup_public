package frc.robot.subsystems.arm;

import java.lang.annotation.Target;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  // Hardware
  private final CANSparkMax rotationMotor1;
  private final CANSparkMax rotationMotor2;
  private final ProfiledPIDController rotationPPC;

  private final CANSparkMax extendMotor;
  private final RelativeEncoder extendEncoder;
  private final SparkMaxPIDController extendController;
  private final ProfiledPIDController extendPPC;
  private double setpoint = 0;

  private final WPI_TalonFX wristMotor;
  private double wristMotorGoal;

  public ArmSubsystem() {
    System.out.println("Starting Construct ArmSubsystem");
    rotationPPC =
        new ProfiledPIDController(
            Constants.ARM_ROTATION_KP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.ARM_ROTATION_MAX_VELOCITY, Constants.ARM_ROTATION_MAX_ACCELLERATION));
    rotationMotor1 = new CANSparkMax(Constants.ARM_ROTATION_ID_1, MotorType.kBrushless);
    rotationMotor2 = new CANSparkMax(Constants.ARM_ROTATION_ID2, MotorType.kBrushless);

    rotationMotor1.restoreFactoryDefaults();
    rotationMotor2.restoreFactoryDefaults();

    rotationMotor1.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

    // We haven't been able to get following to work...
    // rotationMotor2.follow(rotationMotor1);
    // so instead we will just feed the same values to both motors.
    rotationMotor2.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

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

    extendEncoder = extendMotor.getEncoder();
    extendController = extendMotor.getPIDController();

    extendController.setP(0.009524*0.05, 0);
    extendController.setI(0, 0);
    extendController.setD(0.04*0.1, 0);
    extendController.setFF(0, 0);
    extendController.setIZone(0, 0);
    extendController.setOutputRange(-1, 1);
    extendController.setSmartMotionAllowedClosedLoopError(0.1, 0);
    // extendController.setFeedbackDevice(extendEncoder);
    extendController.setSmartMotionMaxAccel(16000, 0);
    extendController.setSmartMotionMaxVelocity(16000, 0);

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
    
    System.out.println(extendMotor.getOutputCurrent());
    double armMotorOutput =
        rotationPPC.calculate(rotationMotor1.getEncoder().getPosition())
            + computeRotationArbitraryFeetForward();
    rotationMotor1.set(armMotorOutput);
    rotationMotor2.set(-armMotorOutput);
    double calc = extendPPC.calculate(extendMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Extend ppc out", calc);
    
    wristMotor.set(
        ControlMode.Position,
        wristMotorGoal
        );
    SmartDashboard.putNumber("ElevatorPosition", extendMotor.getEncoder().getPosition());
  }

  public void setTarget(ArmPosition armPosition) {
    extendController.setReference(armPosition.armExtension, ControlType.kSmartMotion);
    setpoint = armPosition.armExtension;
    System.out.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    rotationPPC.setGoal(
        Math.toRadians(armPosition.armRotation) / Constants.ARM_ROTATION_TICKS_TO_RADIANS);
    extendPPC.setGoal(-1045);
    wristMotorGoal = Math.toRadians(armPosition.wristRotation) / Constants.WRIST_TICKS_TO_RADIANS;
  }

  /**
   * Calculates the amount of power needed to counteract the force of gravity to keep the arm at a
   * constant angle.
   *
   * @return The power needed to keep the arme stable, in ?electrical output units?.
   */
  // TODO: move to I alpha instead of torque
  private double computeRotationArbitraryFeetForward() {
    return rotationArbitraryFeetForward(
        rotationMotor1.getEncoder().getPosition(), extendMotor.getEncoder().getPosition());
  }

  public static double rotationArbitraryFeetForward(
      double rotationPosition, double extendPosition) {
    double theta = rotationPosition * Constants.ARM_ROTATION_TICKS_TO_RADIANS;
    double armLength =
        extendPosition * Constants.ARM_EXTENSIONS_TICKS_TO_METERS
            + Constants.ARM_LENGTH_AT_ZERO_TICKS_METERS;
    return (Math.cos(theta) * armLength * Constants.ARM_WEIGHT_KG)
        / (Constants.STALLED_TORQUE * 2 * Constants.ROTATION_ARM_GEAR_RATIO);
  }

  private double computeWristArbitraryFeetForward() {
    double theta =
        wristMotor.getSelectedSensorPosition() * Constants.WRIST_TICKS_TO_RADIANS
            + rotationMotor1.getEncoder().getPosition() * Constants.ARM_ROTATION_TICKS_TO_RADIANS;

    return (Math.cos(theta) * Constants.WRIST_MOMENT_OF_INERTIA * 9.8)
        / (Constants.STALLED_TORQUE * Constants.WRIST_GEAR_RATIO);
  }
}
