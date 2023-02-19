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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  // Hardware
  private final CANSparkMax leftShoulder;
  private final RelativeEncoder leftShoulderEncoder;
  private final SparkMaxPIDController leftShoulderController;
  private final CANSparkMax rightShoulder;

  private final CANSparkMax extendMotor;
  private final RelativeEncoder extendEncoder;
  private final SparkMaxPIDController extendController;

  private final WPI_TalonFX wristMotor;
  private ArmPosition armPosition = new ArmPosition(0, 1, -9099, "default");

  public ArmSubsystem() {
    System.out.println("Starting Construct ArmSubsystem");
    
    leftShoulder = new CANSparkMax(Constants.LEFT_SHOULDER_ID, MotorType.kBrushless);
    rightShoulder = new CANSparkMax(Constants.RIGHT_SHOULDER_ID, MotorType.kBrushless);



    leftShoulder.restoreFactoryDefaults();
    rightShoulder.restoreFactoryDefaults();

    leftShoulder.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

    // We haven't been able to get following to work...
    // rotationMotor2.follow(rotationMotor1);
    // so instead we will just feed the same values to both motors.
    rightShoulder.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

    leftShoulderEncoder = leftShoulder.getEncoder();
    leftShoulderController = leftShoulder.getPIDController();

    leftShoulderController.setP(0.0005, 0);
    leftShoulderController.setI(0.000, 0);
    leftShoulderController.setD(0.001, 0);
    leftShoulderController.setIZone(0.000, 0);
    leftShoulderController.setFF(0, 0);
    leftShoulderController.setOutputRange(-1, 1);
    leftShoulderController.setSmartMotionAllowedClosedLoopError(1, 0);
    leftShoulderController.setSmartMotionMaxVelocity(400, 0);
    leftShoulderController.setSmartMotionMaxAccel(800, 0);

    rightShoulder.follow(leftShoulder, true);

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

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {
    leftShoulderController.setReference(armPosition.armRotation, ControlType.kSmartMotion, 0, computeShoulderArbitraryFeetForward());
    SmartDashboard.putNumber("arb ffw", computeShoulderArbitraryFeetForward());  
    extendController.setReference(armPosition.armExtension, ControlType.kSmartMotion);


  }

  public void setTarget(ArmPosition armPosition) {
    this.armPosition = armPosition;
    wristMotor.set(ControlMode.MotionMagic, armPosition.wristRotation);
  }

  /**
   * Calculates the amount of power needed to counteract the force of gravity to keep the arm at a
   * constant angle.
   *
   * @return The power needed to keep the arme stable, in ?electrical output units?.
   */
  // TODO: move to I alpha instead of torque
  private double computeShoulderArbitraryFeetForward() {
    double theta = (leftShoulderEncoder.getPosition()/90) - 8;
    double centerOfMass = 0.4825;
    double mass = 6.01;
    double gearRatio = 90;
    double numMotors = 2;
    double torque = Math.cos(theta)*9.81*mass*centerOfMass;
    return 12*torque/(Constants.STALLED_TORQUE*0.85*0.8*gearRatio*numMotors);
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
            + leftShoulder.getEncoder().getPosition() * Constants.ARM_ROTATION_TICKS_TO_RADIANS;

    return (Math.cos(theta) * Constants.WRIST_MOMENT_OF_INERTIA * 9.8)
        / (Constants.STALLED_TORQUE * Constants.WRIST_GEAR_RATIO);
  }
}
