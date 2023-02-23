package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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
  private final ProfiledPIDController shoulderPPC;
  private final CANSparkMax rightShoulder;

  //todo make these WPI_CANCoders. using CANCoder for now because it works and we dont have time for any more testing
  private final CANCoder shoulderCANCoder;
  private final CANCoder wristCANCoder;
  private final WPI_TalonFX brakeFalcon;

  private final CANSparkMax extendMotor;
  private final RelativeEncoder extendEncoder;
  private final SparkMaxPIDController extendController;

  private final WPI_TalonFX wristMotor;
  private ArmPosition armPosition = new ArmPosition(0, 1, -8475, "default");

  public ArmSubsystem() {
    System.out.println("Starting Construct ArmSubsystem");

    brakeFalcon = new WPI_TalonFX(Constants.BREAK_FALCON_ID);
    brakeFalcon.setNeutralMode(NeutralMode.Coast);
    brakeFalcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10, 0));

    leftShoulder = new CANSparkMax(Constants.LEFT_SHOULDER_ID, MotorType.kBrushless);
    rightShoulder = new CANSparkMax(Constants.RIGHT_SHOULDER_ID, MotorType.kBrushless);

    shoulderCANCoder = new CANCoder(20);
    shoulderCANCoder.configMagnetOffset(Constants.SHOULDER_CANCODER_OFFSET);
    shoulderCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    shoulderCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    shoulderCANCoder.setPositionToAbsolute();
    shoulderCANCoder.configSensorDirection(false);
    
    leftShoulder.restoreFactoryDefaults();
    rightShoulder.restoreFactoryDefaults();

    leftShoulder.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

    // We haven't been able to get following to work...
    // rotationMotor2.follow(rotationMotor1);
    // so instead we will just feed the same values to both motors.
    rightShoulder.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

    leftShoulderEncoder = leftShoulder.getEncoder();
    leftShoulderController = leftShoulder.getPIDController();

    shoulderPPC = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(4, 6));

    shoulderPPC.setTolerance(0.02);

    rightShoulder.follow(leftShoulder, true);

    extendMotor = new CANSparkMax(Constants.ARM_EXTENDER_ID, MotorType.kBrushless);
    extendMotor.restoreFactoryDefaults();
    extendMotor.setSmartCurrentLimit(Constants.ARM_EXTENSION_CURRENT_LIMIT);

    extendEncoder = extendMotor.getEncoder();
    extendController = extendMotor.getPIDController();

    extendController.setP(0.009524 * 0.05, 0);
    extendController.setI(0, 0);
    extendController.setD(0.04 * 0.1, 0);
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

    wristCANCoder = new CANCoder(Constants.WRIST_CANCODER_ID);
    wristCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    wristCANCoder.configMagnetOffset(Constants.WRIST_CANCODER_OFFSET);
    wristCANCoder.configSensorDirection(true);
    wristMotor.setSelectedSensorPosition(wristCANCoderToIntegratedSensor(wristCANCoder.getAbsolutePosition()));

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {

    double shoulderOutput;

    if (false) {
      shoulderOutput = 0;
      brakeFalcon.set(ControlMode.PercentOutput, 0.05);
    } else {
      shoulderOutput = computeShoulderOutput();
      brakeFalcon.set(ControlMode.PercentOutput, 0);
    }

    leftShoulder.set(shoulderOutput);
    SmartDashboard.putNumber("arb ffw", computeShoulderArbitraryFeetForward());
    SmartDashboard.putNumber("Extension2 electric boogaloo (encoder pos)", extendEncoder.getPosition());

    extendController.setReference(armPosition.armExtension, ControlType.kSmartMotion, 0, computeElevatorFeedForward(), ArbFFUnits.kPercentOut);


    SmartDashboard.putNumber("Wrist Position (Ticks)", wristMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("shoulder 2 electric", Math.toDegrees(thetaFromCANCoder()));
  }

  public void setTarget(ArmPosition armPosition) {
    this.armPosition = armPosition;
    shoulderPPC.setGoal(armPosition.armRotation);
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
    double theta = thetaFromCANCoder();
    double centerOfMass = computeCenterOfMass();
    double mass = 6.01;
    double gearRatio = 90;
    double numMotors = 2;
    double torque = Math.cos(theta) * 9.81 * mass * centerOfMass;
    double out = torque / (Constants.STALLED_TORQUE * 0.85 * gearRatio * numMotors);
    SmartDashboard.putNumber("shoulder arb ffw", out);
    return out;
  }

  private double computeCenterOfMass() {
    return (extendEncoder.getPosition() / 12 * 1.1175) + 0.4825;
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

  public double thetaFromCANCoder() {
    double rawPos = shoulderCANCoder.getPosition();
    SmartDashboard.putNumber("raw CANcoder", rawPos);
    double theta = Math.toRadians(rawPos * (6.0 / 16.0));
    SmartDashboard.putNumber("shoulder theta", theta);
    return theta;
  }

  public double thetaFromPPC(){
    return shoulderPPC.getSetpoint().position;
  }

  private double computeWristArbitraryFeetForward() {
    double theta =
        wristMotor.getSelectedSensorPosition() * Constants.WRIST_TICKS_TO_RADIANS
            + leftShoulder.getEncoder().getPosition() * Constants.ARM_ROTATION_TICKS_TO_RADIANS;

    return (Math.cos(theta) * Constants.WRIST_MOMENT_OF_INERTIA * 9.8)
        / (Constants.STALLED_TORQUE * Constants.WRIST_GEAR_RATIO);
  }

  private double computeShoulderOutput() {
    double output =
        shoulderPPC.calculate(thetaFromCANCoder()) + computeShoulderArbitraryFeetForward();
    SmartDashboard.putNumber("shoulder output", output);
    return output;
  }

  public double currentArmExtenstion() {
    return extendEncoder.getPosition() * Math.PI * Constants.CAPSTAN_DIAMETER_METERS
        + Constants.ARM_LENGTH_AT_ZERO_TICKS_METERS;
  }

  private double computeElevatorFeedForward(){
    double theta = thetaFromCANCoder(); 
    double magicNumberThatMakesItWork = 0.5;
    double mass = 4.08 - magicNumberThatMakesItWork;
    double stallLoad = 22.929;
    return mass*Math.sin(theta)/stallLoad;
  }

  private double wristCANCoderToIntegratedSensor (double theta){
    theta /= Constants.WRIST_GEAR_RATIO; //output shaft to input shaft
    theta/=360; //degrees to revs
    return theta*2048; //revs to ticks
  }
}
