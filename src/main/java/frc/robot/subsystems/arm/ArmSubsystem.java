package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OverheatedException;
import frc.robot.networktables.DeltaBoard;
import frc.robot.subsystems.color.GamePieceMode;
import frc.robot.utilities.DebugLog;

public class ArmSubsystem extends SubsystemBase {

  // Hardware
  private final CANSparkMax leftShoulder;
  private final ProfiledPIDController shoulderPPC;
  private final ProfiledPIDController elevatorPPC;
  private final CANSparkMax rightShoulder;

  private final CANCoder shoulderCANCoder;
  private final CANCoder wristCANCoder;
  private final CANCoder elevatorCANCoder;

  // Shuffleboard tab to allow for automatic offset configuration
  private final ShuffleboardTab offsetTab = Shuffleboard.getTab("Offsets");
  private final GenericEntry wristOffsetEntry;

  private final CANSparkMax extendMotor;
  private final RelativeEncoder extendEncoder;
  private final SparkMaxPIDController extendController;

  private final WPI_TalonFX wristMotor;

  // State Tracking
  private ArmPosition armPosition = ArmPosition.LOW_CUBE_ARM_POSITION;
  private boolean elevatorInPercentControl = false;
  private double elevatorPercentOutput = 0;
  // High Speed Data logging
  private DebugLog<Double> armRotationLog;
  private DebugLog<Double> armExtensionLog;
  private DebugLog<Double> wristRotationLog;
  private DebugLog<String> armPosLog;
  private DebugLog<Double> elevatorPercentOutputLog;

  private final IntegerSubscriber gamePieceModeSubscriber =
      NetworkTableInstance.getDefault().getIntegerTopic("GAME_PIECE_MODE").subscribe(10);

  public ArmSubsystem() {
    System.out.println("Starting Construct ArmSubsystem");

    // Create and configure shoulder
    leftShoulder = new CANSparkMax(Constants.LEFT_SHOULDER_ID, MotorType.kBrushless);
    rightShoulder = new CANSparkMax(Constants.RIGHT_SHOULDER_ID, MotorType.kBrushless);

    shoulderCANCoder = new CANCoder(20);
    shoulderCANCoder.configMagnetOffset(Constants.SHOULDER_CANCODER_OFFSET);
    shoulderCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    shoulderCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    shoulderCANCoder.setPositionToAbsolute();

    // Create and configure elevator
    elevatorCANCoder = new CANCoder(10);
    elevatorCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    elevatorCANCoder.configSensorDirection(true);
    elevatorCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 4);

    final double shoudlerPositionOnStartUp = shoulderCANCoder.getPosition();

    // If the shoulder says that it is at a negative angle, we can increase its angle by 360 to make
    // the math easier.
    if (shoudlerPositionOnStartUp < -30) {
      shoulderCANCoder.setPosition(shoudlerPositionOnStartUp + 360);
    }

    shoulderCANCoder.configSensorDirection(false);

    leftShoulder.restoreFactoryDefaults();
    rightShoulder.restoreFactoryDefaults();

    leftShoulder.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);
    rightShoulder.setSmartCurrentLimit(Constants.ARM_ROTATION_CURRENT_LIMIT);

    shoulderPPC = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(4, 4));
    elevatorPPC =
        new ProfiledPIDController(
            0.002, 0, 0.000000015, new TrapezoidProfile.Constraints(2 * 19200, 2500));

    shoulderPPC.setTolerance(0.02);
    elevatorPPC.setTolerance(10);

    rightShoulder.follow(leftShoulder, true);

    extendMotor = new CANSparkMax(Constants.ARM_EXTENDER_ID, MotorType.kBrushless);
    extendMotor.restoreFactoryDefaults();
    extendMotor.setSmartCurrentLimit(Constants.ARM_EXTENSION_CURRENT_LIMIT, 100, 4);
    extendMotor.setInverted(true);

    extendEncoder = extendMotor.getEncoder();
    extendController = extendMotor.getPIDController();

    extendController.setP(0.009524 * 0.05, 0);
    extendController.setI(0, 0);
    extendController.setD(0.04 * 0.1, 0);
    extendController.setFF(0, 0);
    extendController.setIZone(0, 0);
    extendController.setOutputRange(-1, 1);
    extendController.setSmartMotionAllowedClosedLoopError(0.1, 0);
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

    offsetTab
        .add(
            new InstantCommand(this::configureWristOffset)
                .ignoringDisable(true)
                .withName("Zero Wrist"))
        .withPosition(0, 0);
    wristOffsetEntry =
        offsetTab
            .add("Wrist Offset", Constants.WRIST_CANCODER_OFFSET)
            .withPosition(1, 0)
            .getEntry();

    wristCANCoder = new CANCoder(Constants.WRIST_CANCODER_ID);
    wristCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    wristCANCoder.configMagnetOffset(Constants.WRIST_CANCODER_OFFSET);
    wristCANCoder.configSensorDirection(true);
    wristMotor.setSelectedSensorPosition(
        wristCANCoderToIntegratedSensor(wristCANCoder.getAbsolutePosition()));

    armRotationLog = new DebugLog(0.0, "/arm/armRotation", this);
    armExtensionLog = new DebugLog(0.0, "/arm/armExtension", this);
    wristRotationLog = new DebugLog(0.0, "/arm/wristRotation", this);
    elevatorPercentOutputLog = new DebugLog(0.0, "/arm/elevatorPercentOutput", this);
    armPosLog = new DebugLog("", "/arm/armPositionOutput", this);

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {
    motorProtection();
    if (DriverStation.isDisabled()) {
      shoulderPPC.setGoal(shoulderThetaFromCANCoder());
      elevatorPPC.setGoal(elevatorCANCoder.getPosition());
    }

    DeltaBoard.putNumber("ElevatorOutput", extendMotor.get());

    double shoulderOutput;
    shoulderOutput = computeShoulderOutput();

    leftShoulder.set(shoulderOutput);

    DeltaBoard.putNumber("Arm Extension (encoder pos)", elevatorCANCoder.getPosition());
    DeltaBoard.putNumber("elevator setpoint raw", armPosition.armExtension);

    DeltaBoard.putNumber("Wrist Position (Ticks)", wristMotor.getSelectedSensorPosition());
    DeltaBoard.putNumber("shoulder angle (degrees)", Math.toDegrees(shoulderThetaFromCANCoder()));

    DeltaBoard.putNumber("Elevator Temp C", extendMotor.getMotorTemperature());

    armPosLog.log(armPosition.toString());
  }

  public void setElevatorPercent(double elevatorPercentOutput) {
    this.elevatorPercentOutput = elevatorPercentOutput;

    if (elevatorPercentOutput == 0) {
      if (elevatorInPercentControl) {
        stop();
      }
      elevatorInPercentControl = false;
    } else {
      elevatorInPercentControl = true;
    }
  }

  /**
   * Sets the position that the arm should aim for.
   *
   * @param armPosition The target arm position.
   */
  public void setTarget(ArmPosition armPosition) {
    elevatorInPercentControl = false;
    this.armPosition =
        armPosition.clamp(ArmPosition.ARM_MIN_POSITION, ArmPosition.ARM_MAX_POSITION);
    shoulderPPC.setGoal(this.armPosition.armRotation);
    elevatorPPC.setGoal(this.armPosition.armExtension);
    wristMotor.set(ControlMode.MotionMagic, this.armPosition.wristRotation);

    armRotationLog.log(armPosition.armRotation);
    armExtensionLog.log(armPosition.armExtension);
    wristRotationLog.log(armPosition.wristRotation);
  }

  /**
   * Calculates the amount of power needed to counteract the force of gravity on the arm.
   *
   * @return The power needed to keep the arm stable, in percent output.
   */
  //
  private double computeShoulderArbitraryFeedForward() {
    double mass = 4.2;

    if ((GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()).isCone())
        && armPosition.getHeight() == ArmHeight.HIGH) {
      mass += 0.45;
    }

    double theta = shoulderThetaFromCANCoder();
    double centerOfMass = computeCenterOfMass();
    double gearRatio = 90;
    double numMotors = 2;
    double torque = Math.cos(theta) * 9.81 * mass * centerOfMass;
    double out = torque / (Constants.STALLED_TORQUE * 0.85 * gearRatio * numMotors);
    return out;
  }

  private double computeCenterOfMass() {
    return ((Math.abs(elevatorCANCoder.getPosition() / 360) / 12.77) * 0.62992) + 0.457;
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

  /**
   * Returns the angle of the shoulder in radians.
   *
   * @return The angle of the shoulder in radians.
   */
  public double shoulderThetaFromCANCoder() {
    double rawPos = shoulderCANCoder.getPosition();
    DeltaBoard.putNumber("raw CANcoder", rawPos);
    double theta = Math.toRadians(rawPos * (6.0 / 16.0));
    return theta;
  }

  public double thetaFromPPC() {
    return shoulderPPC.getSetpoint().position;
  }

  private double computeShoulderOutput() {
    double output =
        shoulderPPC.calculate(shoulderThetaFromCANCoder()) + computeShoulderArbitraryFeedForward();
    return output;
  }

  private double computeElevatorOutput() {
    double output =
        elevatorPPC.calculate(elevatorCANCoder.getPosition()) + computeElevatorFeedForward();
    return output;
  }

  public double currentArmExtenstionMeters() {
    return extendEncoder.getPosition() * Math.PI * Constants.CAPSTAN_DIAMETER_METERS
        + Constants.ARM_LENGTH_AT_ZERO_TICKS_METERS;
  }

  public double currentArmExtenstionRevs() {
    return extendEncoder.getPosition();
  }

  private double computeElevatorFeedForward() {
    double theta = shoulderThetaFromCANCoder();
    double magicNumberThatMakesItWork = 0.5;
    double mass = 4.2 - magicNumberThatMakesItWork;
    double stallLoad = 22.929;
    return (mass * Math.sin(theta)) / stallLoad;
  }

  private double wristCANCoderToIntegratedSensor(double theta) {
    theta /= Constants.WRIST_GEAR_RATIO; // output shaft to input shaft
    theta /= 360; // degrees to revs
    return theta * 2048; // revs to ticks
  }

  /**
   * Reconfigures the wrist position assuming the elevator is entirely back, shoulder is entirely
   * down, and the wrist is touching the ground. The new offset is displayed so the constant can be
   * updated.
   */
  private void configureWristOffset() {
    if (DriverStation.isDisabled()) {
      wristOffsetEntry.setDouble(
          -(wristCANCoder.getAbsolutePosition() - Constants.WRIST_CANCODER_OFFSET)
              + (Constants.ZEROING_OFFSET / 2048) * 360 * Constants.WRIST_GEAR_RATIO);
      wristMotor.setSelectedSensorPosition(Constants.ZEROING_OFFSET);
    }
  }

  /**
   * Returns the current position of the arm according to the encoders
   *
   * @return
   */
  public ArmPosition getArmPosition() {
    return new ArmPosition(
        shoulderThetaFromCANCoder(),
        extendEncoder.getPosition(),
        wristMotor.getSelectedSensorPosition(),
        ArmHeight.NOT_SPECIFIED);
  }

  /**
   * Returns the position that the arm is trying to maintain
   *
   * @return The arm's goal position
   */
  public ArmPosition getArmGoalPosition() {
    return armPosition;
  }

  public double getElevatorCurrentDraw() {
    return extendMotor.getOutputCurrent();
  }

  public void setTheElevatorZero() {
    elevatorCANCoder.setPosition(0);
  }

  /** Has the arm hold it position. */
  public void stop() {
    elevatorInPercentControl = false;
    setTarget(
        new ArmPosition(
            thetaFromPPC(),
            currentArmExtenstionRevs(),
            armPosition.wristRotation,
            ArmHeight.NOT_SPECIFIED));
  }

  /**
   * Checks and outputs the elevator motor's temperature to DeltaBoard, throws an exception if it is
   * too high.
   */
  private void motorProtection() {
    double motorTemp = extendMotor.getMotorTemperature();
    DeltaBoard.putNumber("Elevator Temp C", motorTemp);

    DeltaBoard.putBoolean("ElevatorIsNominal", Constants.WARNING_MOTOR_TEMP > motorTemp);

    if (Constants.STOP_MOTOR_TEMP < motorTemp && !DriverStation.isFMSAttached()) {

      throw new OverheatedException("Elevator Motor is out of temp");
    }
  }

  public void setElevator() {
    if (!elevatorInPercentControl) {
      double elevatorCompute = computeElevatorOutput();
      extendMotor.set(elevatorCompute);
      elevatorPercentOutputLog.log(elevatorCompute);
    } else {
      extendMotor.set(elevatorPercentOutput);
      elevatorPercentOutputLog.log(elevatorPercentOutput);
    }
  }

  /**
   * @return True if the elevator is fully retracted.
   */
  public boolean stalledAtZero() {
    boolean atZero = elevatorPPC.getGoal().position == 0;
    boolean stalled =
        Math.abs(elevatorCANCoder.getVelocity())
            < 2; // moving at less than 2 degrees/sec either direction
    boolean plausablyZero =
        elevatorCANCoder.getPosition()
            < 360; // sanity check. We should never come close to hitting this slip between zeros
    boolean hasError = !elevatorPPC.atGoal(); // cant be at goal
    boolean hasDraw = extendMotor.getOutputCurrent() > 20; // must be drawing some amt of current

    return atZero && stalled && plausablyZero && hasError && hasDraw;
  }

  public boolean zeroElevator() {

    if (elevatorPPC.getGoal().position == 0) {
      System.out.println("Rezero delta:" + elevatorCANCoder.getPosition() + " degrees");
      elevatorCANCoder.setPosition(0);

      return true;
    } else {
      System.out.println("zero rejected! PPC goal not zero!");
      return false;
    }
  }

  public ArmPosition getUnbindPosition() {
    return new ArmPosition(
        shoulderThetaFromCANCoder() + Math.toRadians(5),
        extendEncoder.getPosition(),
        wristMotor.getSelectedSensorPosition(),
        ArmHeight.NOT_SPECIFIED);
  }
}
