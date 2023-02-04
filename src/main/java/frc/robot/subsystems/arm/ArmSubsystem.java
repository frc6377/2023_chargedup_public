package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  // Hardware
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final ProfiledPIDController ppc;
  private final ArmExtender extender;

  // Dashboard elements
  private GenericEntry armHighEntry;
  private GenericEntry armMidEntry;
  private GenericEntry armLowEntry;

  public ArmSubsystem(int rotateID, int extendID) {
    System.out.println("Starting Construct ArmSubsystem");

    ppc = new ProfiledPIDController(0.4, 0, 0, new TrapezoidProfile.Constraints(30, 30));
    motor = new CANSparkMax(rotateID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    motor.setSmartCurrentLimit(40);

    var layout = tab.getLayout("Height", BuiltInLayouts.kGrid);
    layout.addDouble("Current", this::getPosition).withWidget(BuiltInWidgets.kDial);
    armHighEntry =
        layout
            .addPersistent("High Value", -Constants.armRotationHigh)
            .getEntry(DoubleTopic.kTypeString);
    armMidEntry =
        layout
            .addPersistent("Mid Value", -Constants.armRotationMid)
            .getEntry(DoubleTopic.kTypeString);
    armLowEntry =
        layout
            .addPersistent("Low Value", -Constants.armRotationLow)
            .getEntry(DoubleTopic.kTypeString);

    // TODO: when we have a real extension arm, change this to `ActiveArmExtender`
    extender = new DisabledArmExtender(extendID);

    System.out.println("Complete Construct ArmSubsystem");
  }

  @Override
  public void periodic() {
    motor.set(ppc.calculate(encoder.getPosition()) - computeArbitraryFeetForward());
    SmartDashboard.putNumber("encoder pos", encoder.getPosition());
    SmartDashboard.putNumber("ppc error", ppc.getPositionError());
    SmartDashboard.putNumber("setpoint", ppc.getSetpoint().position);
    SmartDashboard.putNumber("goal", ppc.getGoal().position);
    SmartDashboard.putNumber("arbitrary ffw", computeArbitraryFeetForward());
  }

  public void setStowed() {
    setPosition(armLowEntry.getDouble(Constants.armRotationStowed));
    extender.setLength(Constants.armLengthStowed);
  }

  public void setLow() {
    setPosition(Constants.armRotationLow);
    extender.setLength(Constants.armLengthLow);
  }

  public void setMid() {
    setPosition(Constants.armRotationMid);
    extender.setLength(Constants.armLengthMid);
  }

  public void setHigh() {
    setPosition(Constants.armRotationHigh);
    extender.setLength(Constants.armLengthHigh);
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

  public void setPositionDegrees(double degrees){
    setPosition(Math.toRadians(degrees)/Constants.armRotationalTicksToRadians);
  }

  private double getPosition() {
    return encoder.getPosition();
  }
}
