package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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
  private final SparkMaxPIDController controller;
  private final RelativeEncoder encoder;
  private final ProfiledPIDController ppc;

  // Dashboard elements
  private GenericEntry armHighEntry;
  private GenericEntry armMidEntry;
  private GenericEntry armLowEntry;

  public ArmSubsystem(int ID) {
    System.out.println("Starting Construct ArmSubsystem");

    ppc = new ProfiledPIDController(0.4, 0, 0, new TrapezoidProfile.Constraints(30, 30));
    motor = new CANSparkMax(ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    controller = motor.getPIDController();

    controller.setFF(0);
    controller.setP(0.045);
    controller.setI(0);
    controller.setD(1);
    controller.setIZone(0);
    // controller.setOutputRange(-1, 1);

    controller.setFeedbackDevice(encoder);

    motor.setSmartCurrentLimit(40);
    // controller.setSmartMotionMaxVelocity(Constants.armMaxvelo, 0);
    // controller.setSmartMotionMaxAccel(Constants.armMaxAccel, 0);

    var layout = tab.getLayout("Height", BuiltInLayouts.kGrid);
    layout.addDouble("Current", this::getPosition).withWidget(BuiltInWidgets.kDial);
    armHighEntry =
        layout
            .addPersistent("High Value", -Constants.armPositionHigh)
            .getEntry(DoubleTopic.kTypeString);
    armMidEntry =
        layout
            .addPersistent("Mid Value", -Constants.armPositionMid)
            .getEntry(DoubleTopic.kTypeString);
    armLowEntry =
        layout
            .addPersistent("Low Value", -Constants.armPositionLow)
            .getEntry(DoubleTopic.kTypeString);
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

  public void setLow() {
    setPosition(armLowEntry.getDouble(Constants.armPositionLow));
  }

  public void setMid() {
    setPosition(armMidEntry.getDouble(Constants.armPositionMid));
  }

  public void setHigh() {
    setPosition(armHighEntry.getDouble(Constants.armPositionHigh));
  }

  private double computeArbitraryFeetForward() {
    double theta = encoder.getPosition() * Math.PI / 50;
    return (3 * Math.cos(theta - Math.toRadians(9.3)) / 328);
  }

  private void setPosition(double position) {
    // controller.setReference(position, CANSparkMax.ControlType.kPosition);
    ppc.setGoal(position);
  }

  private double getPosition() {
    return encoder.getPosition();
  }
}
