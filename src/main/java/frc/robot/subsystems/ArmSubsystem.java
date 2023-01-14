package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax motor;
  SparkMaxPIDController controller;
  RelativeEncoder encoder;

  public ArmSubsystem(int ID) {

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
    controller.setSmartMotionMaxVelocity(Constants.armMaxvelo, 0);
    controller.setSmartMotionMaxAccel(Constants.armMaxAccel, 0);
  }

  @Override
  public void periodic() {
    System.out.println(encoder.getPosition());
  }

  public void setLow() {
    setPosition(Constants.armPositionLow);
  }

  public void setMid() {
    setPosition(Constants.armPositionMid);
  }

  public void sethHigh() {
    setPosition(Constants.armPositionHigh);
  }

  private void setPosition(double position) {
    controller.setReference(position, CANSparkMax.ControlType.kPosition);
  }
}
