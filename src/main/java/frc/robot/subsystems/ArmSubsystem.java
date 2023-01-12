package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax motor;
  SparkMaxPIDController controller;
  RelativeEncoder encoder;

  public ArmSubsystem(int ID) {

    motor = new CANSparkMax(ID, MotorType.kBrushless);

    encoder = motor.getEncoder();
    controller = motor.getPIDController();
    motor.getPIDController().setFF(Constants.armKf);
    motor.getPIDController().setP(Constants.armKp);

    motor.setSmartCurrentLimit(40);
    controller.setSmartMotionMaxVelocity(Constants.armMaxvelo, 0);
    controller.setSmartMotionMaxAccel(Constants.armMaxAccel, 0);
  }

  private void setLow() {
    setPosition(Constants.armPositionLow);
  }

  private void setMid() {
    setPosition(Constants.armPositionMid);
  }

  private void sethHigh() {
    setPosition(Constants.armPositionHigh);
  }

  private void setPosition(int position) {
    controller.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }
}
