package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndAffectorSubsystem extends SubsystemBase {

  CANSparkMax leftMotor;

  CANSparkMax rightMotor;

  public EndAffectorSubsystem(int leftID, int rightID) {

    leftMotor = new CANSparkMax(leftID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(rightID, MotorType.kBrushless);

    leftMotor.setSmartCurrentLimit(40);
    rightMotor.setSmartCurrentLimit(20);
  }

  public void intake() {

    rightMotor.set(-Constants.END_AFFECTOR_INTAKE_SPEED);
    leftMotor.set(Constants.END_AFFECTOR_INTAKE_SPEED);
  }

  public void fastOutake() {

    rightMotor.set(Constants.END_AFFECTOR_OUTTAKE_SPEED);
    leftMotor.set(-Constants.END_AFFECTOR_OUTTAKE_SPEED);
  }

  public void slowOutake() {

    rightMotor.set(Constants.END_AFFECTOR_SLOW_OUTTAKE_SPEED);
    leftMotor.set(-Constants.END_AFFECTOR_SLOW_OUTTAKE_SPEED);
  }

  public void halt() {

    rightMotor.set(0);
    leftMotor.set(0);
  }

  public void idle() {

    rightMotor.set(-Constants.END_AFFECTOR_IDLE_SPEED);
    leftMotor.set(Constants.END_AFFECTOR_IDLE_SPEED);
  }
}
