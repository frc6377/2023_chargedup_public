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
  }

  public void intake() {

    rightMotor.set(-Constants.endAffectorIntakespeed);
    leftMotor.set(Constants.endAffectorIntakespeed);
  }

  public void outake() {

    rightMotor.set(Constants.endAffectorIntakespeed);
    leftMotor.set(-Constants.endAffectorIntakespeed);
  }

  public void halt() {

    rightMotor.set(0);
    leftMotor.set(0);
  }

  public void idle() {

    rightMotor.set(-Constants.endAffectorIdleSpeed);
    leftMotor.set(Constants.endAffectorIdleSpeed);   
  }
}
