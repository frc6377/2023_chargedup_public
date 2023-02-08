package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndAffectorSubsystem extends SubsystemBase {

  CANSparkMax motor;

  public EndAffectorSubsystem(int ID) {

    motor = new CANSparkMax(ID, MotorType.kBrushless);

    motor.setSmartCurrentLimit(40);
  }

  public void intake() {
    motor.set(Constants.END_AFFECTOR_INTAKE_SPEED);
  }

  public void fastOutake() {
    motor.set(-Constants.END_AFFECTOR_OUTTAKE_SPEED);
  }

  public void slowOutake() {
    motor.set(-Constants.END_AFFECTOR_SLOW_OUTTAKE_SPEED);
  }

  public void halt() {
    motor.set(0);
  }

  public void idle() {
    motor.set(Constants.END_AFFECTOR_IDLE_SPEED);
  }
}
