package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndAffectorSubsystem extends SubsystemBase {
  private WPI_TalonFX motor;
  private boolean isCube;

  public EndAffectorSubsystem(int ID, double kP) {

    motor = new WPI_TalonFX(ID);
    this.isCube = isCube;
    motor.configOpenloopRamp(0.0);
    motor.config_kP(0, kP);
  }

  public void toggleGamePiece() {
    isCube = !isCube;
  }

  public void setGamePiece(final boolean isCube) {
    this.isCube = isCube;
  }

  public void setCube() {
    isCube = true;
  }

  public void setCone() {
    isCube = false;
  }

  public void intake() {
    motor.set(Constants.END_AFFECTOR_INTAKE_SPEED * (isCube ? -1 : 1));
  }

  public void fastOutake() {
    motor.set(-Constants.END_AFFECTOR_OUTTAKE_SPEED * (isCube ? -1 : 30));
  }

  public double getIntakePosition() {
    return motor.getSelectedSensorPosition();
  }

  public void partialEject(double target) {
    motor.set(ControlMode.Position, target);
  }

  public void slowOutake() {
    motor.set(-Constants.END_AFFECTOR_SLOW_OUTTAKE_SPEED * (isCube ? -1 : 300));
  }

  public void halt() {
    motor.set(0);
  }

  public void idle() {
    motor.set(Constants.END_AFFECTOR_IDLE_SPEED * (isCube ? -1 : 1));
  }

  public double getVelocity() {
    return motor.getSelectedSensorVelocity();
  }
}
