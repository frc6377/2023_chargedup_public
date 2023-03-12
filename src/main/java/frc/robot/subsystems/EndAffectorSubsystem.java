package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndAffectorSubsystem extends SubsystemBase {
  private WPI_TalonFX motor;
  private boolean isCube;

  public EndAffectorSubsystem(int ID, final boolean isCube) {
    motor = new WPI_TalonFX(ID);
    motor.configOpenloopRamp(0.25);
    this.isCube = isCube;
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
    motor.set(-Constants.END_AFFECTOR_OUTTAKE_SPEED * (isCube ? -1 : 3));
  }

  public void slowOutake() {
    motor.set(-Constants.END_AFFECTOR_SLOW_OUTTAKE_SPEED * (isCube ? -1 : 3));
  }

  public void halt() {
    motor.set(0);
  }

  public void idle() {
    motor.set(Constants.END_AFFECTOR_IDLE_SPEED * (isCube ? -1 : 1));
  }
}
