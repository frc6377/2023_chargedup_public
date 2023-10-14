package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.color.GamePieceMode;

public class EndAffectorSubsystem extends SubsystemBase {
  private final WPI_TalonFX motor;
  private final IntegerSubscriber gamePieceModeSubscriber =
      NetworkTableInstance.getDefault().getIntegerTopic("GAME_PIECE_MODE").subscribe(10);

  public EndAffectorSubsystem(int ID, double kP) {

    motor = new WPI_TalonFX(ID);
    motor.configOpenloopRamp(0.0);
    motor.config_kP(0, kP);
  }

  public void intake() {
    motor.set(
        Constants.END_AFFECTOR_INTAKE_SPEED
            * ((GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()).isCube()) ? -1 : 1));
  }

  public void fastOutake() {
    motor.set(
        -Constants.END_AFFECTOR_OUTTAKE_SPEED
            * ((GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()).isCube())
                ? -300
                : 30));
  }

  public double getIntakePosition() {
    return motor.getSelectedSensorPosition();
  }

  public void partialEject(double target) {
    motor.set(ControlMode.Position, target);
  }

  public void slowOutake() {
    motor.set(
        -Constants.END_AFFECTOR_SLOW_OUTTAKE_SPEED
            * ((GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()).isCube())
                ? -1
                : 300));
  }

  public void halt() {
    motor.set(0);
  }

  public void idle() {
    motor.set(
        Constants.END_AFFECTOR_IDLE_SPEED
            * ((GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()).isCube()) ? -1 : 1));
  }

  public double getVelocity() {
    return motor.getSelectedSensorVelocity();
  }
}
