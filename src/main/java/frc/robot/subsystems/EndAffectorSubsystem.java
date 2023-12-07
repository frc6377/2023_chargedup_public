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

  /**
   * Handles the intake
   * @param ID The intake motor's ID
   * @param kP the 
   */
  public EndAffectorSubsystem(int ID) {

    motor = new WPI_TalonFX(ID);
    motor.configOpenloopRamp(0.0);
    motor.config_kP(0, Constants.END_AFFECTOR_KP);
  }

  /**
   * Takes in game pieces
   */
  public void intake() {
    motor.set(
        Constants.END_AFFECTOR_INTAKE_SPEED
            * ((GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()).isCube()) ? -1 : 1));
  }

  /**
   * Ejects game pieces
   */
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

  /**
   * Ejects at partial speed
   * @param target Speed to eject at
   */
  public void partialEject(double target) {
    motor.set(ControlMode.Position, target);
  }

  /**
   * Stops the intake
   */
  public void halt() {
    motor.set(0);
  }

  /**
   * Makes the intake run, but only enough to hold a game piece.
   */
  public void idle() {
    motor.set(
        Constants.END_AFFECTOR_IDLE_SPEED
            * ((GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()).isCube()) ? -1 : 1));
  }

  public double getVelocity() {
    return motor.getSelectedSensorVelocity();
  }
}
