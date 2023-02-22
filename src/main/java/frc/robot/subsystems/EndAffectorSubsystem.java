package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndAffectorSubsystem extends SubsystemBase {
  private WPI_TalonFX motor;
  private boolean isCube = true;
  private final BooleanPublisher isCubePublisher;

  public EndAffectorSubsystem(int ID, BooleanTopic isCubeTopic) {
    motor = new WPI_TalonFX(ID);

    isCubePublisher = isCubeTopic.publish();
    isCubePublisher.set(isCube);
  }

  public void toggleGamePiece() {
    isCube = !isCube;
    isCubePublisher.set(isCube);
  }

  public void setCube() {
    isCube = true;
    isCubePublisher.set(isCube);
  }

  public void setCone() {
    isCube = false;
    isCubePublisher.set(isCube);
  }

  public void intake() {
    motor.set(Constants.END_AFFECTOR_INTAKE_SPEED * (isCube ? -1 : 1));
  }

  public void fastOutake() {
    motor.set(-Constants.END_AFFECTOR_OUTTAKE_SPEED * (isCube ? -1 : 1));
  }

  public void slowOutake() {
    motor.set(-Constants.END_AFFECTOR_SLOW_OUTTAKE_SPEED * (isCube ? -1 : 1));
  }

  public void halt() {
    motor.set(0);
  }

  public void idle() {
    motor.set(Constants.END_AFFECTOR_IDLE_SPEED * (isCube ? -1 : 1));
  }
}
