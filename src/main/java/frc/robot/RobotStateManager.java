package frc.robot;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.color.GamePieceMode;

public class RobotStateManager extends SubsystemBase {
  public final IntegerTopic gamePieceModeTopic =
      NetworkTableInstance.getDefault().getIntegerTopic("GAME_PIECE_MODE");
  public final IntegerTopic armTargetHeightTopic =
      NetworkTableInstance.getDefault().getIntegerTopic("ARM_HEIGHT_TOPIC");
  public final IntegerPublisher gamePieceModePublisher = gamePieceModeTopic.publish();
  public final IntegerPublisher armTargetHeightPublisher = armTargetHeightTopic.publish();
  public final IntegerSubscriber gamePieceModeSubscriber = gamePieceModeTopic.subscribe(10);
  public final IntegerSubscriber armTargetHeightSubscriber = armTargetHeightTopic.subscribe(50);

  public RobotStateManager(GamePieceMode gamePieceMode, ArmHeight armTargetHeight) {
    this.gamePieceModePublisher.accept(gamePieceMode.getAsInt());
    this.armTargetHeightPublisher.accept(armTargetHeight.getAsInt());
  }

  public void SwitchCubeCone() {
    GamePieceMode currentGamePieceMode =
        GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get());
    gamePieceModePublisher.accept(
        (currentGamePieceMode.isCube() ? GamePieceMode.CONE : GamePieceMode.CUBE).getAsInt());
  }

  public void ToggleSingleSubstationMode() {
    GamePieceMode currentGamePieceMode =
        GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get());
    System.out.println(currentGamePieceMode);
    gamePieceModePublisher.accept(
        (currentGamePieceMode == GamePieceMode.SINGLE_SUBSTATION
                ? GamePieceMode.CONE
                : GamePieceMode.SINGLE_SUBSTATION)
            .getAsInt());
  }

  public void setGamePieceMode(GamePieceMode mode) {
    gamePieceModePublisher.accept(mode.getAsInt());
  }

  public void setArmTargetHeight(ArmHeight height) {
    armTargetHeightPublisher.accept(height.getAsInt());
  }

  public GamePieceMode getGamePieceMode() {
    return GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get());
  }
}
