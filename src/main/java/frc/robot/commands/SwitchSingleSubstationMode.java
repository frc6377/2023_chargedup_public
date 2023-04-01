package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.GamePieceMode;
import frc.robot.subsystems.color.SignalingSubsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SwitchSingleSubstationMode extends CommandBase {
  private final EndAffectorSubsystem endAffectorSubsystem;
  private final SignalingSubsystem colorSubsystem;
  private final Consumer<GamePieceMode> gamePieceModeConsumer;
  private final Supplier<GamePieceMode> gamePieceModeSupplier;
  private final ArmSubsystem armSubsystem;

  public SwitchSingleSubstationMode(
      EndAffectorSubsystem endAffectorSubsystem,
      SignalingSubsystem colorSubsystem,
      Consumer<GamePieceMode> gamePieceModeConsumer,
      Supplier<GamePieceMode> gamePieceModeSupplier,
      ArmSubsystem armSubsystem) {
    this.endAffectorSubsystem = endAffectorSubsystem;
    this.colorSubsystem = colorSubsystem;
    addRequirements(endAffectorSubsystem, colorSubsystem);
    this.gamePieceModeConsumer = gamePieceModeConsumer;
    this.gamePieceModeSupplier = gamePieceModeSupplier;
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    if (gamePieceModeSupplier.get() == GamePieceMode.SINGLE_SUBSTATION) {
      gamePieceModeConsumer.accept(GamePieceMode.CONE);
      endAffectorSubsystem.setGamePiece(GamePieceMode.CONE);
      colorSubsystem.setGamePiece(GamePieceMode.CONE);
    } else {
      gamePieceModeConsumer.accept(GamePieceMode.SINGLE_SUBSTATION);
      endAffectorSubsystem.setGamePiece(GamePieceMode.SINGLE_SUBSTATION);
      colorSubsystem.setGamePiece(GamePieceMode.SINGLE_SUBSTATION);
    }
    ArmHeight currentHeight = armSubsystem.getArmGoalPosition().getHeight();
    armSubsystem.setTarget(
        ArmPosition.getArmPositionFromHeightAndType(currentHeight, gamePieceModeSupplier.get()));
  }

  public boolean isFinished() {
    return true;
  }
}
