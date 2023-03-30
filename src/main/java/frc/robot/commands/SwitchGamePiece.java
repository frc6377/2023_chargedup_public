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

public class SwitchGamePiece extends CommandBase {
  private final EndAffectorSubsystem endAffectorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final SignalingSubsystem colorSubsystem;
  private final Consumer<GamePieceMode> gamePieceModeConsumer;
  private final Supplier<GamePieceMode> gamePieceModeSupplier;

  public SwitchGamePiece(
      EndAffectorSubsystem endAffectorSubsystem,
      ArmSubsystem armSubsystem,
      SignalingSubsystem colorSubsystem,
      Consumer<GamePieceMode> gamePieceModeConsumer,
      Supplier<GamePieceMode> gamePieceModeSupplier) {
    this.endAffectorSubsystem = endAffectorSubsystem;
    this.armSubsystem = armSubsystem;
    this.colorSubsystem = colorSubsystem;
    addRequirements(endAffectorSubsystem, armSubsystem, colorSubsystem);
    this.gamePieceModeConsumer = gamePieceModeConsumer;
    this.gamePieceModeSupplier = gamePieceModeSupplier;
  }

  @Override
  public void initialize() {
    GamePieceMode goalMode =
        gamePieceModeSupplier.get().isCube() ? GamePieceMode.CONE : GamePieceMode.CUBE;
    gamePieceModeConsumer.accept(goalMode);
    endAffectorSubsystem.setGamePiece(goalMode);
    colorSubsystem.setGamePiece(goalMode);
    ArmHeight currentHeight = armSubsystem.getArmGoalPosition().getHeight();
    System.out.println(currentHeight);
    armSubsystem.setTarget(ArmPosition.getArmPositionFromHeightAndType(currentHeight, goalMode));
  }

  public boolean isFinished() {
    return true;
  }
}
