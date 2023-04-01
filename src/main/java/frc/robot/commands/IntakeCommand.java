package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.GamePieceMode;
import frc.robot.subsystems.color.SignalingSubsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class IntakeCommand extends CommandBase {

  private final EndAffectorSubsystem subsystem;
  private final SignalingSubsystem signalingSubsystem;
  private final Timer startIntakeTimer = new Timer();
  private final Consumer<GamePieceMode> gamePieceModeConsumer;
  private final Supplier<GamePieceMode> gamePieceModeSupplier;
  private final ArmSubsystem armSubsystem;

  public IntakeCommand(
      EndAffectorSubsystem endAffectorSubsystem,
      SignalingSubsystem signalingSubsystem,
      Consumer<GamePieceMode> gamePieceModeConsumer,
      Supplier<GamePieceMode> gamePieceModeSupplier,
      ArmSubsystem armSubsystem) {
    this.subsystem = endAffectorSubsystem;
    this.signalingSubsystem = signalingSubsystem;
    this.gamePieceModeConsumer = gamePieceModeConsumer;
    this.gamePieceModeSupplier = gamePieceModeSupplier;
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    startIntakeTimer.start();
    subsystem.intake();
  }

  @Override
  public void execute() {
    if (startIntakeTimer.hasElapsed(Constants.GAME_PIECE_DETECTION_WAIT)) {
      if (detectGamePiece()) {
        signalingSubsystem.hasGamePieceSignalStart();
      } else {
        signalingSubsystem.hasGamePieceSignalStop();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (detectGamePiece()) {
      if (gamePieceModeSupplier.get() == GamePieceMode.SINGLE_SUBSTATION) {
        new SwitchSingleSubstationMode(
                subsystem, signalingSubsystem, gamePieceModeConsumer, gamePieceModeSupplier)
            .initialize();
      }
      armSubsystem.setTarget(Constants.STOWED_ARM_POSITION);
    }
    signalingSubsystem.hasGamePieceSignalStop();
    subsystem.idle();
  }

  private boolean detectGamePiece() {
    return Math.abs(subsystem.getVelocity()) < Constants.GAME_PIECE_DETECTION_VELOCITY;
  }
}
