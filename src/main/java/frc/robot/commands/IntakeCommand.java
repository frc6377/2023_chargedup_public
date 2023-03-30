package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.color.SignalingSubsystem;

public class IntakeCommand extends CommandBase {

  private final EndAffectorSubsystem subsystem;
  private final SignalingSubsystem signalingSubsystem;
  private Timer startIntakeTimer = new Timer();

  public IntakeCommand(
      EndAffectorSubsystem endAffectorSubsystem, SignalingSubsystem signalingSubsystem) {
    this.subsystem = endAffectorSubsystem;
    this.signalingSubsystem = signalingSubsystem;
  }

  @Override
  public void initialize() {
    startIntakeTimer.start();
    subsystem.intake();
  }

  @Override
  public void execute() {
    if (startIntakeTimer.hasElapsed(Constants.GAME_PIECE_DETECTION_WAIT)) {
      if (Math.abs(subsystem.getVelocity()) < Constants.GAME_PIECE_DETECTION_VELOCITY) {
        signalingSubsystem.hasGamePieceSignalStart();
      } else signalingSubsystem.hasGamePieceSignalStop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    signalingSubsystem.hasGamePieceSignalStop();
    subsystem.idle();
  }
}
