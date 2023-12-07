package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.SignalingSubsystem;

public class IntakeCommand extends CommandBase {

  private final EndAffectorSubsystem subsystem;
  private final SignalingSubsystem signalingSubsystem;
  private final Timer startIntakeTimer = new Timer();
  private final ArmSubsystem armSubsystem;

  public IntakeCommand(
      EndAffectorSubsystem endAffectorSubsystem,
      SignalingSubsystem signalingSubsystem,
      ArmSubsystem armSubsystem) {
    this.subsystem = endAffectorSubsystem;
    this.signalingSubsystem = signalingSubsystem;
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    subsystem.intake();
    startIntakeTimer.start();
    startIntakeTimer.reset();
  }

  @Override
  public void execute() {
    if (startIntakeTimer.hasElapsed(Constants.GAME_PIECE_DETECTION_WAIT)) {
      if (belowThreshhold()) {
        signalingSubsystem.hasGamePieceSignalStart();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return (startIntakeTimer.hasElapsed(Constants.GAME_PIECE_DETECTION_WAIT)
        && belowThreshhold()
        && armSubsystem.getArmGoalPosition().getHeight() == ArmHeight.LOW);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      armSubsystem.setTarget(ArmPosition.HYBRID_CUBE_ARM_POSITION);
    }
    signalingSubsystem.hasGamePieceSignalStop();
    subsystem.idle();
  }

  private boolean belowThreshhold() {
    return Math.abs(subsystem.getVelocity()) < Constants.GAME_PIECE_DETECTION_VELOCITY;
  }
}
