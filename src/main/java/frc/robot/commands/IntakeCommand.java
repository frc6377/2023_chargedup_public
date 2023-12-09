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

  private final EndAffectorSubsystem endAffectorSubsystem;
  private final SignalingSubsystem signalingSubsystem;
  private final Timer startIntakeTimer = new Timer();
  private final ArmSubsystem armSubsystem;
  /**
   * Runs the intake and automatically stows the arm and signals if it detects a game piece
   *
   * @param endAffectorSubsystem End affector subsystem
   * @param signalingSubsystem Signaling subsystem
   * @param armSubsystem Arm subsystem
   */
  public IntakeCommand(
      EndAffectorSubsystem endAffectorSubsystem,
      SignalingSubsystem signalingSubsystem,
      ArmSubsystem armSubsystem) {
    this.endAffectorSubsystem = endAffectorSubsystem;
    this.signalingSubsystem = signalingSubsystem;
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    endAffectorSubsystem.intake();
    startIntakeTimer.start();
    startIntakeTimer.reset();
  }

  @Override
  public void execute() {
    // Checks if the motor isn't moving and enough time has passed. If it has stopped, there is most
    // likely a game piece. We need to wait because the motor takes time to start up.
    if (startIntakeTimer.hasElapsed(Constants.GAME_PIECE_DETECTION_WAIT)) {
      if (belowThreshhold()) {
        signalingSubsystem.hasGamePieceSignalStart();
      }
    }
  }

  @Override
  public boolean isFinished() {
    // Ends when it detects a game piece and the arm is in pickup position.
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
    endAffectorSubsystem.idle();
  }

  private boolean belowThreshhold() {
    return Math.abs(endAffectorSubsystem.getVelocity()) < Constants.GAME_PIECE_DETECTION_VELOCITY;
  }
}
