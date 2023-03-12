package frc.robot.commands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SwitchTargetObject extends CommandBase {
  private final EndAffectorSubsystem endAffectorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final BooleanSubscriber isCubeSubscriber;

  public SwitchTargetObject(
      EndAffectorSubsystem endAffectorSubsystem,
      ArmSubsystem armSubsystem,
      BooleanTopic isCubeTopic) {
    this.endAffectorSubsystem = endAffectorSubsystem;
    this.armSubsystem = armSubsystem;
    addRequirements(endAffectorSubsystem, armSubsystem);
    isCubeSubscriber = isCubeTopic.subscribe(true);
  }

  @Override
  public void initialize() {
    endAffectorSubsystem.toggleGamePiece();
    ArmHeight currentHeight = armSubsystem.getArmPosition().getHeight();
    armSubsystem.setTarget(
        ArmPosition.getArmPositionFromHeightAndType(currentHeight, isCubeSubscriber.get()));
  }

  public boolean isFinished() {
    return true;
  }
}
