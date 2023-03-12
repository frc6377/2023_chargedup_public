package frc.robot.commands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.ColorSubsystem;

public class SwitchTargetObject extends CommandBase {
  private final EndAffectorSubsystem endAffectorSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ColorSubsystem colorSubsystem;
  private final BooleanPublisher isCubePublisher;
  private final BooleanSubscriber isCubeSubscriber;

  public SwitchTargetObject(
      EndAffectorSubsystem endAffectorSubsystem,
      ArmSubsystem armSubsystem,
      ColorSubsystem colorSubsystem,
      BooleanTopic isCubeTopic) {
    this.endAffectorSubsystem = endAffectorSubsystem;
    this.armSubsystem = armSubsystem;
    this.colorSubsystem = colorSubsystem;
    addRequirements(endAffectorSubsystem, armSubsystem, colorSubsystem);
    this.isCubeSubscriber = isCubeTopic.subscribe(true);
    this.isCubePublisher = isCubeTopic.publish();
  }

  @Override
  public void initialize() {
    final boolean isCube = !isCubeSubscriber.get();
    isCubePublisher.set(isCube);

    endAffectorSubsystem.setGamePiece(isCube);
    colorSubsystem.updateLEDs(isCube);
    ArmHeight currentHeight = armSubsystem.getArmPosition().getHeight();
    armSubsystem.setTarget(
        ArmPosition.getArmPositionFromHeightAndType(currentHeight, isCube));
  }

  public boolean isFinished() {
    return true;
  }
}
