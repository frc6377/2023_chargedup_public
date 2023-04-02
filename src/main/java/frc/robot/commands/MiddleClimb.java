package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class MiddleClimb extends SequentialCommandGroup {
  public MiddleClimb(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector) {
    addCommands(
      new ArmPowerCommand(arm.getUnbindPosition(), arm, 3),
        new WaitCommand(0.75),
        new ScheduleCommand(
            new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3).withTimeout(0.5)),
        new WaitCommand(1),
        new ScheduleCommand(
            new ArmPowerCommand(Constants.STOWED_ARM_POSITION, arm, 3).withTimeout(0.5)),
        new WaitCommand(1),
        factory.generateCommandFromFile("ClimbMiddle", true),
        new AutoBalanceCommand(drive));
  }
}
