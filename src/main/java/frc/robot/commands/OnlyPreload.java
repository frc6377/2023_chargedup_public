package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class OnlyPreload extends SequentialCommandGroup {
  public OnlyPreload(ArmSubsystem arm) {
    addCommands(
        new ArmPowerCommand(arm.getUnbindPosition(), arm, 3),
        new WaitCommand(1),
        new ScheduleCommand(
            new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3).withTimeout(0.5)));
  }
}
