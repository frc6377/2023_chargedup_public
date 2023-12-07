package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotStateManager;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;

public class OnlyPreload extends SequentialCommandGroup {
  public OnlyPreload(ArmSubsystem arm, RobotStateManager robotState) {
    addCommands(
        new ArmPowerCommand(arm.getUnbindPosition(), arm, 3, robotState),
        new WaitCommand(1),
        new ScheduleCommand(
            new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState)
                .withTimeout(0.5)));
  }
}
