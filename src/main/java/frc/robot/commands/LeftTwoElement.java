package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class LeftTwoElement extends SequentialCommandGroup {
  public LeftTwoElement(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector) {
    addCommands(
        new ArmPowerCommand(arm.getUnbindPosition(), arm, 3),
        new InstantCommand(() -> endAffector.setCube()),
        new InstantCommand(() -> endAffector.intake()),
        new WaitCommand(0.75),
        new ScheduleCommand(
            new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3).withTimeout(0.5)),
        new WaitCommand(0.5),
        factory.generateCommandFromFile("PickFirstElementBlue", true),
        new ScheduleCommand(
            new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3).withTimeout(0.5)),
        new ScheduleCommand(
            new WaitCommand(2)
                .andThen(
                    new ArmPowerCommand(Constants.HIGH_CUBE_ARM_POSITION, arm, 3)
                        .withTimeout(1.5))),
        new InstantCommand(() -> endAffector.idle()),
        factory.generateCommandFromFile("ScoreFirstElementBlue", false),
        new WaitCommand(0.75),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new ScheduleCommand(
            new WaitCommand(0.5)
                .andThen(
                    new ArmPowerCommand(Constants.STOWED_ARM_POSITION, arm, 3).withTimeout(0.5))),
        new InstantCommand(() -> endAffector.idle()),
        factory.generateCommandFromFile("ClimbBlue", false),
        new AutoBalanceCommand(drive));
  }
}
