package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class LeftTwoElementNoClimb extends SequentialCommandGroup {
  public LeftTwoElementNoClimb(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector) {
    addCommands(
        new InstantCommand(() -> endAffector.intake()),
        new WaitCommand(0.75),
        new ScheduleCommand(
            new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3).withTimeout(0.5)),
        new WaitCommand(0.75),
        factory.generateCommandFromFile("PickFirstElementBlue", true),
        new ScheduleCommand(
            new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3).withTimeout(0.5)),
        new InstantCommand(() -> endAffector.idle()),
        new ScheduleCommand(
            new WaitCommand(1)
                .andThen(
                    new ArmPowerCommand(Constants.MID_CUBE_ARM_POSITION, arm, 3).withTimeout(1.5))),
        factory.generateCommandFromFile("ScoreFirstElementBlue", false),
        new WaitCommand(0.5),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new ScheduleCommand(
            new WaitCommand(1.5)
                .andThen(
                    new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3).withTimeout(0.5))),
        new InstantCommand(() -> endAffector.intake()),
        factory.generateCommandFromFile("PickSecondElementBlue", false));
  }
}