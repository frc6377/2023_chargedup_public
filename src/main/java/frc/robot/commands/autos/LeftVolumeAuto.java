package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmPowerCommand;
import frc.robot.commands.SwerveAutoFactory;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class LeftVolumeAuto extends SequentialCommandGroup {
  public LeftVolumeAuto(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      EndAffectorSubsystem endAffector,
      ArmSubsystem arm) {
    addCommands(
        new InstantCommand(() -> endAffector.setCube()),
        new InstantCommand(() -> endAffector.intake()),
        factory
            .generateCommandFromFile("LeftVolumeFirstElement", true)
            .alongWith(new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3)),
        new InstantCommand(() -> endAffector.idle()),
        factory
            .generateCommandFromFile("LeftVolumeScoreFirst", false)
            .alongWith(new ArmPowerCommand(Constants.MID_CUBE_ARM_POSITION, arm, 3)),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new InstantCommand(() -> endAffector.idle()),
        new WaitCommand(0.2),
        new InstantCommand(() -> endAffector.intake()),
        factory
            .generateCommandFromFile("LeftVolumeSecondElement", false)
            .alongWith(new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3)),
        new InstantCommand(() -> endAffector.idle()),
        factory
            .generateCommandFromFile("LeftVolumeScoreSecond", false)
            .alongWith(new ArmPowerCommand(Constants.MID_CUBE_ARM_POSITION, arm, 3)),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new InstantCommand(() -> endAffector.idle()),
        new WaitCommand(0.2),
        new InstantCommand(() -> endAffector.intake()),
        factory
            .generateCommandFromFile("LeftVolumeThirdElement", false)
            .alongWith(new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3)),
        new InstantCommand(() -> endAffector.idle()),
        factory
            .generateCommandFromFile("LeftVolumeScoreThird", false)
            .alongWith(new ArmPowerCommand(Constants.MID_CUBE_ARM_POSITION, arm, 3)),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new InstantCommand(() -> endAffector.idle()),
        factory
            .generateCommandFromFile("LeftVolumeBackup", false)
            .alongWith(new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3)));
  }
}
