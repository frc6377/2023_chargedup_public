package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotStateManager;
import frc.robot.commands.ArmPowerCommand;
import frc.robot.commands.SwerveAutoFactory;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.GamePieceMode;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class LeftVolumeAuto extends SequentialCommandGroup {
  public LeftVolumeAuto(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      EndAffectorSubsystem endAffector,
      ArmSubsystem arm,
      RobotStateManager robotState) {
    addCommands(
        new InstantCommand(() -> robotState.setGamePieceMode(GamePieceMode.CUBE)),
        new InstantCommand(() -> endAffector.intake()),
        factory
            .generateCommandFromFile("LeftVolumeFirstElement", true)
            .alongWith(new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState)),
        new InstantCommand(() -> endAffector.idle()),
        factory
            .generateCommandFromFile("LeftVolumeScoreFirst", false)
            .alongWith(new ArmPowerCommand(ArmPosition.MID_CUBE_ARM_POSITION, arm, 3, robotState)),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new InstantCommand(() -> endAffector.idle()),
        new WaitCommand(0.2),
        new InstantCommand(() -> endAffector.intake()),
        factory
            .generateCommandFromFile("LeftVolumeSecondElement", false)
            .alongWith(new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState)),
        new InstantCommand(() -> endAffector.idle()),
        factory
            .generateCommandFromFile("LeftVolumeScoreSecond", false)
            .alongWith(new ArmPowerCommand(ArmPosition.MID_CUBE_ARM_POSITION, arm, 3, robotState)),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new InstantCommand(() -> endAffector.idle()),
        new WaitCommand(0.2),
        new InstantCommand(() -> endAffector.intake()),
        factory
            .generateCommandFromFile("LeftVolumeThirdElement", false)
            .alongWith(new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState)),
        new InstantCommand(() -> endAffector.idle()),
        factory
            .generateCommandFromFile("LeftVolumeScoreThird", false)
            .alongWith(new ArmPowerCommand(ArmPosition.MID_CUBE_ARM_POSITION, arm, 3, robotState)),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new InstantCommand(() -> endAffector.idle()),
        factory
            .generateCommandFromFile("LeftVolumeBackup", false)
            .alongWith(new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState)));
  }
}
