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

public class LeftThreeElement extends SequentialCommandGroup {
  public LeftThreeElement(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector,
      RobotStateManager robotState) {
    addCommands(
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(ArmPosition.BACKWARDS_MID_CONE_POSITION, arm, 3, robotState),
        new WaitCommand(0.25),
        new InstantCommand(
            () -> {
              robotState.setGamePieceMode(GamePieceMode.CUBE);
            }),
        new InstantCommand(
            () -> {
              endAffector.intake();
            }),
        new WaitCommand(0.25),
        new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState)
            .alongWith(
                new WaitCommand(0.5)
                    .andThen(
                        factory
                            .generateCommandFromFile("PickFirstElementBlue", true, 4, 3)
                            .andThen(new InstantCommand(() -> endAffector.idle())))),
        new ArmPowerCommand(ArmPosition.HIGH_STOWED_ARM_POSITION, arm, 3, robotState)
            .andThen(
                new WaitCommand(0.5)
                    .andThen(
                        new ArmPowerCommand(ArmPosition.MID_CUBE_ARM_POSITION, arm, 3, robotState)))
            .alongWith(factory.generateCommandFromFile("ScoreFirstElementBlue", false, 4, 2.9)),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.25),
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(ArmPosition.HIGH_STOWED_ARM_POSITION, arm, 3, robotState)
            .andThen(
                new WaitCommand(0.0)
                    .andThen(
                        new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState)))
            .alongWith(
                new WaitCommand(0.0)
                    .andThen(
                        factory.generateCommandFromFile(
                            "PickSecondElementBlue", true, 3, 2.5))) // yes this should be true
            .alongWith(
                new WaitCommand(0.25).andThen(new InstantCommand(() -> endAffector.intake()))),
        new ArmPowerCommand(ArmPosition.HIGH_STOWED_ARM_POSITION, arm, 3, robotState)
            .andThen(
                new WaitCommand(0.25)
                    .andThen(new InstantCommand(() -> endAffector.idle()))
                    .andThen(
                        new ArmPowerCommand(
                            ArmPosition.HYBRID_CUBE_ARM_POSITION, arm, 3, robotState))
                    .andThen(new InstantCommand(() -> endAffector.fastOutake())))
            .alongWith(factory.generateCommandFromFile("ScoreSecondElementBlue", false, 3, 3)));
  }
}
