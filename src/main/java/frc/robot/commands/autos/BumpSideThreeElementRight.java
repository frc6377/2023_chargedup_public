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

public class BumpSideThreeElementRight extends SequentialCommandGroup {
  public BumpSideThreeElementRight(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector,
      RobotStateManager robotState) {
    addCommands(
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(ArmPosition.BACKWARDS_HIGH_CONE_POSITION, arm, 3, robotState),
        new WaitCommand(1),
        new InstantCommand(
            () -> {
              robotState.setGamePieceMode(GamePieceMode.CUBE);
            }),
        new InstantCommand(
            () -> {
              endAffector.intake();
            }),
        new WaitCommand(0.25),
        new ArmPowerCommand(ArmPosition.HIGH_STOWED_ARM_POSITION, arm, 3, robotState)
            .andThen(new WaitCommand(0.5))
            .andThen(new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState))
            .alongWith(
                new WaitCommand(0.5)
                    .andThen(
                        factory
                            .generateCommandFromFile("PickFirstElementBumpsideRight", true, 4, 2.5)
                            .andThen(new WaitCommand(0.25)))),
        new InstantCommand(() -> endAffector.idle()),
        new ArmPowerCommand(ArmPosition.AUTON_SAFECHUCK_POSITION, arm, 3, robotState)
            .alongWith(factory.generateCommandFromFile("BumpSideRightBowlFirst", false, 4, 2.5))
            .alongWith(
                new WaitCommand(1.2).andThen(new InstantCommand(() -> endAffector.fastOutake()))),
        new WaitCommand(0.25),
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(ArmPosition.HYBRID_CUBE_ARM_POSITION, arm, 3, robotState)
            .andThen(
                new WaitCommand(0.0)
                    .andThen(
                        new ArmPowerCommand(ArmPosition.LOW_CUBE_ARM_POSITION, arm, 3, robotState)))
            .alongWith(
                new WaitCommand(0.0)
                    .andThen(
                        factory.generateCommandFromFile("BumpSideRightPickSecond", false, 3, 2.5)))
            .alongWith(
                new WaitCommand(0.25).andThen(new InstantCommand(() -> endAffector.intake()))),
        new ArmPowerCommand(ArmPosition.HIGH_STOWED_ARM_POSITION, arm, 3, robotState)
            .andThen(
                new WaitCommand(0.75)
                    .andThen(new InstantCommand(() -> endAffector.idle()))
                    .andThen(
                        new ArmPowerCommand(ArmPosition.HIGH_CUBE_ARM_POSITION, arm, 3, robotState))
                    .andThen(new InstantCommand(() -> endAffector.fastOutake())))
            .alongWith(factory.generateCommandFromFile("BumpSideRightScoreSecond", false, 3, 3)),
        new ArmPowerCommand(ArmPosition.HIGH_STOWED_ARM_POSITION, arm, 3, robotState));
  }
}
