package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class LeftThreeElement extends SequentialCommandGroup {
  public LeftThreeElement(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector) {
    addCommands(
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(Constants.BACKWARDS_MID_CONE, arm, 3),
        new WaitCommand(0.25),
        new InstantCommand(
            () -> {
              endAffector.setCube();
            }),
        new InstantCommand(
            () -> {
              endAffector.intake();
            }),
        new WaitCommand(0.25),
        new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3)
            .alongWith(
                new WaitCommand(0.5)
                    .andThen(
                        factory
                            .generateCommandFromFile("PickFirstElementBlue", true, 4, 3)
                            .andThen(new InstantCommand(() -> endAffector.idle())))),
        new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3)
            .andThen(
                new WaitCommand(0.5)
                    .andThen(new ArmPowerCommand(Constants.HIGH_CUBE_ARM_POSITION, arm, 3)))
            .alongWith(factory.generateCommandFromFile("ScoreFirstElementBlue", false, 4, 2.9)),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.25),
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3).andThen(new WaitCommand(0.0)
        .andThen(new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3)))
            .alongWith(new WaitCommand(0.0)
                .andThen(factory.generateCommandFromFile("PickSecondElementBlue", true, 3, 2.5))) //yes this should be true
            
            .alongWith(new WaitCommand(0.25)
                .andThen(new InstantCommand(()-> endAffector.intake()))),
        new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3)
            .andThen(new WaitCommand(0.25).andThen(new InstantCommand(()-> endAffector.idle()))
                .andThen(new ArmPowerCommand(Constants.HYBRID_CUBE_ARM_POSITION, arm, 3)).andThen(new InstantCommand(()-> endAffector.fastOutake())))
            .alongWith(factory.generateCommandFromFile("ScoreSecondElementBlue", false, 3, 3)));
  }
}
