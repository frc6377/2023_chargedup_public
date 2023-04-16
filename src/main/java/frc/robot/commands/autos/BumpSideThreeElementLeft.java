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

public class BumpSideThreeElementLeft extends SequentialCommandGroup {
  public BumpSideThreeElementLeft(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector) {
    addCommands(
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(Constants.BACKWARDS_HIGH_CONE, arm, 3),
        new WaitCommand(1),
        new InstantCommand(
            () -> {
              endAffector.setCube();
            }),
        new InstantCommand(
            () -> {
              endAffector.intake();
            }),
        new WaitCommand(0.25),
        new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3).andThen(new WaitCommand(0.5)).andThen(new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3))
            .alongWith(
                new WaitCommand(0.5)
                    .andThen(
                        factory
                            .generateCommandFromFile("PickFirstElementBumpsideBlue", true, 4, 2.5)
                            .andThen(new WaitCommand(0.25)))),
                            new InstantCommand(() -> endAffector.idle()),
        new ArmPowerCommand(Constants.AUTON_SAFECHUCK, arm, 3)
            .alongWith(factory.generateCommandFromFile("BumpSideBlueBowlFirst", false, 4, 2.5)).alongWith(new WaitCommand(1.5).andThen(new InstantCommand(() -> endAffector.fastOutake()))),
        
        new WaitCommand(0.25),
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(Constants.HYBRID_CUBE_ARM_POSITION, arm, 3).andThen(new WaitCommand(0.0)
        .andThen(new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3)))
            .alongWith(new WaitCommand(0.0)
                .andThen(factory.generateCommandFromFile("BumpSideBluePickSecond", false, 3, 2.5)))
            
            .alongWith(new WaitCommand(0.25)
                .andThen(new InstantCommand(()-> endAffector.intake()))),
        new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3)
            .andThen(new WaitCommand(1).andThen(new InstantCommand(()-> endAffector.idle()))
                .andThen(new ArmPowerCommand(Constants.HIGH_CUBE_ARM_POSITION, arm, 3)).andThen(new InstantCommand(()-> endAffector.fastOutake())))
            .alongWith(factory.generateCommandFromFile("BumpSideBlueScoreSecond", false, 3, 3)),
            new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3)
            );

  }
}
