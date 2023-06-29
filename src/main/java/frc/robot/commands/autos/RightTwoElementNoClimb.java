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

public class RightTwoElementNoClimb extends SequentialCommandGroup {
  public RightTwoElementNoClimb(
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
        new ArmPowerCommand(Constants.LOW_CUBE_ARM_POSITION, arm, 3)
            .alongWith(
                new WaitCommand(0.5)
                    .andThen(
                        factory
                            .generateCommandFromFile("PickFirstElementRight", true, 3, 3)
                            .andThen(new InstantCommand(() -> endAffector.idle())))),
        new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3)
            .andThen(
                new WaitCommand(1)
                    .andThen(new ArmPowerCommand(Constants.HIGH_CUBE_ARM_POSITION, arm, 3)))
            .alongWith(factory.generateCommandFromFile("ScoreFirstElementRight", false, 3, 3)),
        new WaitCommand(0.75),
        new InstantCommand(() -> endAffector.fastOutake()),
        new WaitCommand(0.5),
        new InstantCommand(() -> endAffector.idle()),
        new ArmPowerCommand(Constants.HIGH_STOWED_ARM_POSITION, arm, 3));
  }
}