package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class MiddleClimb extends SequentialCommandGroup {
  public MiddleClimb(
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
        new WaitCommand(0.75),
        new InstantCommand(() -> endAffector.halt()),
        
        new ArmPowerCommand(Constants.STOWED_ARM_POSITION, arm, 3).alongWith(new WaitCommand(0.5).andThen(factory
        .generateCommandFromFile("ClimbMiddle", true))),
        new AutoBalanceCommand(drive));
  }
}
