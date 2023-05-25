package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.OmegaBalance;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TwoStepAutoBalance extends SequentialCommandGroup {
  public TwoStepAutoBalance(
      double settlingTime, double driveVelocity, double targetOmega, DrivetrainSubsystem drive) {
    addCommands(
        new OmegaBalance(drive, driveVelocity, targetOmega),
        new WaitCommand(settlingTime),
        new AutoBalanceCommand(drive));
  }
}
