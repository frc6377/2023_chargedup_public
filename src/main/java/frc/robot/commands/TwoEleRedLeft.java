package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TwoEleRedLeft extends SequentialCommandGroup {
  public TwoEleRedLeft(DrivetrainSubsystem drive, SwerveAutoFactory factory) {
    addCommands(
        factory.generateCommandFromFile("PickFirstElementBlue", true),
        factory.generateCommandFromFile("ScoreFirstElementBlue", false),
        factory.generateCommandFromFile("ClimbBlue", false),
        new AutoBalanceCommand(drive));
  }
}
