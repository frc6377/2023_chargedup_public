package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class LeftVolumeAuto extends SequentialCommandGroup {
    public LeftVolumeAuto (DrivetrainSubsystem drive, SwerveAutoFactory factory){
        addCommands(
            factory.generateCommandFromFile("LeftVolumeFirstElement", true),
            factory.generateCommandFromFile("LeftVolumeScoreFirst", false),
            factory.generateCommandFromFile("LeftVolumeSecondElement", false),
            factory.generateCommandFromFile("LeftVolumeScoreSecond", false),
            factory.generateCommandFromFile("LeftVolumeThirdElement", false),
            factory.generateCommandFromFile("LeftVolumeScoreThird", false),
            factory.generateCommandFromFile("LeftVolumeBackup", false)


        );
    }
}
