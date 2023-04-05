package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class MidOneAndAHalf extends SequentialCommandGroup {
    public MidOneAndAHalf(
        DrivetrainSubsystem drive,
        SwerveAutoFactory factory,
        ArmSubsystem arm,
        EndAffectorSubsystem endAffector) {
      addCommands(
        new InstantCommand(()-> endAffector.halt()),
        new ArmPowerCommand(Constants.BACKWARDS_HIGH_CONE, arm, 3),
        new WaitCommand(1),
        new InstantCommand(
            () -> {
              endAffector.setCone();
            }),
        new InstantCommand(
            () -> {
              endAffector.fastOutake();
            }),
          new WaitCommand(0.25),
          new InstantCommand(()-> endAffector.halt()),
          factory.generateCommandFromFile("MobilityMiddle", true, 1, 1).alongWith(new WaitCommand(0.5).andThen(new ArmPowerCommand(Constants.STOWED_ARM_POSITION, arm, 3)).andThen(new WaitCommand(2)).andThen(new ArmPowerCommand(Constants.LOW_CONE_ARM_POSITION, arm, 3).alongWith(new InstantCommand(()-> endAffector.intake())))),
          new InstantCommand(()-> endAffector.idle()),
          new ArmPowerCommand(Constants.STOWED_ARM_POSITION, arm, 3),
          factory.generateCommandFromFile("MobilityBalance", false),
          new AutoBalanceCommand(drive));
    }
  }
  

