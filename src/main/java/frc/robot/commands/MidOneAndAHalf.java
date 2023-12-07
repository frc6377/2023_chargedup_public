package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotStateManager;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class MidOneAndAHalf extends SequentialCommandGroup {
  public MidOneAndAHalf(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector,
      RobotStateManager robotState) {
    addCommands(
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(ArmPosition.BACKWARDS_MID_CONE_POSITION, arm, 3, robotState),
        new WaitCommand(1),
        /*new InstantCommand(
        () -> {
          endAffector.setCone();
        }),*/
        new InstantCommand(
            () -> {
              endAffector.fastOutake();
            }),
        new WaitCommand(0.25),
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(ArmPosition.HYBRID_CUBE_ARM_POSITION, arm, 3, robotState)
            .andThen(
                new WaitCommand(2)
                    .andThen(
                        new ArmPowerCommand(
                                ArmPosition.HYBRID_CUBE_ARM_POSITION, arm, 3, robotState)
                            .alongWith(new InstantCommand(() -> endAffector.intake()))))
            .alongWith(
                new WaitCommand(0.5)
                    .andThen(factory.generateCommandFromFile("MobilityMiddle", true, 1, 1))),
        new InstantCommand(() -> endAffector.idle()),
        new ArmPowerCommand(ArmPosition.AUTO_STOWED_ARM_POSITION, arm, 3, robotState)
            .alongWith(factory.generateCommandFromFile("MobilityBalance", false)),
        new AutoBalanceCommand(drive));
  }
}
