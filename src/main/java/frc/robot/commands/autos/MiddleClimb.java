package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotStateManager;
import frc.robot.commands.ArmPowerCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.SwerveAutoFactory;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class MiddleClimb extends SequentialCommandGroup {
  public MiddleClimb(
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector,
      RobotStateManager robotState) {
    addCommands(
        new InstantCommand(() -> endAffector.halt()),
        new ArmPowerCommand(ArmPosition.HIGH_STOWED_ARM_POSITION, arm, 3, robotState),
        new ArmPowerCommand(ArmPosition.HYBRID_CUBE_ARM_POSITION, arm, 3, robotState),
        new WaitCommand(0.75),
        factory.generateCommandFromFile("ClimbMiddle", true),
        new AutoBalanceCommand(drive));
  }
}
