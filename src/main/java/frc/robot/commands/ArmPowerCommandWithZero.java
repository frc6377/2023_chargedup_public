package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotStateManager;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmPowerCommandWithZero extends SequentialCommandGroup {
  /**
   * Runs an ArmPowerCommand, but will re-zero the elevator in the process.
   * @param armPos The armPosition to go to
   * @param arm Arm subsystem
   * @param pow Power with which to run
   * @param robotState Robot state
   */
  public ArmPowerCommandWithZero(
      ArmPosition armPos, ArmSubsystem arm, double pow, RobotStateManager robotState) {
    armPos =
        new ArmPosition(
            armPos.getArmRotationRadians(),
            0,
            armPos.getWristRotation(),
            armPos.getHeight()); // ensures extension is 0
    addCommands(
        new ArmPowerCommand(armPos, arm, pow, robotState),
        new ArmZeroCommand(arm).withTimeout(1.5)); // 1.5 secs to find 0
  }
  /**
   * Runs an ArmPowerCommand, but will re-zero the elevator in the process.
   * @param armHeight The armHeight to go to
   * @param arm Arm subsystem
   * @param pow Power with which to run
   * @param robotState Robot state
   */
  public ArmPowerCommandWithZero(
      ArmHeight armHeight, ArmSubsystem arm, double pow, RobotStateManager robotState) {
    addCommands(
        new ArmPowerCommand(armHeight, arm, pow, robotState),
        new ArmZeroCommand(arm).withTimeout(1.5)); // 1.5 secs to find 0
  }
}
