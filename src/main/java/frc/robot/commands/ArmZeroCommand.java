package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmZeroCommand extends CommandBase {
  private final ArmSubsystem arm;
  private int counter = 0;

  /**
   * Rezeros the elevator
   *
   * @param arm Arm subsystem
   */
  public ArmZeroCommand(ArmSubsystem arm) {
    this.arm = arm;
  }

  @Override
  public void execute() {
    if (arm.stalledAtZero()) counter++;
    else counter = 0;
  }

  public boolean isFinished() {
    return counter >= 25; // half second of stall required
  }

  public void end(boolean interupted) {
    if (interupted) {
      System.out.println("Rezero was interupted or timed out before it could complete!");
    } else {
      if (arm.zeroElevator()) {
        System.out.println("Rezero successful!");
      } else {
        System.out.println("Arm subsystem rejected rezero!");
      }
    }
  }
}
