package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ZeroElevator extends CommandBase {
  private Timer timer;
  private final ArmSubsystem subsystem;

  public ZeroElevator(ArmSubsystem subsystem) {
    this.timer = new Timer();
    this.subsystem = subsystem;
  }

  @Override
  public void initialize() {
    timer.start();
    subsystem.setElevatorPercent(Constants.ELEVATOR_ZEROING_PERCENT);
  }

  @Override
  public void execute() {
    if (subsystem.getElevatorCurrentDraw() < Constants.ELEVATOR_ZERO_AMPRAGE) {
      timer.reset();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(Constants.ELEVATOR_ZEROING_TIME_SECONDS);
  }

  @Override
  public void end(boolean interupt) {
    if (!interupt) {
      subsystem.setTheElevatorZero();
    }

    subsystem.stop();
  }
}
