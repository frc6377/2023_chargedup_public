package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndAffectorSubsystem;
import java.util.function.DoubleConsumer;

public class IntakeCommand extends CommandBase {
  private static final double INTAKE_WAIT = 0.2;
  private static final double INTAKE_RUMBLE_VELOCITY = 5;
  private static final double RUMBLE_INTENSTIY = 0.5;

  private final EndAffectorSubsystem subsystem;
  private final DoubleConsumer rumbleSetter;
  private Timer startIntakeTimer = new Timer();

  public IntakeCommand(EndAffectorSubsystem subsystem, DoubleConsumer rumble) {
    this.subsystem = subsystem;
    this.rumbleSetter = rumble;
  }

  @Override
  public void initialize() {
    startIntakeTimer.start();
    subsystem.intake();
  }

  @Override
  public void execute() {
    if (startIntakeTimer.hasElapsed(INTAKE_WAIT)) {
      if (Math.abs(subsystem.getVelocity()) < INTAKE_RUMBLE_VELOCITY) {
        rumbleSetter.accept(RUMBLE_INTENSTIY);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    rumbleSetter.accept(0);
    subsystem.idle();
  }
}
