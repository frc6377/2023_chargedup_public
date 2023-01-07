package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {

  public static Command Noop() {
    return Commands.print("No autonomous command configured");
  }
}
