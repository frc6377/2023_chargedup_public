package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Normal CommandXboxController doesn't support rumble, this class adds it

public class HowdyXboxController extends CommandXboxController {
  private final XboxController rumbleXboxController;

  public HowdyXboxController(int port) {
    super(port);
    this.rumbleXboxController = new XboxController(port);
  }

  public void setRumble(double rumbleIntensity) {
    rumbleXboxController.setRumble(RumbleType.kBothRumble, rumbleIntensity);
  }
}
