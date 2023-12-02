package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class AutoDeliveryCommand extends SequentialCommandGroup {
  private final SendableChooser<Integer> deliveryChooser = new SendableChooser<>();
  private final SwerveAutoFactory autoGenerator;

  public AutoDeliveryCommand(ArmSubsystem arm, DrivetrainSubsystem drive) {
    this.autoGenerator = new SwerveAutoFactory(null, drive);
    addOptions();
    addCommands(this.autoGenerator.generateGridCommand(deliveryChooser.getSelected()));
  }

  private void addOptions() {
    deliveryChooser.setDefaultOption("left 1", 8);
    deliveryChooser.addOption("left 2", 7);
    deliveryChooser.addOption("left 3", 6);
    deliveryChooser.addOption("mid 1", 5);
    deliveryChooser.addOption("mid 2", 4);
    deliveryChooser.addOption("mid 3", 3);
    deliveryChooser.addOption("right 1", 2);
    deliveryChooser.addOption("right 2", 1);
    deliveryChooser.addOption("right 3", 0);
  }
}
