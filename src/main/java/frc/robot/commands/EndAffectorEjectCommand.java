package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmHeight;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class EndAffectorEjectCommand extends CommandBase {
  private final DoubleSupplier shootSupplier;
  private final EndAffectorSubsystem endAffector;
  private double target;
  private final Supplier<ArmHeight> armHeightSupplier;
  private boolean enablePartial = false;
  ;
  private final BooleanSupplier isCube;

  public EndAffectorEjectCommand(
      DoubleSupplier shootSupplier,
      EndAffectorSubsystem endAffector,
      Supplier<ArmHeight> armHeightSupplier,
      BooleanSupplier isCube) {
    this.shootSupplier = shootSupplier;
    this.endAffector = endAffector;
    this.armHeightSupplier = armHeightSupplier;
    this.isCube = isCube;
    addRequirements(endAffector);
  }

  @Override
  public void initialize() {
    target = endAffector.getIntakePosition() + Constants.END_AFFECTOR_OFFSET;
    enablePartial = armHeightSupplier.get() == ArmHeight.HIGH && !isCube.getAsBoolean();
  }

  @Override
  public void execute() {
    if (shootSupplier.getAsDouble() > 0.2) {
      if (enablePartial && shootSupplier.getAsDouble() < 0.95) {
        endAffector.partialEject(target);
      } else {
        endAffector.fastOutake();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    endAffector.halt();
  }
}
