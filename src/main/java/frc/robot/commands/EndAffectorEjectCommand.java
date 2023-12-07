package frc.robot.commands;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.color.GamePieceMode;
import java.util.function.DoubleSupplier;

public class EndAffectorEjectCommand extends CommandBase {
  private final DoubleSupplier shootSupplier;
  private final EndAffectorSubsystem endAffector;
  private double target;
  private final IntegerSubscriber gamePieceModeSubscriber =
      NetworkTableInstance.getDefault().getIntegerTopic("GAME_PIECE_MODE").subscribe(10);
  private boolean enablePartial = false;
  ;
  private final IntegerSubscriber armHeightSubscriber =
      NetworkTableInstance.getDefault().getIntegerTopic("GAME_PIECE_MODE").subscribe(10);

  /**
   * Has the intake eject
   *
   * @param shootSupplier The strength that the intake should eject
   * @param endAffector End affector subsystem
   */
  public EndAffectorEjectCommand(DoubleSupplier shootSupplier, EndAffectorSubsystem endAffector) {
    this.shootSupplier = shootSupplier;
    this.endAffector = endAffector;
    addRequirements(endAffector);
  }

  @Override
  public void initialize() {
    target = endAffector.getIntakePosition() + Constants.END_AFFECTOR_OFFSET;

    // We don't partially eject unless we're placing high and mid cones
    enablePartial =
        (ArmHeight.getFromInt((int) armHeightSubscriber.get()) == ArmHeight.HIGH
                || ArmHeight.getFromInt((int) armHeightSubscriber.get()) == ArmHeight.MID)
            && (GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()).isCone());
    System.out.println(
        "Partial:"
            + enablePartial
            + " armHeight"
            + ArmHeight.getFromInt((int) armHeightSubscriber.get()));
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
