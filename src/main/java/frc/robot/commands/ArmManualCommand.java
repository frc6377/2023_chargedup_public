package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmManualCommand extends CommandBase {
  public final DoubleSupplier wristDoubleSupplier;
  public final DoubleSupplier armLengthDoubleSupplier;
  public final ArmSubsystem armSubsystem;

  public ArmManualCommand(
      DoubleSupplier wristDoubleSupplier,
      DoubleSupplier armLengthDoubleSupplier,
      ArmSubsystem armSubsystem) {
    this.wristDoubleSupplier = wristDoubleSupplier;
    this.armLengthDoubleSupplier = armLengthDoubleSupplier;
    this.armSubsystem = armSubsystem;
  }

  public void execute() {
    armSubsystem.setTarget(
        armSubsystem
            .getArmGoalPosition()
            .add(
                new ArmPosition(
                    0,
                    armLengthDoubleSupplier.getAsDouble()
                        * Constants.ARM_LENGTH_MANUAL_OVERRIDE_SPEED,
                    wristDoubleSupplier.getAsDouble() * Constants.WRIST_MANUAL_OVERRIDE_SPEED,
                    ArmHeight.NOT_SPECIFIED)));
  }
}
