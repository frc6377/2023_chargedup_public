package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.Constants;

public class EndAffectorEjectCommand extends CommandBase {
    private DoubleSupplier shootSupplier;
    private EndAffectorSubsystem endAffector;
    private final double target;

    public EndAffectorEjectCommand(DoubleSupplier shootSupplier, EndAffectorSubsystem endAffector) {
        this.shootSupplier = shootSupplier;
        this.endAffector = endAffector;
        this.target = endAffector.getIntakePosition() + Constants.END_AFFECTOR_OFFSET;
        addRequirements(endAffector);
    }

    @Override
    public void execute() {
        if(shootSupplier.getAsDouble() > 0.3){
            if (shootSupplier.getAsDouble() > 0.5) {
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
