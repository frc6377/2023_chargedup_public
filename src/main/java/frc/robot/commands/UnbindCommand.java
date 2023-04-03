package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmSubsystem;

public class UnbindCommand extends SequentialCommandGroup {
    public UnbindCommand (ArmSubsystem arm){
        addCommands(new InstantCommand(()-> arm.setTarget(arm.getUnbindPosition())), new WaitCommand(0.35));
    }
}
