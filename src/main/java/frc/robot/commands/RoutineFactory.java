package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class RoutineFactory {
  private final ArmSubsystem arm;
  private final EndAffectorSubsystem endAffector;
  private final DrivetrainSubsystem drive;
  private final SwerveAutoFactory factory;

  public RoutineFactory(
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector,
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory) {
    this.arm = arm;
    this.endAffector = endAffector;
    this.drive = drive;
    this.factory = factory;
  }

  public Command getAuto(Routines routine) {
    switch (routine) {
      case LEFT_2_ELEMENT_CLIMB:
        System.out.println("loading " + routine.name());
        return new LeftTwoElement(drive, factory, arm, endAffector);

      case LEFT_2_ELEMENT_NOCLIMB:
        return new LeftTwoElementNoClimb(drive, factory, arm, endAffector);

      case MIDDLE_CLIMB:
        return new MiddleClimb(drive, factory, arm, endAffector);

      case NO_OP:
        return new InstantCommand();

      case PRELOAD_ONLY:
        return new OnlyPreload(arm);

      case RIGHT_2_ELEMENT_CLIMB:
        return new RightTwoElement(drive, factory, arm, endAffector);

      case RIGHT_2_ELEMENT_NOCLIMB:
        return new RightTwoElementNoClimb(drive, factory, arm, endAffector);

      default:
        System.out.println("NO VALID AUTO FOUND! RUNNING NOTHING");
        return new InstantCommand();
    }
  }

  public enum Routines {
    NO_OP,
    MIDDLE_CLIMB,
    PRELOAD_ONLY,
    LEFT_2_ELEMENT_CLIMB,
    RIGHT_2_ELEMENT_CLIMB,
    LEFT_2_ELEMENT_NOCLIMB,
    RIGHT_2_ELEMENT_NOCLIMB
  }
}
