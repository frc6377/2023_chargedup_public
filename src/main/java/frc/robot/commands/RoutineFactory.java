package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.EndAffectorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class RoutineFactory {
  private final LeftTwoElement leftTwoElement;
  private final LeftTwoElementNoClimb leftTwoElementNoClimb;
  private final MiddleClimb middleClimb;
  private final OnlyPreload preloadOnly;
  private final RightTwoElement rightTwoElement;
  private final RightTwoElementNoClimb rightTwoElementNoClimb;

  public RoutineFactory(
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector,
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory) {
    preloadOnly = new OnlyPreload(arm);
    leftTwoElement = new LeftTwoElement(drive, factory, arm, endAffector);
    leftTwoElementNoClimb = new LeftTwoElementNoClimb(drive, factory, arm, endAffector);
    middleClimb = new MiddleClimb(drive, factory, arm, endAffector);
    rightTwoElement = new RightTwoElement(drive, factory, arm, endAffector);
    rightTwoElementNoClimb = new RightTwoElementNoClimb(drive, factory, arm, endAffector);
  }

  public Command getAuto(Routines routine) {
    System.out.println("loading " + routine.name());
    switch (routine) {
      case LEFT_2_ELEMENT_CLIMB:
        return leftTwoElement;

      case LEFT_2_ELEMENT_NOCLIMB:
        return leftTwoElementNoClimb;

      case MIDDLE_CLIMB:
        return middleClimb;

      case NO_OP:
        return new InstantCommand();

      case PRELOAD_ONLY:
        return preloadOnly;

      case RIGHT_2_ELEMENT_CLIMB:
        return rightTwoElement;

      case RIGHT_2_ELEMENT_NOCLIMB:
        return rightTwoElementNoClimb;

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
