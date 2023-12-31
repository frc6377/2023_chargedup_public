package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotStateManager;
import frc.robot.commands.autos.BumpSideThreeElementLeft;
import frc.robot.commands.autos.BumpSideThreeElementRight;
import frc.robot.commands.autos.LeftThreeElement;
import frc.robot.commands.autos.LeftTwoElement;
import frc.robot.commands.autos.LeftTwoElementNoClimb;
import frc.robot.commands.autos.LeftVolumeAuto;
import frc.robot.commands.autos.MiddleClimb;
import frc.robot.commands.autos.RightThreeElement;
import frc.robot.commands.autos.RightTwoElement;
import frc.robot.commands.autos.RightTwoElementNoClimb;
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
  private final LeftVolumeAuto leftVolume;
  private final RightThreeElement rightThreeElement;
  private final LeftThreeElement leftThreeElement;
  private final BumpSideThreeElementLeft leftBumpsideThreeElement;
  private final MidOneAndAHalf midOnePlusGrab;
  private final BumpSideThreeElementRight rightBumpsideThreeElement;

  public RoutineFactory(
      ArmSubsystem arm,
      EndAffectorSubsystem endAffector,
      DrivetrainSubsystem drive,
      SwerveAutoFactory factory,
      RobotStateManager robotState) {
    preloadOnly = new OnlyPreload(arm, robotState);
    leftTwoElement = new LeftTwoElement(drive, factory, arm, endAffector, robotState);
    leftTwoElementNoClimb = new LeftTwoElementNoClimb(drive, factory, arm, endAffector, robotState);
    middleClimb = new MiddleClimb(drive, factory, arm, endAffector, robotState);
    rightTwoElement = new RightTwoElement(drive, factory, arm, endAffector, robotState);
    rightTwoElementNoClimb =
        new RightTwoElementNoClimb(drive, factory, arm, endAffector, robotState);
    leftVolume = new LeftVolumeAuto(drive, factory, endAffector, arm, robotState);
    rightThreeElement = new RightThreeElement(drive, factory, arm, endAffector, robotState);
    leftThreeElement = new LeftThreeElement(drive, factory, arm, endAffector, robotState);
    leftBumpsideThreeElement =
        new BumpSideThreeElementLeft(drive, factory, arm, endAffector, robotState);
    midOnePlusGrab = new MidOneAndAHalf(drive, factory, arm, endAffector, robotState);
    rightBumpsideThreeElement =
        new BumpSideThreeElementRight(drive, factory, arm, endAffector, robotState);
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

      case RIGHT_3_ELEMENT_NOCLIMB:
        return rightThreeElement;

      case LEFT_VOLUME:
        return leftVolume;

      case LEFT_3_ELEMENT_NOCLIMB:
        return leftThreeElement;

      case LEFT_BUMPSIDE_3_ELEMENT:
        return leftBumpsideThreeElement;

      case MID_1_AND_GRAB:
        return midOnePlusGrab;

      case RIGHT_BUMPSIDE_3_ELEMENT:
        return rightBumpsideThreeElement;

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
    RIGHT_2_ELEMENT_NOCLIMB,
    LEFT_VOLUME,
    RIGHT_3_ELEMENT_NOCLIMB,
    LEFT_3_ELEMENT_NOCLIMB,
    LEFT_BUMPSIDE_3_ELEMENT,
    MID_1_AND_GRAB,
    RIGHT_BUMPSIDE_3_ELEMENT
  }
}
