// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drivetrain.config.PodName;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int Degrees0 = 0;
  public static final int Degrees45 = 45;
  public static final int Degrees90 = 90;
  public static final int Degrees180 = 180;
  public static final int Degrees270 = 270;

  public static final int driverControllerID = 0;
  public static final int gunnerControllerID =
      -1; // This is -1 to disable it as it is not currently going to be used.
  public static final int streamDeckID = 1;
  // TODO: add real value for the motor ID
  public static final int endAffectorID = 9;

  public static final int TeamNumber = 6377;

  public static final double MetersToInches = 39.3701;
  /**
   * TalonFX number of ticks per 1 motor revolution
   *
   * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
   */
  public static final int TalonFXTicksPerRevolution = 2048;

  public static final int TalonFullPowerInternal = 1023;

  /**
   * Pigeon number of units per 1 full 360 degree yaw rotation
   *
   * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
   */
  public static final int PigeonUnitsPerRotation = 8192;

  /** Default timeout to use when sending messages over the can bus */
  public static final int CANTimeout = 50;

  /**
   * You can have up to 2 devices assigned remotely to a talon/victor HowdyBot conventions: -
   * Drivetrains: - use 0 for a remote left encoder - use 1 for a remote pigeon
   */
  public static final int RemoteEncoder = 0;

  public static final int RemotePigeon = 1;

  /**
   * The talon runs 2 pid loops, a primary and an aux pid loop. HowdyBot conventions: - Drivetrains:
   * - use primary for closed loop velocity or closed loop position - use aux for turning
   */
  public static final int PIDLoopPrimary = 0;

  public static final int PIDLoopTurn = 1;

  /** HowydBot convention: the aux pid is expressed in units of 1/10 of a degree */
  public static final int TurnPIDRange = 3600;

  /**
   * The talon can have up to 4 pid loops configured. HowdyBot conventions: - Drivetrains: - Slot0:
   * velocity (primary pid loop) - Slot1: turninig (on aux pid loop) - Slot2: position (primary pid
   * loop) - Slot3: motion profile mode (primary pid loop)
   */
  public static final int PIDSlotVelocity = 0;

  public static final int PIDSlotTurning = 1;
  public static final int PIDSlotPosition = 2;
  public static final int PIDSlotMotionProfile = 3;

  public static final double doubleEpsilon = 0.001;

  public static final double armKp = 5e-5 * 10;
  public static final double armKf = 0.000156 * 10;
  public static final int armMaxvelo = 2;
  public static final int armMaxAccel = 1;
  // added new functions from line 83-98 for rotational motor and extension motor.
  // TODO: convert arm rotation values to radians
  // TODO: decide on actual values for arm length, rotation, and wrist rotation values and IDs
  public static final double armRotationStowed = -5;
  public static final double armRotationLow = -3;
  public static final double armRotationCubeMid = -10.428622245788574 - 3;
  public static final double armRotationCubeHigh = -10.428622245788574 - 7;
  public static final double armRotationConeMid = -10.428622245788574 - 3;
  public static final double armRotationConeHigh = -10.428622245788574 - 7;
  public static final int armRotationCurrentLimit = 0;

  public static final int armRotationID1 = -1;
  public static final int armRotationID2 = -1;

  public static final double armRotationalTicksToRadians = Math.PI / 50;

  public static final double armLengthStowed = -5;
  public static final double armLengthLow = -3;
  public static final double armLengthCubeMid = -10.428622245788574 - 3;
  public static final double armLengthCubeHigh = -10.428622245788574 - 7;
  public static final double armLengthConeMid = -10.428622245788574 - 3;
  public static final double armLengthConeHigh = -10.428622245788574 - 7;
  public static final int armExtensionCurrentLimit = 0;

  public static final int armExtenderID = -1;

  public static final double wristRotationLow = -3;
  public static final double wristRotationStowed = -5;
  public static final double wristRotationCubeMid = -10.428622245788574 - 3;
  public static final double wristRotationCubeHigh = -10.428622245788574 - 7;
  public static final double wristRotationConeMid = -10.428622245788574 - 3;
  public static final double wristRotationConeHigh = -10.428622245788574 - 7;
  public static final int wristCurrentLimit = 0;

  public static final double wristTicksToRadians = Math.PI / 50; // TODO: Calculate the actual value

  public static final int wristID = -1;
  public static final double armLengthTicksToMeters = 0;
  public static final double armLengthAtZeroTicks = 0.7;
  // TODO: Get actual value(is the weight of the arm multiplied by the number needed to convert
  public static final double rotationArmGearRatio = 100;
  public static final double stalledTorque = 2.6;
  public static final double armWeight = 4.3;
  public static final double armAngleAtRest = Math.toRadians(9.3);

  public static final double endAffectorIntakespeed = 0.4;
  public static final double endAffectorOutakespeed = 0.3;
  public static final double endAffectorSlowOutakespeed = 0.075;
  public static final double endAffectorIdleSpeed = 0.1;

  // Arm Rotation
  public static final double armRotationKp = 0.099902;
  public static final double armRotationMaxVelo = 15;
  public static final double armRotationMaxAccel = 17.64705882;

  // Arm Extension
  public static final double armExtensionKp = 0;
  public static final double armExtensionMaxVelo = 0;
  public static final double armExtensionMaxAccel = 0;

  // Wrist
  public static final double wristKp = 0;
  public static final double wristMaxVelo = 0;
  public static final double wristMaxAccel = 0;
  public static final double wristMomentOfInertia = 0;
  public static final double wristGearRatio = 0;

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.62865;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.62865;

  public static final int DRIVETRAIN_PIGEON_ID = 1;

  public static final PodName FRONT_LEFT_POD_NAME = PodName.F;

  public static final PodName FRONT_RIGHT_POD_NAME = PodName.H;

  public static final PodName BACK_LEFT_POD_NAME = PodName.E;

  public static final PodName BACK_RIGHT_POD_NAME = PodName.G;

  public static final double autoMaxVelocity = 4.5;

  public static final double autoMaxAcceleration = 2.5;
}
