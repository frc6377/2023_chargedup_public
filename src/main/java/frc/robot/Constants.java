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
  public static final int DEGREES_0 = 0;
  public static final int DEGREES_45 = 45;
  public static final int DEGREES_90 = 90;
  public static final int DEGREES_180 = 180;
  public static final int DEGREES_270 = 270;

  public static final int DRIVER_CONTROLLER_ID = 0;
  // This is -1 to disable it as it is not currently going to be used.
  public static final int GUNNER_CONTROLLER_ID = 2;
  public static final int STEAM_DECK_ID = 1;
  // TODO: add real value for the motor ID
  public static final int END_AFFECTOR_ID_1 = 9;
  public static final int END_AFFECTOR_ID_2 = 10;

  public static final int TEAM_NUMBER = 6377;

  public static final double METERS_TO_INCHES = 39.3701;
  /**
   * TalonFX number of ticks per 1 motor revolution
   *
   * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
   */
  public static final int TALON_FX_TICKS_PER_REVOLUTION = 2048;

  public static final int TALON_FULL_POWER_INTERVAL = 1023;

  /**
   * Pigeon number of units per 1 full 360 degree yaw rotation
   *
   * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
   */
  public static final int PIGEON_UNITS_PER_ROTATION = 8192;

  /** Default timeout to use when sending messages over the can bus */
  public static final int CAN_TIMEOUT = 50;

  /**
   * You can have up to 2 devices assigned remotely to a talon/victor HowdyBot conventions: -
   * Drivetrains: - use 0 for a remote left encoder - use 1 for a remote pigeon
   */
  public static final int REMOTE_ENCODER = 0;

  public static final int REMOTE_PIGEON = 1;

  /**
   * The talon runs 2 pid loops, a primary and an aux pid loop. HowdyBot conventions: - Drivetrains:
   * - use primary for closed loop velocity or closed loop position - use aux for turning
   */
  public static final int PID_LOOP_PRIMARY = 0;

  public static final int PID_LOOP_TURN = 1;

  /** HowydBot convention: the aux pid is expressed in units of 1/10 of a degree */
  public static final int TURN_PID_RANGE = 3600;

  /**
   * The talon can have up to 4 pid loops configured. HowdyBot conventions: - Drivetrains: - Slot0:
   * velocity (primary pid loop) - Slot1: turninig (on aux pid loop) - Slot2: position (primary pid
   * loop) - Slot3: motion profile mode (primary pid loop)
   */
  public static final int PID_SLOT_VELOCITY = 0;

  public static final int PID_SLOT_TURNING = 1;
  public static final int PID_SLOT_POSITION = 2;
  public static final int PID_SLOT_MOTION_PROFILE = 3;

  public static final double DOUBLE_EPSILON = 0.001;

  public static final double ARM_KP = 5e-5 * 10;
  public static final double ARM_KF = 0.000156 * 10;
  public static final int ARM_MAX_VELO = 2;
  public static final int ARM_MAX_ACCEL = 1;
  //TODO: rename to be in all caps+full names(also the extender and wrist)
  // added new functions from line 83-98 for rotational motor and extension motor.
  // TODO: convert arm rotation values to radians
  // TODO: decide on actual values for arm length, rotation, and wrist rotation values and IDs
  public static final int ARM_ROTATION_CURRENT_LIMIT = 40;

  public static final int ARM_ROTATION_ID_1 = 11;
  public static final int ARM_ROTATION_ID2 = 12;

  public static final double ARM_ROTATION_TICKS_TO_RADIANS = Math.PI / 50;

  public static final int ARM_EXTENSION_CURRENT_LIMIT = 40;

  public static final int ARM_EXTENDER_ID = 13;
  
  public static final double ARM_EXTENSIONS_TICKS_TO_METERS = 0;
  public static final double ARM_LENGTH_AT_ZERO_TICKS = 0.7;

  public static final int WRIST_CURRENT_LIMIT = 30;
  public static final int WRIST_STATOR_LIMIT = 40;

  public static final double WRIST_TICKS_TO_RADIANS = Math.PI / 50; // TODO: Calculate the actual value

  public static final int WRIST_ID = 14;
  // TODO: Get actual value(is the weight of the arm multiplied by the number needed to convert
  public static final double ROTATION_ARM_GEAR_RATIO = 100;
  public static final double STALLED_TORQUE = 2.6;
  public static final double ARM_WEIGHT = 0;
  public static final double ARM_ANGLE_AT_REST = Math.toRadians(9.3);

  public static final double END_AFFECTOR_INTAKE_SPEED = 0.4;
  public static final double END_AFFECTOR_OUTTAKE_SPEED = 0.3;
  public static final double END_AFFECTOR_SLOW_OUTTAKE_SPEED = 0.075;
  public static final double END_AFFECTOR_IDLE_SPEED = 0.1;

  // Arm Rotation
  public static final double ARM_ROTATION_KP = 0.99902;
  public static final double ARM_ROTATION_MAX_VELOCITY = 15;
  public static final double ARM_ROTATION_MAX_ACCELLERATION = 17.64705882;

  // Arm Extension
  public static final double ARM_EXTENSION_KP = 0.99902;
  public static final double ARM_EXTENSION_MAX_VELOCITY = 15;
  public static final double ARM_EXTENSION_MAX_ACCELLERATION = 17.64705882;

  // Wrist
  public static final double WRIST_KP = 0.1;
  public static final double WRIST_MAX_VELOCITY = 5000;
  public static final double WRIST_MAX_ACCELLERATION = 5000;
  public static final double WRIST_MOMENT_OF_INERTIA = 0;
  public static final double WRIST_GEAR_RATIO = 0;
  public static final double WRIST_ROTATION_STOWED = 0;

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

  public static final double AUTO_MAX_VELOCITY = 4.5;

  public static final double AUTO_MAX_ACCELERATION = 2.5;
}
