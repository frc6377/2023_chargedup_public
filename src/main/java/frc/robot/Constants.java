// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.config.PodConfig;
import java.util.Arrays;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String V1_MAC_ADDRESS = "00:80:2F:33:9E:07";
  public static final String V2_MAC_ADDRESS = "00:80:2F:35:D5:2C";

  private static final double FRONT_LEFT_V1_CANCODER_OFFSET = 194.590;
  private static final double FRONT_RIGHT_V1_CANCODER_OFFSET = 63.896 + 180;
  private static final double BACK_LEFT_V1_CANCODER_OFFSET = 214.805;
  private static final double BACK_RIGHT_V1_CANCODER_OFFSET = 108.809 + 180;

  private static final double FRONT_LEFT_V2_CANCODER_OFFSET  = 139.570;
  private static final double FRONT_RIGHT_V2_CANCODER_OFFSET = 286.436;
  private static final double BACK_LEFT_V2_CANCODER_OFFSET   = 107.666;
  private static final double BACK_RIGHT_V2_CANCODER_OFFSET  = 233.438;
  
  public static final int DEGREES_0 = 0;
  public static final int DEGREES_45 = 45;
  public static final int DEGREES_90 = 90;
  public static final int DEGREES_180 = 180;
  public static final int DEGREES_270 = 270;

  public static final int DRIVER_CONTROLLER_ID = 0;
  public static final int GUNNER_CONTROLLER_ID = 1;
  public static final int STEAM_DECK_ID = 2;
  public static final double ARM_MANUAL_OVERRIDE_DEADZONE = 0.1;
  public static final double ARM_LENGTH_MANUAL_OVERRIDE_SPEED = -0.2;
  public static final double WRIST_MANUAL_OVERRIDE_SPEED = -100;
  // TODO: add real value for the motor ID
  public static final int END_AFFECTOR_ID = 9;

  public static final int GAME_PIECE_CANDLE = 2;
  public static final int GRID_SELECT_CANDLE = 3;

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
  // TODO: rename to be in all caps+full names(also the extender and wrist)
  // added new functions from line 83-98 for rotational motor and extension motor.
  // TODO: convert arm rotation values to radians
  public static final int ARM_ROTATION_CURRENT_LIMIT = 40;

  public static final int LEFT_SHOULDER_ID = 13;
  public static final int RIGHT_SHOULDER_ID = 11;

  public static final double ARM_ROTATION_TICKS_TO_RADIANS =
      Math.PI * 2 / 90; // The arm is geared 90:1

  public static final int ARM_EXTENSION_CURRENT_LIMIT = 40;
  public static final int ARM_LENGTH_OFFSET_JOYSTICK_MULTIPLIER = 100;
  public static final int WRIST_OFFSET_JOYSTICK_MULTIPLIER = 100;

  public static final int ARM_EXTENDER_ID = 12;

  // This is the Circumference of the pully
  public static final double ARM_EXTENSIONS_TICKS_TO_METERS = 0.0254 * Math.PI;
  public static final double ARM_LENGTH_AT_ZERO_TICKS_METERS = 0.7;

  public static final int WRIST_CURRENT_LIMIT = 30;
  public static final int WRIST_STATOR_LIMIT = 40;

  public static final double WRIST_TICKS_TO_RADIANS =
      Math.PI / 50; // TODO: Calculate the actual value

  public static final int WRIST_ID = 14;

  // TODO: Get actual value(is the weight of the arm multiplied by the number needed to convert
  public static final double ROTATION_ARM_GEAR_RATIO = 100;
  public static final double STALLED_TORQUE = 2.6;
  public static final double ARM_WEIGHT_KG = 5.4;
  public static final double ARM_ANGLE_AT_REST = Math.toRadians(9.3);

  public static final double END_AFFECTOR_INTAKE_SPEED = 1;
  public static final double END_AFFECTOR_OUTTAKE_SPEED = 0.3;
  public static final double END_AFFECTOR_SLOW_OUTTAKE_SPEED = 0.075;
  public static final double END_AFFECTOR_IDLE_SPEED = 0.1;

  public static final double END_AFFECTOR_KP = 0.2;
  public static final double END_AFFECTOR_OFFSET = -2250;

  // Arm Rotation
  public static final double ARM_ROTATION_KP = 0.99902;
  public static final double ARM_ROTATION_MAX_VELOCITY = 15;
  public static final double ARM_ROTATION_MAX_ACCELLERATION = 17.64705882;

  // Arm Extension
  public static final double ARM_EXTENSION_KP = 0.6;
  public static final double ARM_EXTENSION_MAX_VELOCITY = 15 * 0.25;
  public static final double ARM_EXTENSION_MAX_ACCELLERATION = 17.64705882 * 0.25;

  // Wrist
  public static final double WRIST_KP = 0.1;
  public static final double WRIST_MAX_VELOCITY = 3 * 5000;
  public static final double WRIST_MAX_ACCELLERATION = 3 * 5000;
  public static final double WRIST_MOMENT_OF_INERTIA = 0;
  public static final double WRIST_GEAR_RATIO = 1.0 / 41.67;
  public static final double WRIST_ROTATION_STOWED = 0;

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.457;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.457;

  public static final int DRIVETRAIN_PIGEON_ID = 1;

  public static final PodConfig FRONT_LEFT_V1_CONFIG =
      new PodConfig(3, 4, 4, -Math.toRadians(FRONT_LEFT_V1_CANCODER_OFFSET), "front left v1");
  public static final PodConfig FRONT_RIGHT_V1_CONFIG =
      new PodConfig(5, 6, 6, -Math.toRadians(FRONT_RIGHT_V1_CANCODER_OFFSET), "front right v1");
  public static final PodConfig BACK_LEFT_V1_CONFIG =
      new PodConfig(1, 2, 2, -Math.toRadians(BACK_LEFT_V1_CANCODER_OFFSET), "back left v1");
  public static final PodConfig BACK_RIGHT_V1_CONFIG =
      new PodConfig(7, 8, 8, -Math.toRadians(BACK_RIGHT_V1_CANCODER_OFFSET), "back right v1");

  public static final PodConfig FRONT_LEFT_V2_CONFIG =
      new PodConfig(3, 4, 4, -Math.toRadians(FRONT_LEFT_V2_CANCODER_OFFSET), "front left v2");
  public static final PodConfig FRONT_RIGHT_V2_CONFIG =
      new PodConfig(5, 6, 6, -Math.toRadians(FRONT_RIGHT_V2_CANCODER_OFFSET), "front right v2");
  public static final PodConfig BACK_LEFT_V2_CONFIG =
      new PodConfig(1, 2, 2, -Math.toRadians(BACK_LEFT_V2_CANCODER_OFFSET), "front left v2");
  public static final PodConfig BACK_RIGHT_V2_CONFIG =
      new PodConfig(7, 8, 8, -Math.toRadians(BACK_RIGHT_V2_CANCODER_OFFSET), "front right v2");

  public static final double FIELD_X = 16.54;
  public static final double FIELD_Y = 8.02;

  // otf pathing constants
  public static final double AUTO_MAX_VELOCITY = 2.0;
  public static final double AUTO_MAX_ACCELERATION = 2.0;

  // relative distance from your alliance station wall or the left of the field depending on axis.
  // Gets converted into absolute coordinates in FieldPoses.java
  public static final double CLOSE_PROXIMITY_BOUNDARY = 2.4;
  public static final double MID_PROXIMITY_BOUNDARY = 5.4;

  // defined such that the blue driver station is to the left
  public static final double BOTTOM_ZONE_BOUNDARY = 1.54;
  public static final double BOTTOM_STATION_ZONE_BOUNDARY = 3.978656;
  public static final double TOP_ZONE_BOUNDARY = 2.76;
  public static final double TOP_STATION_ZONE_BOUNDARY = 8.02;

  public static final Translation2d BOTTOM_FAR_INFLECTION_POINT = new Translation2d(5.4, 0.75);
  public static final Translation2d BOTTOM_CLOSE_INFLECTION_POINT = new Translation2d(2.4, 0.75);

  public static final Translation2d BOTTOM_STATION_FAR_INFLECTION_POINT =
      new Translation2d(5.4, 2.27);
  public static final Translation2d BOTTOM_STATION_CLOSE_INFLECTION_POINT =
      new Translation2d(2.4, 2.27);

  public static final Translation2d TOP_FAR_INFLECTION_POINT = new Translation2d(5.4, 4.73);
  public static final Translation2d TOP_CLOSE_INFLECTION_POINT = new Translation2d(2.4, 4.73);

  public static final Translation2d TOP_STATION_FAR_INFLECTION_POINT = new Translation2d(5.4, 3.18);
  public static final Translation2d TOP_STATION_CLOSE_INFLECTION_POINT =
      new Translation2d(2.4, 3.18);

  // each bay is scoring location. 0 is defined as the rightmost bay. These positions are based on
  // distance from the blue alliance station wall in the x axis, and distance from the field wall
  // such that the blue alliance station is to the left
  public static final List<Translation2d> DELIVERY_BAYS =
      Arrays.asList(
          new Translation2d(1.85718, 0.511626),
          new Translation2d(1.85718, 1.071626),
          new Translation2d(1.85718, 1.631626),
          new Translation2d(1.85718, 2.188026),
          new Translation2d(1.85718, 2.748026),
          new Translation2d(1.85718, 3.308026),
          new Translation2d(1.85718, 3.864426),
          new Translation2d(1.85718, 4.424426),
          new Translation2d(1.85718, 4.984426));

  public static final Translation2d DOUBLE_SUBSTATION = new Translation2d(15.7, 7.32);
  public static final Translation2d SINGLE_SUBSTATION = new Translation2d(14.24, 7.2);

  public static final Rotation2d SINGLE_SUB_ROTATION = new Rotation2d(Math.PI / 2);
  public static final Rotation2d DELIVERY_ROTATION = new Rotation2d(Math.PI);
  public static final Rotation2d DOUBLE_SUB_ROTATION = new Rotation2d(0);
  public static final double CAPSTAN_DIAMETER_METERS = 0.0254;
  public static final double ARM_ALLOWED_ANGLE_ERROR = 0.001;
  public static final double ARM_ALLOWED_EXTENSION_ERROR = 0.0254;
  public static final int BREAK_VICTOR_ID = 7;
  public static final double SHOULDER_CANCODER_OFFSET = 105.996;

  public static final int WRIST_CANCODER_ID = 14;
  public static final double WRIST_CANCODER_OFFSET = 51.855;
public static final ArmPosition ARM_MAX_POSITION =
      new ArmPosition(110, 360 * 13.8, 22002, ArmHeight.NOT_SPECIFIED);
  public static final ArmPosition ARM_MIN_POSITION =
      new ArmPosition(-6, 0, -21788, ArmHeight.NOT_SPECIFIED);

  public static final ArmPosition STOWED_ARM_POSITION =
      new ArmPosition(Math.toRadians(-7.5), 0, 8000, ArmHeight.STOWED);
  public static final ArmPosition HIGH_STOWED_ARM_POSITION =
      new ArmPosition(Math.toRadians(70), 0, -16000, ArmHeight.HIGH_STOWED);

  public static final ArmPosition LOW_CUBE_ARM_POSITION =
      new ArmPosition(Math.toRadians(-7.5), 0, 3300, ArmHeight.LOW);
  public static final ArmPosition LOW_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(-7.5), 0, -5700, ArmHeight.LOW);

  public static final ArmPosition MID_CUBE_ARM_POSITION =
      new ArmPosition(Math.toRadians(35), 360.0 * 5.95, -2358, ArmHeight.MID);
  public static final ArmPosition MID_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(44.75), 360.0 * 7.5, -21787, ArmHeight.MID);

  public static final ArmPosition HIGH_CUBE_ARM_POSITION =
      new ArmPosition(Math.toRadians(34.94), 360 * 10.35, 3498, ArmHeight.HIGH);
  public static final ArmPosition HIGH_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(45.5), 4600, -21000, ArmHeight.HIGH);
  public static final ArmPosition HIGHER_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(45.5), 4800, -21000, ArmHeight.HIGH);
  public static final ArmPosition SINGLE_SUBSTATION_CONE_POSITION =
      new ArmPosition(
          Math.toRadians(48.416748), 6.591797, -12481.000000, ArmHeight.SINGLE_SUBSTATION);
  public static final ArmPosition DOUBLE_SUBSTATION_CONE_POSITION =
      new ArmPosition(Math.toRadians(60), 880+360*1.5, -21000, ArmHeight.DOUBLE_SUBSTATION);

  public static final ArmPosition HYBRID_CUBE_ARM_POSITION =
      new ArmPosition(Math.toRadians(-7.5), 360, 23001, ArmHeight.LOW);

  public static final ArmPosition HYBRID_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(35), 360, -27000, ArmHeight.LOW);

  public static final ArmPosition BACKWARDS_HIGH_CONE =
      new ArmPosition(Math.toRadians(125), 8.2 * 360, 16000, ArmHeight.NOT_SPECIFIED);

  public static final ArmPosition BACKWARDS_MID_CONE = 
      new ArmPosition(Math.toRadians(114.56), 1066, 19660, ArmHeight.NOT_SPECIFIED);

  public static final ArmPosition AUTON_SAFECHUCK =
      new ArmPosition(Math.toRadians(20), 0, 17000, ArmHeight.NOT_SPECIFIED);
      
  public static final ArmPosition SELF_RIGHT = 
      new ArmPosition(Math.toRadians(125), 0, 0, ArmHeight.HIGH);

  // A value between 0 and 1.
  public static final double RAINBOW_ANIMATION_SPEED = 0.5;

  public static final double ELEVATOR_ZEROING_PERCENT = -0.2;
  public static final int ELEVATOR_ZERO_AMPRAGE = 20;
  public static final double ELEVATOR_ZEROING_TIME_SECONDS = 0.2;

  public static final int GAME_PIECE_DETECTION_VELOCITY = 500;
  public static final double GAME_PIECE_DETECTION_WAIT = 0.1;
  public static final double RUMBLE_INTENSITY = 0.75;
  public static final double GAME_PIECE_SIGNALING_TIME = 0.5;
  public static final double FLASHING_TIME = 0.2;
}
