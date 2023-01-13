// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.*;

import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.config.PodConstants;
import frc.robot.subsystems.drivetrain.config.PodLocation;
import frc.robot.subsystems.drivetrain.config.Pods;
import java.util.function.DoubleSupplier;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   *
   * <p>This can be reduced to cap the robot's maximum speed. Typically, this is useful during
   * initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;

  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      5880.0
          / 60.0
          * SdsModuleConfigurations.MK4I_L3.getDriveReduction()
          * SdsModuleConfigurations.MK4I_L3.getWheelDiameter()
          * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a
  // measured amount.
  public final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private PodConstants frontLeftPodConstants =
      Pods.getPod(FRONT_LEFT_POD_NAME, PodLocation.FRONT_LEFT);

  private PodConstants frontRightPodConstants =
      Pods.getPod(FRONT_RIGHT_POD_NAME, PodLocation.FRONT_RIGHT);

  private PodConstants backLeftPodConstants =
      Pods.getPod(BACK_LEFT_POD_NAME, PodLocation.BACK_LEFT);

  private PodConstants backRightPodConstants =
      Pods.getPod(BACK_RIGHT_POD_NAME, PodLocation.BACK_RIGHT);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(
              -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  private final DoubleSupplier yawSupplier;

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final Field2d m_field2d = new Field2d();

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem(DoubleSupplier yawSupplier) {
    this.yawSupplier = yawSupplier;

    SmartDashboard.putData("Field", m_field2d);
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is
    // for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is
    // for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    Mk4ModuleConfiguration modCfg = new Mk4ModuleConfiguration();
    modCfg.setDriveCurrentLimit(45);

    m_frontLeftModule =
        Mk4iSwerveModuleHelper.createNeo(
            // This parameter is optional, but will allow you to see the current state of the module
            // on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
            modCfg,
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L3,
            // This is the ID of the drive motor
            frontLeftPodConstants.DRIVE_MOTOR,
            // This is the ID of the steer motor
            frontLeftPodConstants.STEER_MOTOR,
            // This is the ID of the steer encoder
            frontLeftPodConstants.STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is
            // facing straight forward)
            frontLeftPodConstants.STEER_OFFSET);

    // We will do the same for the other modules
    m_frontRightModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
            modCfg,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            frontRightPodConstants.DRIVE_MOTOR,
            frontRightPodConstants.STEER_MOTOR,
            frontRightPodConstants.STEER_ENCODER,
            frontRightPodConstants.STEER_OFFSET);

    m_backLeftModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
            modCfg,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            backLeftPodConstants.DRIVE_MOTOR,
            backLeftPodConstants.STEER_MOTOR,
            backLeftPodConstants.STEER_ENCODER,
            backLeftPodConstants.STEER_OFFSET);

    m_backRightModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
            modCfg,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            backRightPodConstants.DRIVE_MOTOR,
            backRightPodConstants.STEER_MOTOR,
            backRightPodConstants.STEER_ENCODER,
            backRightPodConstants.STEER_OFFSET);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(0);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void updateAutoDemand(SwerveModuleState[] states) {
    drive(m_kinematics.toChassisSpeeds(states));
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void sendTrajectoryToNT(Trajectory traj) {
    m_field2d.getObject("traj").setTrajectory(traj);
  }

  private SwerveModulePosition[] getOdometry() {
    SwerveModulePosition[] states =
        new SwerveModulePosition[] {
          new SwerveModulePosition(
              m_frontLeftModule.getDriveVelocity(),
              new Rotation2d(m_frontLeftModule.getSteerAngle())),
          new SwerveModulePosition(
              m_frontRightModule.getDriveVelocity(),
              new Rotation2d(m_frontRightModule.getSteerAngle())),
          new SwerveModulePosition(
              m_backLeftModule.getDriveVelocity(),
              new Rotation2d(m_backLeftModule.getSteerAngle())),
          new SwerveModulePosition(
              m_backRightModule.getDriveVelocity(),
              new Rotation2d(m_backRightModule.getSteerAngle())),
        };
    return states;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    SmartDashboard.putNumber(
        "front left pod",
        states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE);

    m_frontLeftModule.set(
        states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[0].angle.getRadians());

    m_frontRightModule.set(
        states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[1].angle.getRadians());

    m_backLeftModule.set(
        states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[2].angle.getRadians());

    m_backRightModule.set(
        states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[3].angle.getRadians());
  }
}