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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.config.RobotVersion;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.config.PodConfig;

public class DrivetrainSubsystem extends SubsystemBase {

  private final SwerveModuleState[] xStates =
      new SwerveModuleState[] {
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + Math.PI / 2)),
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + Math.PI - Math.PI / 2)),
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4 + Math.PI + Math.PI / 2 - Math.PI / 2))
      };

  // Constants
  public static final double MAX_MOTOR_VOLTAGE = 12.0;

  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      5880.0
          / 60.0
          * SdsModuleConfigurations.MK4I_L3.getDriveReduction()
          * SdsModuleConfigurations.MK4I_L3.getWheelDiameter()
          * Math.PI;

  public final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

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

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private final Field2d m_field2d = new Field2d();

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem(RobotVersion robotVersion) {

    SmartDashboard.putData("Field", m_field2d);
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    Mk4ModuleConfiguration modCfg = new Mk4ModuleConfiguration();
    modCfg.setDriveCurrentLimit(45);
    PodConfig frontLeftPodConfig =
        robotVersion == RobotVersion.V1
            ? Constants.FRONT_LEFT_V1_CONFIG
            : Constants.FRONT_LEFT_V2_CONFIG;
    m_frontLeftModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
            modCfg,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            frontLeftPodConfig.getDriveID(),
            frontLeftPodConfig.getTurnID(),
            frontLeftPodConfig.getCanCoderID(),
            frontLeftPodConfig.getCanCoderOffset());

    PodConfig frontRightPodConfig =
        robotVersion == RobotVersion.V1
            ? Constants.FRONT_RIGHT_V1_CONFIG
            : Constants.FRONT_RIGHT_V2_CONFIG;
    m_frontRightModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
            modCfg,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            frontRightPodConfig.getDriveID(),
            frontRightPodConfig.getTurnID(),
            frontRightPodConfig.getCanCoderID(),
            frontRightPodConfig.getCanCoderOffset());

    PodConfig backLeftPodConfig =
        robotVersion == RobotVersion.V1
            ? Constants.BACK_LEFT_V1_CONFIG
            : Constants.BACK_LEFT_V2_CONFIG;
    m_backLeftModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
            modCfg,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            backLeftPodConfig.getDriveID(),
            backLeftPodConfig.getTurnID(),
            backLeftPodConfig.getCanCoderID(),
            backLeftPodConfig.getCanCoderOffset());

    PodConfig backRightPodConfig =
        robotVersion == RobotVersion.V1
            ? Constants.BACK_RIGHT_V1_CONFIG
            : Constants.BACK_RIGHT_V2_CONFIG;
    m_backRightModule =
        Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
            modCfg,
            Mk4iSwerveModuleHelper.GearRatio.L3,
            backRightPodConfig.getDriveID(),
            backRightPodConfig.getTurnID(),
            backRightPodConfig.getCanCoderID(),
            backRightPodConfig.getCanCoderOffset());
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

  public SwerveModuleState[] getOdometry() {
    SwerveModuleState[] states =
        new SwerveModuleState[] {
          new SwerveModuleState(
              m_frontLeftModule.getDriveVelocity(),
              new Rotation2d(m_frontLeftModule.getSteerAngle())),
          new SwerveModuleState(
              m_frontRightModule.getDriveVelocity(),
              new Rotation2d(m_frontRightModule.getSteerAngle())),
          new SwerveModuleState(
              m_backLeftModule.getDriveVelocity(),
              new Rotation2d(m_backLeftModule.getSteerAngle())),
          new SwerveModuleState(
              m_backRightModule.getDriveVelocity(),
              new Rotation2d(m_backRightModule.getSteerAngle())),
        };
    return states;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        new SwerveModuleState[] {
          new SwerveModuleState(
              m_frontLeftModule.getDriveVelocity(),
              new Rotation2d(m_frontLeftModule.getSteerAngle())),
          new SwerveModuleState(
              m_frontRightModule.getDriveVelocity(),
              new Rotation2d(m_frontRightModule.getSteerAngle())),
          new SwerveModuleState(
              m_backLeftModule.getDriveVelocity(),
              new Rotation2d(m_backLeftModule.getSteerAngle())),
          new SwerveModuleState(
              m_backRightModule.getDriveVelocity(),
              new Rotation2d(m_backRightModule.getSteerAngle()))
        });
  }

  @Override
  public void periodic() {

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    if (Math.hypot(m_chassisSpeeds.vxMetersPerSecond, m_chassisSpeeds.vyMetersPerSecond) == 0
        && m_chassisSpeeds.omegaRadiansPerSecond == 0) {
      states = xStates;
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(
        states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_MOTOR_VOLTAGE,
        states[0].angle.getRadians());
    m_frontRightModule.set(
        states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_MOTOR_VOLTAGE,
        states[1].angle.getRadians());
    m_backLeftModule.set(
        states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_MOTOR_VOLTAGE,
        states[2].angle.getRadians());
    m_backRightModule.set(
        states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_MOTOR_VOLTAGE,
        states[3].angle.getRadians());
  }
}
