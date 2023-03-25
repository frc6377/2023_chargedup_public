package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class OmegaBalance extends CommandBase {
  private final DoubleArraySubscriber omegaSub =
      NetworkTableInstance.getDefault()
          .getDoubleArrayTopic("omegas")
          .subscribe(new double[] {0, 0, 0});
  private final DoubleArraySubscriber thetaSub =
      NetworkTableInstance.getDefault()
          .getDoubleArrayTopic("pitch")
          .subscribe(new double[] {0, 0, 0});
  private final DrivetrainSubsystem drive;
  private int count = 0;
  private final double driveVelocity;
  private final double targetOmega;

  public OmegaBalance(DrivetrainSubsystem drive, double driveVelocity, double targetOmega) {
    this.drive = drive;
    this.driveVelocity = driveVelocity;
    this.targetOmega = targetOmega;
  }

  @Override
  public void execute() {
    drive.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(driveVelocity * computeSign(), 0, 0),
            new Rotation2d(Math.toRadians(omegaSub.get()[0]))));
  }

  @Override
  public boolean isFinished() {
    if (computeOmega() > targetOmega) {
      count++;
    } else {
      count = 0;
    }

    return count > 3 || isLevel();
  }

  @Override
  public void end(boolean interupted) {
    drive.drive(new ChassisSpeeds());
  }

  private double computeOmega() {
    double pitch = (omegaSub.get()[1]);
    double roll = (omegaSub.get()[2]);

    System.out.println("pitch " + pitch + " roll " + roll);
    return Math.hypot(roll, pitch);
  }
  // DONT TOUCH UNLESS U HAVE A FIX. I AM A BROKEN MAN
  private double computeSign() {
    double yaw = (thetaSub.get()[0]);
    double pitch = (thetaSub.get()[1]);
    double roll = (thetaSub.get()[2]);

    double sign = 0;
    if (yaw >= -45 && yaw < 45) {
      sign = roll;
    } else if (yaw >= 45 && yaw < 135) {
      sign = -pitch;

    } else if (yaw <= -45 && yaw > -135) {
      sign = pitch;

    } else {
      sign = -roll;
    }

    return Math.copySign(1, sign);
  }

  private boolean isLevel() {
    double pitch = (thetaSub.get()[1]);
    double roll = (thetaSub.get()[2]);

    return Math.hypot(pitch, roll) < 2.5;
  }
}
