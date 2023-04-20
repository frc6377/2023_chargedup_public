package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class AutoBalanceCommand extends CommandBase {
  private final DoubleArraySubscriber sub =
      NetworkTableInstance.getDefault()
          .getDoubleArrayTopic("pitch")
          .subscribe(new double[] {0, 0, 0});
  private final DrivetrainSubsystem drive;
  private final ProfiledPIDController pitchController;
  private int count = 0;

  public AutoBalanceCommand(DrivetrainSubsystem drive) {
    this.drive = drive;
    pitchController =
        new ProfiledPIDController(
            0.0333,
            0,
            0.01,
            new TrapezoidProfile.Constraints(
                Constants.AUTO_MAX_VELOCITY, Constants.AUTO_MAX_ACCELERATION));
  }

  @Override
  public void initialize() {
    pitchController.setGoal(0);
  }

  @Override
  public void execute() {
    double theta = computeTheta();
    drive.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(-pitchController.calculate(theta), 0, 0),
            new Rotation2d(Math.toRadians(sub.get()[0]))));
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(computeTheta()) <= 2.5) {
      count++;
    } else {
      count = 0;
    }

    return count > 10;
  }

  @Override
  public void end(boolean interupted) {
    drive.drive(new ChassisSpeeds());
  }

  private double computeTheta() {
    double yaw = (sub.get()[0]);
    double pitch = (sub.get()[1]);
    double roll = (sub.get()[2]);

    // DONT TOUCH UNLESS U HAVE A FIX. I AM A BROKEN MAN

    double sign = 0;
    if (yaw >= -45 && yaw < 45) {
      sign = pitch;
    } else if (yaw >= 45 && yaw < 135) {
      sign = -roll;

    } else if (yaw <= -45 && yaw > -135) {
      sign = roll;

    } else {
      sign = -pitch;
    }

    return Math.copySign(Math.hypot(roll, pitch), -sign);
  }
}
