package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.Supplier;

public class ArmPowerCommand extends CommandBase {

  private final Supplier<ArmPosition> armPositionSupplier;
  private final ArmSubsystem armSubsystem;
  private final double pow;

  private PolarPoint initalPose;
  private PolarPoint targetPose;
  private double targetWristAngle;

  public ArmPowerCommand(
      final Supplier<ArmPosition> armPositionSupplier,
      final ArmSubsystem armSubsystem,
      final double pow) {
    this.armPositionSupplier = armPositionSupplier;
    this.armSubsystem = armSubsystem;
    this.pow = pow;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    initalPose =
        new PolarPoint(armSubsystem.thetaFromCANCoder(), armSubsystem.currentArmExtenstion());
    final ArmPosition targetPosition = armPositionSupplier.get();
    targetPose = new PolarPoint(targetPosition.getArmRotation(), targetPosition.getArmRotation());
    targetWristAngle = targetPosition.getWristRotation();
  }

  @Override
  public void execute() {
    double armExtension =
        (targetPose.theta != initalPose.theta)
            ? computeExtension()
            : targetPose.r; // math breaks if theta doesnt change

    armSubsystem.setTarget(new ArmPosition(targetPose.theta, armExtension, targetWristAngle, ""));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(armSubsystem.thetaFromCANCoder() - targetPose.theta)
            < Constants.ARM_ALLOWED_ANGLE_ERROR
        && Math.abs(armSubsystem.currentArmExtenstion() - targetPose.r)
            < Constants.ARM_ALLOWED_EXTENSION_ERROR;
  }

  private double computeExtension() {

    // math from https://www.desmos.com/calculator/r6gnd4hvdc
    double theta = armSubsystem.thetaFromCANCoder();
    double thetaRatio = (theta - initalPose.theta) / (targetPose.theta - initalPose.theta);
    double extensionDelta = targetPose.r - initalPose.r;
    return Math.pow(thetaRatio, computePow(extensionDelta)) * extensionDelta + initalPose.r;
  }

  private double computePow(double extensionDelta) {
    // if we extend more we want to it last, if we retract we want to do it first.
    return (extensionDelta < 0) ? 1 / pow : pow;
  }

  private class PolarPoint {
    public final double theta;
    public final double r;

    public PolarPoint(double theta, double r) {
      this.theta = theta;
      this.r = r;
    }
  }
}
