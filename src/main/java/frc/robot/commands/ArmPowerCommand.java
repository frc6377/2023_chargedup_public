package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmPowerCommand extends CommandBase {

  private PolarPoint initalPose;
  private double targetWristAngle;
  private final PolarPoint targetPose;
  private final ArmSubsystem armSubsystem;
  private final double pow;

  public ArmPowerCommand(ArmPosition targetPosition, ArmSubsystem armSubsystem, double pow) {
    this.targetPose = new PolarPoint(targetPosition.armRotation, targetPosition.armExtension);
    this.targetWristAngle = targetPosition.wristRotation;
    this.armSubsystem = armSubsystem;
    this.pow = pow;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    initalPose =
        new PolarPoint(armSubsystem.thetaFromCANCoder(), armSubsystem.currentArmExtenstion());
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
