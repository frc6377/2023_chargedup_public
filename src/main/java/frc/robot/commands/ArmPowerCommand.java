package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.Supplier;

public class ArmPowerCommand extends CommandBase {

  private PolarPoint initalPose;
  private double targetWristAngle;
  private Supplier<ArmPosition> targetPositionSupplier;

  private PolarPoint targetPose;

  private final ArmSubsystem armSubsystem;
  private ArmHeight targetHeight;
  private final double pow;

  public ArmPowerCommand(ArmPosition targetPosition, ArmSubsystem armSubsystem, double pow) {
    this.targetPose =
        new PolarPoint(
            targetPosition.getArmRotation(),
            MathUtil.clamp(targetPosition.getArmExtension(), 0, 13.8 * 360.0));
    this.targetWristAngle = targetPosition.getWristRotation();
    this.armSubsystem = armSubsystem;
    targetHeight = targetPosition.getHeight();
    this.pow = pow;
    addRequirements(armSubsystem);
  }

  public ArmPowerCommand(
      Supplier<ArmPosition> targetPositionSupplier, ArmSubsystem armSubsystem, double pow) {
    this.targetPositionSupplier = targetPositionSupplier;
    this.armSubsystem = armSubsystem;
    this.pow = pow;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    if (targetPose == null) {
      ArmPosition targetPosition = targetPositionSupplier.get();
      this.targetPose =
          new PolarPoint(
              targetPosition.getArmRotation(),
              MathUtil.clamp(targetPosition.getArmExtension(), 0, 13.8 * 360.0));
      this.targetWristAngle = targetPosition.getWristRotation();
      targetHeight = targetPosition.getHeight();
    }
    
    initalPose =
        new PolarPoint(
            armSubsystem.shoulderThetaFromCANCoder(), armSubsystem.currentArmExtenstionMeters());
  }

  @Override
  public void execute() {
    double armExtension =
        (targetPose.theta != initalPose.theta)
            ? computeExtension()
            : targetPose.r; // math breaks if theta doesnt change
    armSubsystem.setTarget(
        new ArmPosition(
            targetPose.theta,
            MathUtil.clamp(
                armExtension,
                Math.min(targetPose.r, initalPose.r),
                Math.max(targetPose.r, initalPose.r)),
            targetWristAngle,
            targetHeight));
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.thetaFromPPC()
        == targetPose.theta; // once this is true the command has delivered its final
    // setpoint and its job is
    // done
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm power command finished | interupted: " + interrupted);
  }

  private double computeExtension() {

    // math from https://www.desmos.com/calculator/r6gnd4hvdc
    double theta = armSubsystem.thetaFromPPC();
    double thetaRatio = (theta - initalPose.theta) / (targetPose.theta - initalPose.theta);
    double extensionDelta = targetPose.r - initalPose.r;
    // DeltaBoard.putNumber("Pow", computePow(extensionDelta));
    return Math.pow(thetaRatio, computePow(extensionDelta)) * extensionDelta + initalPose.r;
  }

  private double computePow(double extensionDelta) {
    // if we extend more we want to it last, if we retract we want to do it first.
    return (extensionDelta < 0) ? pow : pow;
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
