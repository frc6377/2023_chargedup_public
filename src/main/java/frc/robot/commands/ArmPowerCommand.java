package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotStateManager;
import frc.robot.subsystems.arm.ArmHeight;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.color.GamePieceMode;

public class ArmPowerCommand extends CommandBase {

  private PolarPoint initalPose;
  private double targetWristAngle;

  private PolarPoint targetPose;

  private final ArmSubsystem armSubsystem;
  private ArmPosition targetPosition;
  private ArmHeight targetHeight;
  private final double pow;
  private final IntegerSubscriber gamePieceModeSubscriber =
      NetworkTableInstance.getDefault().getIntegerTopic("GAME_PIECE_MODE").subscribe(10);
  private final RobotStateManager robotState;
  private final boolean calculateArmPosition;

  /**
   * Moves the arm to a specified position with minimal momentum. Accepts an armHeight and decides
   * position based on game piece mode when executed.
   *
   * @param armHeight The armHeight to go to
   * @param armSubsystem Arm subsystem
   * @param pow Power with which to run
   * @param robotState Robot state manager
   */
  public ArmPowerCommand(
      final ArmHeight armHeight,
      final ArmSubsystem armSubsystem,
      final double pow,
      final RobotStateManager robotState) {
    this.armSubsystem = armSubsystem;
    targetHeight = armHeight;
    this.pow = pow;
    this.robotState = robotState;
    calculateArmPosition = true;
    addRequirements(armSubsystem, robotState);
  }
  /**
   * Moves the arm to a specified position with minimal momentum
   *
   * @param targetPosition The armPosition to go to
   * @param armSubsystem Arm subsystem
   * @param pow Power with which to run
   * @param robotState Robot state
   */
  public ArmPowerCommand(
      ArmPosition targetPosition,
      ArmSubsystem armSubsystem,
      double pow,
      RobotStateManager robotState) {
    this.targetPose =
        new PolarPoint(
            targetPosition.getArmRotationRadians(),
            MathUtil.clamp(targetPosition.getArmExtension(), 0, 13.8 * 360.0));
    this.targetWristAngle = targetPosition.getWristRotation();
    this.armSubsystem = armSubsystem;
    this.targetPosition = targetPosition;
    this.pow = pow;
    this.robotState = robotState;
    calculateArmPosition = false;
    addRequirements(armSubsystem, robotState);
  }

  @Override
  public void initialize() {
    // If the command was initialized with an armHeight, we need to recalculate the target position
    // each time the command is run based on the robot state
    if (calculateArmPosition) {
      targetPosition =
          ArmPosition.getArmPositionFromHeightAndType(
              targetHeight, GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()));
      this.targetPose =
          new PolarPoint(
              targetPosition.getArmRotationRadians(),
              MathUtil.clamp(targetPosition.getArmExtension(), 0, 13.8 * 360.0));
      this.targetWristAngle = targetPosition.getWristRotation();
    } else {
      targetHeight = targetPosition.getHeight();
    }

    robotState.setArmTargetHeight(targetHeight);

    initalPose =
        new PolarPoint(
            armSubsystem.shoulderThetaFromCANCoder(), armSubsystem.currentArmExtenstionMeters());
  }

  @Override
  public void execute() {
    double armExtension =
        (targetPose.theta != initalPose.theta) ? computeExtension() : targetPose.r;
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
    return armSubsystem.thetaFromPPC() == targetPose.theta;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm power command finished | interupted: " + interrupted);
    if (targetPosition.equals(ArmPosition.HIGH_CONE_ARM_POSITION)) {
      armSubsystem.setTarget(ArmPosition.HIGHER_CONE_ARM_POSITION);
    }
  }

  private double computeExtension() {

    double theta = armSubsystem.thetaFromPPC();
    double thetaRatio = (theta - initalPose.theta) / (targetPose.theta - initalPose.theta);
    double extensionDelta = targetPose.r - initalPose.r;
    return Math.pow(thetaRatio, computePow(extensionDelta)) * extensionDelta + initalPose.r;
  }

  private double computePow(double extensionDelta) {
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
