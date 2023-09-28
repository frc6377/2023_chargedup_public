package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.color.GamePieceMode;

public class ArmPosition {
  protected final double armRotation;
  protected final double armExtension;
  protected final double wristRotation;
  protected final ArmHeight height;
  protected final String description;

  public ArmPosition(
      double armRotation, double armExtension, double wristRotation, ArmHeight armHeight) {
    this.armRotation = armRotation;
    this.armExtension = armExtension;
    this.wristRotation = wristRotation;
    this.height = armHeight;
    this.description =
        String.format(
            "%s - armRotation: %.3f, armExtension: %.3f, wristRotation: %.3f",
            armHeight.name(), armRotation, armExtension, wristRotation);
  }

  public ArmPosition add(ArmPosition armPos2) {
    return new ArmPosition(
        armRotation + armPos2.getArmRotation(),
        armExtension + armPos2.getArmExtension(),
        wristRotation + armPos2.getWristRotation(),
        ArmHeight.NOT_SPECIFIED);
  }

  public String toString() {
    return description;
  }

  public double getArmRotation() {
    return armRotation;
  }

  public double getArmExtension() {
    return armExtension;
  }

  public double getWristRotation() {
    return wristRotation;
  }

  public ArmHeight getHeight() {
    return height;
  }

  public String getDescription() {
    return description;
  }

  public static ArmPosition getArmPositionFromHeightAndType(
      ArmHeight height, GamePieceMode gamePieceMode) {
    boolean isCube = gamePieceMode.isCube();
    switch (height) {
      case HIGH:
        if (isCube) {
          return Constants.HIGH_CUBE_ARM_POSITION;
        } else return Constants.HIGH_CONE_ARM_POSITION;

      case HIGH_STOWED:
        if (gamePieceMode == GamePieceMode.SINGLE_SUBSTATION) {
          return Constants.SINGLE_SUBSTATION_CONE_POSITION;
        } else return Constants.HIGH_STOWED_ARM_POSITION;

      case LOW:
        if (isCube) return Constants.LOW_CUBE_ARM_POSITION;
        else return Constants.LOW_CONE_ARM_POSITION;

      case MID:
        if (isCube) {
          return Constants.MID_CUBE_ARM_POSITION;
        } else return Constants.MID_CONE_ARM_POSITION;

      case HYBRID:
        if (isCube) {
          return Constants.HYBRID_CUBE_ARM_POSITION;
        } else return Constants.HYBRID_CONE_ARM_POSITION;

      case STOWED:
        return Constants.STOWED_ARM_POSITION;

      default:
        return Constants.STOWED_ARM_POSITION;
    }
  }

  public ArmPosition clamp(ArmPosition armMinPosition, ArmPosition armMaxPosition) {
    ArmPosition clamped =
        new ArmPosition(
            MathUtil.clamp(
                this.armRotation, armMinPosition.getArmRotation(), armMaxPosition.getArmRotation()),
            MathUtil.clamp(
                this.armExtension,
                armMinPosition.getArmExtension(),
                armMaxPosition.getArmExtension()),
            MathUtil.clamp(
                this.wristRotation,
                armMinPosition.getWristRotation(),
                armMaxPosition.getWristRotation()),
            ArmHeight.NOT_SPECIFIED);

    if (clamped.equals(this)) {
      return this;
    } else {
      return clamped;
    }
  }

  @Override
  public boolean equals(Object o) {
    if (o.getClass() != this.getClass()) return false;
    ArmPosition other = (ArmPosition) o;
    return other.armExtension == this.armExtension
        && this.armRotation == other.armRotation
        && this.wristRotation == other.wristRotation;
  }
}
