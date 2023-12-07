package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.color.GamePieceMode;

public class ArmPosition {
  public static ArmPosition ARM_MAX_POSITION =
      new ArmPosition(110, 360 * 13.8, 22002, ArmHeight.NOT_SPECIFIED);
  public static ArmPosition ARM_MIN_POSITION =
      new ArmPosition(-6, 0, -21788, ArmHeight.NOT_SPECIFIED);
  public static ArmPosition AUTO_STOWED_ARM_POSITION =
      new ArmPosition(Math.toRadians(-7.5), 0, 8000, ArmHeight.AUTO_STOWED);
  public static ArmPosition HIGH_STOWED_ARM_POSITION =
      new ArmPosition(Math.toRadians(70), 0, -16000, ArmHeight.HIGH_STOWED);
  public static ArmPosition LOW_CUBE_ARM_POSITION =
      new ArmPosition(Math.toRadians(-7.5), 0, 3300, ArmHeight.LOW);
  public static ArmPosition LOW_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(-7.5), 0, -5700, ArmHeight.LOW);
  public static ArmPosition MID_CUBE_ARM_POSITION =
      new ArmPosition(Math.toRadians(35), 360.0 * 5.95, -2358, ArmHeight.MID);
  public static ArmPosition MID_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(44.75), 360.0 * 7.5, -21787, ArmHeight.MID);
  public static ArmPosition HIGH_CUBE_ARM_POSITION =
      new ArmPosition(Math.toRadians(34.94), 360 * 10.35, 3498, ArmHeight.HIGH);
  public static ArmPosition HIGH_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(45.5), 4600, -21000, ArmHeight.HIGH);
  public static ArmPosition HIGHER_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(45.5), 4800, -21000, ArmHeight.HIGH);
  public static ArmPosition SINGLE_SUBSTATION_CONE_POSITION =
      new ArmPosition(
          Math.toRadians(48.416748), 6.591797, -12481.000000, ArmHeight.SINGLE_SUBSTATION);
  public static ArmPosition DOUBLE_SUBSTATION_CONE_POSITION =
      new ArmPosition(Math.toRadians(60), 880 + 360 * 1.5, -21000, ArmHeight.DOUBLE_SUBSTATION);
  public static ArmPosition HYBRID_CUBE_ARM_POSITION =
      new ArmPosition(Math.toRadians(-7.5), 360, 23001, ArmHeight.LOW);
  public static ArmPosition HYBRID_CONE_ARM_POSITION =
      new ArmPosition(Math.toRadians(35), 360, -27000, ArmHeight.LOW);
  public static ArmPosition BACKWARDS_HIGH_CONE_POSITION =
      new ArmPosition(Math.toRadians(125), 8.2 * 360, 16000, ArmHeight.NOT_SPECIFIED);
  public static ArmPosition BACKWARDS_MID_CONE_POSITION =
      new ArmPosition(Math.toRadians(114.56), 1066, 19660, ArmHeight.NOT_SPECIFIED);
  public static ArmPosition AUTON_SAFECHUCK_POSITION =
      new ArmPosition(Math.toRadians(20), 0, 17000, ArmHeight.NOT_SPECIFIED);
  public static ArmPosition SELF_RIGHT_POSITION =
      new ArmPosition(Math.toRadians(125), 0, 0, ArmHeight.HIGH);

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
          return HIGH_CUBE_ARM_POSITION;
        } else return HIGH_CONE_ARM_POSITION;

      case HIGH_STOWED:
        if (gamePieceMode == GamePieceMode.SINGLE_SUBSTATION) {
          return SINGLE_SUBSTATION_CONE_POSITION;
        } else return HIGH_STOWED_ARM_POSITION;

      case STOWED:
        if (gamePieceMode == GamePieceMode.SINGLE_SUBSTATION) {
          return SINGLE_SUBSTATION_CONE_POSITION;
        } else return HYBRID_CUBE_ARM_POSITION;

      case LOW:
        if (isCube) return LOW_CUBE_ARM_POSITION;
        else return LOW_CONE_ARM_POSITION;

      case MID:
        if (isCube) {
          return MID_CUBE_ARM_POSITION;
        } else return MID_CONE_ARM_POSITION;

      case HYBRID:
        if (isCube) {
          return HYBRID_CUBE_ARM_POSITION;
        } else return HYBRID_CONE_ARM_POSITION;

      case AUTO_STOWED:
        return AUTO_STOWED_ARM_POSITION;

      default:
        return AUTO_STOWED_ARM_POSITION;
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

  // @Override
  // public boolean equals(Object o) {
  //   if (o.getClass() != this.getClass()) return false;
  //   ArmPosition other = (ArmPosition) o;
  //   return other.armExtension == this.armExtension
  //       && this.armRotation == other.armRotation
  //       && this.wristRotation == other.wristRotation;
  // }
}
