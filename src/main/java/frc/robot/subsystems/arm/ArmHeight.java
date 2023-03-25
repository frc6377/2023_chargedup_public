package frc.robot.subsystems.arm;

public enum ArmHeight {
  STOWED,
  HIGH_STOWED,
  HIGH,
  MID,
  LOW,
  NOT_SPECIFIED,
  SINGLE_SUBSTATION,
  DOUBLE_SUBSTATION;

  public boolean isStowed() {
    return this == ArmHeight.STOWED || this == ArmHeight.HIGH_STOWED;
  }
}
