package frc.robot.subsystems.arm;

public enum ArmHeight {
  STOWED,
  HIGH_STOWED,
  HIGH,
  MID,
  LOW,
  NOT_SPECIFIED,
  DOUBLE_SUBSTATION;

  public boolean isStowed() {
    System.out.println(this);
    return this == ArmHeight.STOWED || this == ArmHeight.HIGH_STOWED;
  }
}
