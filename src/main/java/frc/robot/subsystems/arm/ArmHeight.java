package frc.robot.subsystems.arm;

import java.util.Arrays;

public enum ArmHeight {
  NOT_SPECIFIED(0),
  AUTO_STOWED(10),
  STOWED(15),
  HIGH_STOWED(20),
  HIGH(30),
  MID(40),
  LOW(50),
  HYBRID(60),
  SINGLE_SUBSTATION(70),
  DOUBLE_SUBSTATION(80);

  private final int integerValue;

  ArmHeight(final int integerValue) {
    this.integerValue = integerValue;
  }

  public int getAsInt() {
    return integerValue;
  }

  public static ArmHeight getFromInt(final int integerValue) {
    return Arrays.stream(ArmHeight.values())
        .filter(v -> v.getAsInt() == integerValue)
        .findFirst()
        .orElse(NOT_SPECIFIED);
  }

  public boolean isStowed() {
    return this == ArmHeight.AUTO_STOWED || this == ArmHeight.HIGH_STOWED;
  }
}
