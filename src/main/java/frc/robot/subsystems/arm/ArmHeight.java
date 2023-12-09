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

  /**
   * Represents a specific height of the arm. Each height has several positions that correspond with
   * different game piece modes.
   *
   * @param integerValue ArmHeight ID.
   */
  ArmHeight(final int integerValue) {
    this.integerValue = integerValue;
  }

  public int getAsInt() {
    return integerValue;
  }

  /**
   * Creates an ArmHeight corresponding to a numerical value.
   *
   * @param integerValue ArmHeight ID.
   */
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
