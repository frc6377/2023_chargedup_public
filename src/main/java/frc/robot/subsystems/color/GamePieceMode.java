package frc.robot.subsystems.color;

import java.util.Arrays;

public enum GamePieceMode {
  CUBE(10),
  CONE(20),
  SINGLE_SUBSTATION(30);

  private final int integerValue;

  GamePieceMode(final int integerValue) {
    this.integerValue = integerValue;
  }

  public int getAsInt() {
    return integerValue;
  }

  public static GamePieceMode getFromInt(final int integerValue) {
    return Arrays.stream(GamePieceMode.values())
        .filter(v -> v.getAsInt() == integerValue)
        .findFirst()
        .orElse(CUBE);
  }

  public boolean isCube() {
    return this == CUBE;
  }

  public boolean isCone() {
    return this != CUBE;
  }
}
