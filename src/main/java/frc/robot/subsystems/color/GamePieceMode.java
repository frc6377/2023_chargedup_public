package frc.robot.subsystems.color;

public enum GamePieceMode {
  CUBE,
  CONE,
  SINGLE_SUBSTATION;

  public RGB color() {
    if (this == CUBE) return RGB.PURPLE;
    if (this == CONE) return RGB.YELLOW;
    if (this == SINGLE_SUBSTATION) return RGB.GREEN;
    return RGB.WHITE;
  }

  public boolean isCube() {
    return this == CUBE;
  }

  public boolean isCone() {
    return this != CUBE;
  }
}
