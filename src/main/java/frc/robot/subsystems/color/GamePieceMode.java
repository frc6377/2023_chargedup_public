package frc.robot.subsystems.color;

public enum GamePieceMode {
  CUBE,
  CONE,
  SINGLE_SUBSTATION;

  public RGB color() {
    if (this == CUBE) return RGB.PURPLE;
    if (this == CONE || this == SINGLE_SUBSTATION) return RGB.YELLOW;
    return RGB.WHITE;
  }

  public boolean shouldFlash() {
    return this == SINGLE_SUBSTATION;
  }

  public boolean isCube() {
    return this == CUBE;
  }

  public boolean isCone() {
    return this != CUBE;
  }
}
