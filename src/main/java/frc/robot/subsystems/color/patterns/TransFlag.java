package frc.robot.subsystems.color.patterns;

import frc.robot.subsystems.color.RGB;

public class TransFlag {
  public static int numberOfLEDS;
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.BLACK, 1),
    new PatternNode(RGB.HOWDY_BLUE, 4),
    new PatternNode(RGB.PINK, 4),
    new PatternNode(RGB.BLUE, 4),
    new PatternNode(RGB.PINK, 4),
    new PatternNode(RGB.HOWDY_BLUE, 4),
  };

  public static PatternNode[] getColors(int step) {
    int initalStep = step % pattern.length;
    PatternNode[] fullPattern = new PatternNode[numberOfLEDS];
    for (int i = 0; i < numberOfLEDS; i++) {
      fullPattern[i] = pattern[(i + initalStep) % pattern.length];
    }
    return fullPattern;
  }
}
