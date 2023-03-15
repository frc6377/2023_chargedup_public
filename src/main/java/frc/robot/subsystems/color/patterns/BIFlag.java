package frc.robot.subsystems.color.patterns;

import frc.robot.subsystems.color.RGB;

public class BIFlag {
  public static int numberOfLEDS;
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.BLACK, 2),
    new PatternNode(RGB.PINK, 3),
    new PatternNode(RGB.PURPLE, 3),
    new PatternNode(RGB.BLUE, 3),
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
