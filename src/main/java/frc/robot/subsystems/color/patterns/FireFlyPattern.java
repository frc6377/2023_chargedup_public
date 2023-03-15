package frc.robot.subsystems.color.patterns;

import frc.robot.subsystems.color.RGB;

public class FireFlyPattern {
  public static int numberOfLEDS;
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.GREEN, 5), new PatternNode(RGB.WHITE, 5)
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
