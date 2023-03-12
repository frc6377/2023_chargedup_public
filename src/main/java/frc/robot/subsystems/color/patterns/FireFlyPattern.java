package frc.robot.subsystems.color.patterns;

import frc.robot.subsystems.color.RGB;

public class FireFlyPattern {
  public static int numberOfLEDS;
  private static final RGB[] pattern = {
    RGB.GREEN, RGB.GREEN, RGB.GREEN, RGB.GREEN, RGB.GREEN, RGB.GREEN, RGB.WHITE, RGB.WHITE,
    RGB.WHITE, RGB.WHITE, RGB.WHITE, RGB.WHITE,
  };

  public static RGB[] getColors(int step) {
    int initalStep = step % pattern.length;
    RGB[] fullPattern = new RGB[numberOfLEDS];
    for (int i = 0; i < numberOfLEDS; i++) {
      fullPattern[i] = pattern[(i + initalStep) % pattern.length];
    }
    return fullPattern;
  }
}
