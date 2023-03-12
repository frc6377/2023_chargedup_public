package frc.robot.subsystems.color.patterns;

import frc.robot.subsystems.color.RGB;

public class BIFlag {
  public static int numberOfLEDS;
  private static final RGB[] pattern = {
    RGB.BLACK,
    RGB.PINK,
    RGB.PINK,
    RGB.PINK,
    RGB.PURPLE,
    RGB.PURPLE,
    RGB.BLUE,
    RGB.BLUE,
    RGB.BLUE,
    RGB.BLACK,
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
