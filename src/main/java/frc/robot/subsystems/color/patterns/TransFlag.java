package frc.robot.subsystems.color.patterns;

import frc.robot.subsystems.color.RGB;

public class TransFlag {
  public static int numberOfLEDS;
  private static final RGB[] pattern = {
    RGB.BLACK,
    RGB.HOWDY_BLUE,
    RGB.HOWDY_BLUE,
    RGB.HOWDY_BLUE,
    RGB.HOWDY_BLUE,
    RGB.HOWDY_BLUE,
    RGB.PINK,
    RGB.PINK,
    RGB.PINK,
    RGB.PINK,
    RGB.PINK,
    RGB.WHITE,
    RGB.WHITE,
    RGB.WHITE,
    RGB.WHITE,
    RGB.WHITE,
    RGB.PINK,
    RGB.PINK,
    RGB.PINK,
    RGB.PINK,
    RGB.PINK,
    RGB.HOWDY_BLUE,
    RGB.HOWDY_BLUE,
    RGB.HOWDY_BLUE,
    RGB.HOWDY_BLUE,
    RGB.HOWDY_BLUE,
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
