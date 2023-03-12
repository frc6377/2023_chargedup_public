package frc.config;

import frc.robot.Constants;

public enum RobotVersion {
  V1(Constants.V1_MAC_ADDRESS),
  V2(Constants.V2_MAC_ADDRESS);

  private RobotVersion(final String rioMacAddress) {}

  public static RobotVersion getVersionForMacAddress(final String actualMacAddress) {
    System.out.println(actualMacAddress);
    switch (actualMacAddress) {
      case Constants.V2_MAC_ADDRESS:
        System.out.println("V2 " + Constants.V2_MAC_ADDRESS);
        return V2;
      default:
        System.out.println("V1");
        return V1;
    }
  }
}
